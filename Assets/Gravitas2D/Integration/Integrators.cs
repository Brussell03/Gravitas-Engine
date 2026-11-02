using Unity.Mathematics;
using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;

namespace Gravitas2D
{
	[BurstCompile]
	public static class Integrators
	{
		[BurstCompile]
		public struct IntegrateJobVelocityVerlet : IJobParallelFor
		{
			[ReadOnly] public float Time;
			[ReadOnly] public float TimeStep;
			[ReadOnly] public double CellSize;

			[ReadOnly] public NativeArray<GravitySourceData> GravitationalBodies;
			[ReadOnly] public NativeList<int> GravitationalBodyIndices;
			[ReadOnly] public NativeParallelMultiHashMap<int, int> SpatialMap;
			[ReadOnly] public NativeArray<StateVector> BodyStateVectors;
			[ReadOnly] public NativeArray<VehicleModel> BodyVehicleModels;
			[ReadOnly] public NativeArray<double2> BodyForces;
			[ReadOnly] public NativeArray<double> BodyMoments;
			[ReadOnly] public NativeArray<bool> CollisionTriggerMap;
			[ReadOnly] public NativeArray<bool> CollidableMap;
			
			[WriteOnly, NativeDisableParallelForRestriction] public NativeArray<StateVector> NextStateVectors;
			[WriteOnly] public NativeQueue<CollisionEvent>.ParallelWriter CollisionQueue;

			public void Execute(int index)
			{
				StateVector currentStateVector = BodyStateVectors[index];
				VehicleModel currentVehicleModel = BodyVehicleModels[index];
				double2 currentForce = BodyForces[index];
				double currentMoment = BodyMoments[index];

				VelocityVerlet(
					currentStateVector,
					currentVehicleModel,
					currentForce,
					currentMoment,
					GravitationalBodies,
					GravitationalBodyIndices,
					Time,
					TimeStep,
					index,
					out StateVector nextState
				);

				NextStateVectors[index] = nextState;
				CollisionDetection.EvaluateCollisions(index, currentStateVector, currentVehicleModel, nextState, CollisionTriggerMap, CollidableMap, TimeStep, CellSize, BodyStateVectors, BodyVehicleModels, SpatialMap, ref CollisionQueue);
			}
		}

		[BurstCompile]
		public struct IntegrateJobRK4 : IJobParallelFor
		{
			[ReadOnly] public float Time;
			[ReadOnly] public float TimeStep;
			[ReadOnly] public double CellSize;

			[ReadOnly] public NativeArray<GravitySourceData> GravitationalBodies;
			[ReadOnly] public NativeList<int> GravitationalBodyIndices;
			[ReadOnly] public NativeParallelMultiHashMap<int, int> SpatialMap;
			[ReadOnly] public NativeArray<StateVector> BodyStateVectors;
			[ReadOnly] public NativeArray<VehicleModel> BodyVehicleModels;
			[ReadOnly] public NativeArray<double2> BodyForces;
			[ReadOnly] public NativeArray<double> BodyMoments;
			[ReadOnly] public NativeArray<bool> CollisionTriggerMap;
			[ReadOnly] public NativeArray<bool> CollidableMap;
			
			[WriteOnly, NativeDisableParallelForRestriction] public NativeArray<StateVector> NextStateVectors;
			[WriteOnly] public NativeQueue<CollisionEvent>.ParallelWriter CollisionQueue;

			public void Execute(int index)
			{
				StateVector currentStateVector = BodyStateVectors[index];
				VehicleModel currentVehicleModel = BodyVehicleModels[index];
				double2 currentForce = BodyForces[index];
				double currentMoment = BodyMoments[index];

				RungeKutta4(
					currentStateVector,
					currentVehicleModel,
					currentForce,
					currentMoment,
					GravitationalBodies,
					GravitationalBodyIndices,
					Time,
					TimeStep,
					index,
					out StateVector nextState
				);

				NextStateVectors[index] = nextState;
				CollisionDetection.EvaluateCollisions(index, currentStateVector, currentVehicleModel, nextState, CollisionTriggerMap, CollidableMap, TimeStep, CellSize, BodyStateVectors, BodyVehicleModels, SpatialMap, ref CollisionQueue);
			}
		}

		/// <summary>
		/// Performs 4th-order Runge-Kutta integration to solve a system of ODEs.
		/// </summary>
		[BurstCompile]
		public static void RungeKutta4(
			in StateVector stateVector,
			in VehicleModel vehicleModel,
			in double2 force,
			in double moment,
			in NativeArray<GravitySourceData> gravitationalBodies,
			in NativeList<int> gravitationalBodiesIndices,
			float time,
			float timeStep,
			int physicsBodyIndex,
			out StateVector integratedState)
		{
			double timeStep_d = (double)timeStep;

			GravitationalAcceleration.CalculateGravitationalAcceleration(stateVector.Position, gravitationalBodiesIndices, gravitationalBodies, physicsBodyIndex, out double2 currentGravity);
			EquationsOfMotion.CalculateDerivative(time, stateVector, vehicleModel, currentGravity, force, moment, out StateDerivative k1);
			CalculateIntermediateState(stateVector, k1, timeStep * 0.5f, out StateVector state_k2);

			GravitationalAcceleration.CalculateGravitationalAcceleration(state_k2.Position, gravitationalBodiesIndices, gravitationalBodies, physicsBodyIndex, out currentGravity);
			EquationsOfMotion.CalculateDerivative(time + 0.5f * timeStep, state_k2, vehicleModel, currentGravity, force, moment, out StateDerivative k2);
			CalculateIntermediateState(stateVector, k2, timeStep * 0.5f, out StateVector state_k3);

			GravitationalAcceleration.CalculateGravitationalAcceleration(state_k3.Position, gravitationalBodiesIndices, gravitationalBodies, physicsBodyIndex, out currentGravity);
			EquationsOfMotion.CalculateDerivative(time + 0.5f * timeStep, state_k3, vehicleModel, currentGravity, force, moment, out StateDerivative k3);
			CalculateIntermediateState(stateVector, k3, timeStep, out StateVector state_k4);

			GravitationalAcceleration.CalculateGravitationalAcceleration(state_k4.Position, gravitationalBodiesIndices, gravitationalBodies, physicsBodyIndex, out currentGravity);
			EquationsOfMotion.CalculateDerivative(time + timeStep, state_k4, vehicleModel, currentGravity, force, moment, out StateDerivative k4);

			// Linear components (Position and Velocity)
			double2 positionChange = (timeStep_d / 6.0) * (k1.Velocity + 2.0 * k2.Velocity + 2.0 * k3.Velocity + k4.Velocity);
			double2 velocityChange = (timeStep_d / 6.0) * (k1.Acceleration + 2.0 * k2.Acceleration + 2.0 * k3.Acceleration + k4.Acceleration);

			// Angular Velocity - average the angular accelerations
			float angularVelocityChange = (timeStep / 6.0f) * (k1.AngularAcceleration + 2.0f * k2.AngularAcceleration + 2.0f * k3.AngularAcceleration + k4.AngularAcceleration);
			float nextAngularVelocity = stateVector.AngularVelocity + angularVelocityChange;

			// Orientation - average the angular velocities and integrate
			// Effective angular velocity for the step (weighted average)
			float effectiveAngularVelocity = (k1.AngularVelocity + 2.0f * k2.AngularVelocity + 2.0f * k3.AngularVelocity + k4.AngularVelocity) / 6.0f;
			float nextOrientation = WrapAngle(stateVector.Orientation + effectiveAngularVelocity * timeStep);

			// Calculate and store the next state in the array
			integratedState = new StateVector
			{
				Position = stateVector.Position + positionChange,
				Velocity = stateVector.Velocity + velocityChange,
				Orientation = nextOrientation,
				AngularVelocity = nextAngularVelocity
			};
		}
		
		/// <summary>
		/// Performs Velocity Verlet integration for a 6DOF system.
		/// </summary>
		[BurstCompile]
		public static void VelocityVerlet(
			in StateVector stateVector,
			in VehicleModel vehicleModel,
			in double2 force,
			in double moment,
			in NativeArray<GravitySourceData> gravitationalBodies,
			in NativeList<int> gravitationalBodiesIndices,
			float time,
			float timeStep,
			int bodyIndex,
			out StateVector integratedState)
		{
			double timeStep_d = (double)timeStep;

			GravitationalAcceleration.CalculateGravitationalAcceleration(stateVector.Position, gravitationalBodiesIndices, gravitationalBodies, bodyIndex, out double2 currentGravity);
			EquationsOfMotion.CalculateDerivative(time, stateVector, vehicleModel, currentGravity, force, moment, out StateDerivative currentDerivative);
			
			double2 currentAccel = currentDerivative.Acceleration;
			float currentAngularAccel = currentDerivative.AngularAcceleration;

			// Half-step velocities
			double2 velocityHalfStep = stateVector.Velocity + 0.5 * timeStep_d * currentAccel;
			float angularVelocityHalfStep = stateVector.AngularVelocity + 0.5f * timeStep * currentAngularAccel;

			// Full-step positions/orientations
			double2 nextPosition = stateVector.Position + timeStep_d * velocityHalfStep;
			float nextOrientation = WrapAngle(stateVector.Orientation + angularVelocityHalfStep * timeStep);

			StateVector stateAtNextPos = new StateVector
			{
				Position = nextPosition,
				Velocity = velocityHalfStep, 
				Orientation = nextOrientation,
				AngularVelocity = angularVelocityHalfStep
			};

			GravitationalAcceleration.CalculateGravitationalAcceleration(nextPosition, gravitationalBodiesIndices, gravitationalBodies, bodyIndex, out currentGravity);
			EquationsOfMotion.CalculateDerivative(time + timeStep, stateAtNextPos, vehicleModel, currentGravity, force, moment, out StateDerivative nextDerivative);
			
			double2 nextAccel = nextDerivative.Acceleration;
			float nextAngularAccel = nextDerivative.AngularAcceleration;

			double2 nextVelocity = velocityHalfStep + 0.5 * timeStep_d * nextAccel;
			float nextAngularVelocity = angularVelocityHalfStep + 0.5f * timeStep * nextAngularAccel;

			integratedState = new StateVector
			{
				Position = nextPosition,
				Velocity = nextVelocity,
				Orientation = nextOrientation,
				AngularVelocity = nextAngularVelocity
			};
		}
		
		// Helper to calculate intermediate states including quaternion update
		[BurstCompile]
		private static void CalculateIntermediateState(in StateVector current, in StateDerivative k, float dt, out StateVector interState)
		{
			double dt_d = (double)dt;

			interState = new StateVector
			{
				Position = current.Position + dt_d * k.Velocity,
				Velocity = current.Velocity + dt_d * k.Acceleration,
				Orientation = current.Orientation + dt * k.AngularVelocity,
				AngularVelocity = current.AngularVelocity + dt * k.AngularAcceleration
			};
		}

		[BurstCompile]
		private static float WrapAngle(float angle)
		{
			angle = (angle + math.PI) % (2.0f * math.PI);
			if (angle < 0f) angle += 2.0f * math.PI;
			return angle - math.PI;
		}
	}
}