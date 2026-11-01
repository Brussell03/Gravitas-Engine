using UnityEngine;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;
using Unity.Physics;

namespace Gravitas
{
	[BurstCompile]
	public static class Integrators
	{
		public delegate void CalculateGravitationAcceleration(in double3 position, in NativeList<int> gravitationalBodiesPhysicsIndices, in NativeArray<PhysicsBodyData> gravitationalBodies, int physicsBodyIndex, out double3 gravity_w);
		public delegate void DerivativeFunction(float time, in StateVector stateVector, in VehicleModel vmod, in double3 gravity_w, in float3 externalForce, in float3 externalMoment,out StateDerivative dx);
		public delegate void DerivativeFunctionPtr(float time, in StateVector stateVector, in VehicleModel vmod, in double3 gravity_w, in float3 externalForce, in float3 externalMoment,out StateDerivative dx);

		[BurstCompile]
		public struct IntegrateJobVelocityVerlet : IJobParallelFor
		{
			[ReadOnly] public float time;
			[ReadOnly] public float timeStep;
			[ReadOnly] public FunctionPointer<DerivativeFunctionPtr> derivativeFuncPtr;
			[ReadOnly] public FunctionPointer<CalculateGravitationAcceleration> gravAccelFuncPtr;
			[ReadOnly] public NativeArray<PhysicsBodyData> GravitationalBodies;
			[ReadOnly] public NativeList<int> GravitationalBodiesPhysicsIndices;

			public NativeArray<PhysicsBodyData> Bodies;

			public void Execute(int index)
			{
				PhysicsBodyData currentBody = Bodies[index];

				Integrators.VelocityVerlet(
					derivativeFuncPtr,
					gravAccelFuncPtr,
					currentBody,
					GravitationalBodies,
					GravitationalBodiesPhysicsIndices,
					time,
					timeStep,
					index,
					out StateVector nextState
				);

				Bodies[index] = new PhysicsBodyData(nextState, currentBody.vmod, float3.zero, float3.zero);
			}
		}

		[BurstCompile]
		public struct IntegrateJobRK4 : IJobParallelFor
		{
			[ReadOnly] public float time;
			[ReadOnly] public float timeStep;
			[ReadOnly] public FunctionPointer<DerivativeFunctionPtr> derivativeFuncPtr;
			[ReadOnly] public FunctionPointer<CalculateGravitationAcceleration> gravAccelFuncPtr;
			[ReadOnly] public NativeArray<PhysicsBodyData> GravitationalBodies;
			[ReadOnly] public NativeList<int> GravitationalBodiesPhysicsIndices;

			public NativeArray<PhysicsBodyData> Bodies;

			public void Execute(int index)
			{
				PhysicsBodyData currentBody = Bodies[index];

				Integrators.RungeKutta4(
					derivativeFuncPtr,
					gravAccelFuncPtr,
					currentBody,
					GravitationalBodies,
					GravitationalBodiesPhysicsIndices,
					time,
					timeStep,
					index,
					out StateVector nextState
				);

				Bodies[index] = new PhysicsBodyData(nextState, currentBody.vmod, float3.zero, float3.zero);
			}
		}

		/// <summary>
		/// Performs 4th-order Runge-Kutta integration to solve a system of ODEs.
		/// </summary>
		/// <param name="f">The function representing the right-hand side of the differential equation (dx/dt = f(t, x, ...)).</param>
		/// <param name="time">The time point.</param>
		/// <param name="x">The solution state.</param>
		/// <param name="timeStep">The step size.</param>
		/// <param name="vmod">Vehicle model data.</param>
		/// <param name="amod">Atmospheric model data.</param>
		[BurstCompile]
		public static void RungeKutta4(
			FunctionPointer<DerivativeFunctionPtr> fPtr,
			FunctionPointer<CalculateGravitationAcceleration> gPtr,
			in PhysicsBodyData body,
			in NativeArray<PhysicsBodyData> gravitationalBodies,
			in NativeList<int> gravitationalBodiesPhysicsIndices,
			float time,
			float timeStep,
			int physicsBodyIndex,
			out StateVector integratedState)
		{
			StateVector x = body.stateVector;
			VehicleModel vmod = body.vmod;

			double timeStep_d = (double)timeStep;

			gPtr.Invoke(x.position, gravitationalBodiesPhysicsIndices, gravitationalBodies, physicsBodyIndex, out double3 currentGravity);
			fPtr.Invoke(time, x, vmod, currentGravity, body.externalForce, body.externalMoment, out StateDerivative k1);
			CalculateIntermediateState(x, k1, timeStep * 0.5f, out StateVector state_k2);

			gPtr.Invoke(state_k2.position, gravitationalBodiesPhysicsIndices, gravitationalBodies, physicsBodyIndex, out currentGravity);
			fPtr.Invoke(time + 0.5f * timeStep, state_k2, vmod, currentGravity, body.externalForce, body.externalMoment, out StateDerivative k2);
			CalculateIntermediateState(x, k2, timeStep * 0.5f, out StateVector state_k3);

			gPtr.Invoke(state_k3.position, gravitationalBodiesPhysicsIndices, gravitationalBodies, physicsBodyIndex, out currentGravity);
			fPtr.Invoke(time + 0.5f * timeStep, state_k3, vmod, currentGravity, body.externalForce, body.externalMoment, out StateDerivative k3);
			CalculateIntermediateState(x, k3, timeStep, out StateVector state_k4);

			gPtr.Invoke(state_k4.position, gravitationalBodiesPhysicsIndices, gravitationalBodies, physicsBodyIndex, out currentGravity);
			fPtr.Invoke(time + timeStep, state_k4, vmod, currentGravity, body.externalForce, body.externalMoment, out StateDerivative k4);

			// Linear components (Position and Velocity) - same as before
			double3 positionChange = (timeStep_d / 6.0) * (k1.velocity + 2.0 * k2.velocity + 2.0 * k3.velocity + k4.velocity);
			double3 velocityChange = (timeStep_d / 6.0) * (k1.acceleration + 2.0 * k2.acceleration + 2.0 * k3.acceleration + k4.acceleration);

			// Angular Velocity - average the angular accelerations
			float3 angularVelocityChange = (timeStep / 6.0f) * (k1.angularAcceleration + 2.0f * k2.angularAcceleration + 2.0f * k3.angularAcceleration + k4.angularAcceleration);
			float3 nextAngularVelocity = x.angularVelocity + angularVelocityChange;

			// Orientation - average the angular velocities and integrate quaternion
			// Effective angular velocity for the step (weighted average)
			float3 effectiveAngularVelocity = (k1.angularVelocity + 2.0f * k2.angularVelocity + 2.0f * k3.angularVelocity + k4.angularVelocity) / 6.0f;
			IntegrateQuaternion(x.orientation, effectiveAngularVelocity, timeStep, out quaternion orientationChange);
			quaternion nextOrientation = math.normalize(orientationChange); // Normalize after integration

			// Calculate and store the next state in the array
			integratedState = new StateVector
			{
				position = x.position + positionChange,
				velocity = x.velocity + velocityChange,
				orientation = nextOrientation,
				angularVelocity = nextAngularVelocity
			};
		}
		
		/// <summary>
		/// Performs Velocity Verlet integration for a 6DOF system.
		/// </summary>
		/// <param name="f">The function representing the right-hand side of the differential equation (dx/dt = f(t, x, ...)).</param>
		/// <param name="time">The time point.</param>
		/// <param name="x">The solution state.</param>
		/// <param name="timeStep">The step size.</param>
		/// <param name="vmod">Vehicle model data.</param>
		/// <param name="amod">Atmospheric model data.</param>
		[BurstCompile]
		public static void VelocityVerlet(
			FunctionPointer<DerivativeFunctionPtr> fPtr,
			FunctionPointer<CalculateGravitationAcceleration> gPtr,
			in PhysicsBodyData body,
			in NativeArray<PhysicsBodyData> gravitationalBodies,
			in NativeList<int> gravitationalBodiesPhysicsIndices,
			float time,
			float timeStep,
			int physicsBodyIndex,
			out StateVector integratedState)
		{
			StateVector x = body.stateVector;
			VehicleModel vmod = body.vmod;

			// --- Step 1: Calculate current accelerations ---
			gPtr.Invoke(x.position, gravitationalBodiesPhysicsIndices, gravitationalBodies, physicsBodyIndex, out double3 currentGravity);
			fPtr.Invoke(time, x, vmod, currentGravity, body.externalForce, body.externalMoment, out StateDerivative currentDerivative);
			double3 currentAccel = currentDerivative.acceleration;
			float3 currentAngularAccel = currentDerivative.angularAcceleration;

			// --- Step 2: Calculate half-step velocities ---
			// Linear (double)
			double timeStep_d = (double)timeStep;
			double3 velocityHalfStep = x.velocity + 0.5 * timeStep_d * currentAccel;
			// Angular (float)
			float3 angularVelocityHalfStep = x.angularVelocity + 0.5f * timeStep * currentAngularAccel;

			// --- Step 3: Calculate full-step positions/orientations ---
			// Linear position (double)
			double3 nextPosition = x.position + timeStep_d * velocityHalfStep;
			// Angular orientation (float) - Rotate by average angular velocity (approximated by half step velocity)
			IntegrateQuaternion(x.orientation, angularVelocityHalfStep, timeStep, out quaternion nextOrientation);
			nextOrientation = math.normalize(nextOrientation);

			// --- Create intermediate state for calculating next acceleration ---
			// We use the NEW position/orientation but keep OLD velocities for now (or half-step?)
			// Standard Verlet uses the new position/orientation.
			StateVector stateAtNextPos = new StateVector
			{
				position = nextPosition,
				velocity = velocityHalfStep, // Use half-step velocity temporarily
				orientation = nextOrientation,
				angularVelocity = angularVelocityHalfStep // Use half-step angular velocity temporarily
			};

			// --- Step 4: Calculate accelerations at the new position/orientation ---
			gPtr.Invoke(nextPosition, gravitationalBodiesPhysicsIndices, gravitationalBodies, physicsBodyIndex, out currentGravity);
			fPtr.Invoke(time + timeStep, stateAtNextPos, vmod, currentGravity, body.externalForce, body.externalMoment, out StateDerivative nextDerivative);
			double3 nextAccel = nextDerivative.acceleration;
			float3 nextAngularAccel = nextDerivative.angularAcceleration;

			// --- Step 5: Calculate full-step velocities ---
			// Linear (double)
			double3 nextVelocity = velocityHalfStep + 0.5 * timeStep_d * nextAccel;
			// Angular (float)
			float3 nextAngularVelocity = angularVelocityHalfStep + 0.5f * timeStep * nextAngularAccel;

			// --- Return the final state ---
			integratedState = new StateVector
			{
				position = nextPosition,
				velocity = nextVelocity,
				orientation = nextOrientation,
				angularVelocity = nextAngularVelocity
			};
		}
		
		// Helper to calculate intermediate states including quaternion update
		[BurstCompile] private static void CalculateIntermediateState(in StateVector current, in StateDerivative k, float dt, out StateVector interState)
		{
			IntegrateQuaternion(current.orientation, k.angularVelocity, dt, out quaternion orientationChange);

			double dt_d = (double)dt;

			interState = new StateVector
			{
				position = current.position + dt_d * k.velocity,
				velocity = current.velocity + dt_d * k.acceleration,
				orientation = math.normalize(orientationChange), // Normalize intermediate steps too
				angularVelocity = current.angularVelocity + dt * k.angularAcceleration
			};
		}

		// Helper function to integrate quaternion orientation
		// q_next = q_current + 0.5 * dt * Omega(omega) * q_current
		// where Omega(omega) is a matrix form, simplified below
		[BurstCompile] private static void IntegrateQuaternion(in quaternion q, in float3 omega, float dt, out quaternion q_rot)
		{
			// Small rotation quaternion from angular velocity vector
			float angle = math.length(omega) * dt;
			if (angle < 1e-9) // Avoid division by zero if no rotation
			{
				q_rot = q;
			}
			float3 axis = math.normalize(omega);
			quaternion deltaRotation = quaternion.AxisAngle(axis, angle);

			// Apply the rotation: q_next = q_current * deltaRotation
            // This assumes deltaRotation represents a rotation in the local (body) frame.
            
            // --- FIX ---
            // return math.mul(deltaRotation, q); // OLD (Incorrect: applies as world rotation)
            q_rot = math.mul(q, deltaRotation); // NEW (Correct: applies as body-space rotation)
		}
	}
}