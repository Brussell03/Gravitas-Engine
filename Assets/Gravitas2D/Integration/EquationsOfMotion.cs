using Unity.Mathematics;
using Unity.Burst;

namespace Gravitas2D
{
	[BurstCompile]
	public static class EquationsOfMotion
	{
		[BurstCompile]
		public static void CalculateDerivative(
			double time,
			in StateVector x,
			in VehicleModel vehicleModel,
			in double2 gravity_w,
			in double2 externalForce,
			in double externalMoment,
			out StateDerivative dx)
		{
			double2 vel_w = x.Velocity;
			float omega_b = x.AngularVelocity; 

			// --- Forces ---

			// Add Gravity + World External Forces
			double2 accel_w = new double2(
				gravity_w.x + (externalForce.x * vehicleModel.InvMass),
				gravity_w.y + (externalForce.y * vehicleModel.InvMass)
			);

			// --- Moments ---
			
			// Total Moments and Angular Acceleration (Torque is frame-invariant in 2D)
			float angularAccel = (float)(externalMoment * vehicleModel.InvInertiaTensor);

			// --- State Derivative Output ---
			dx = new StateDerivative
			{
				Velocity = vel_w,
				Acceleration = accel_w,
				AngularVelocity = omega_b,
				AngularAcceleration = angularAccel
			};
		}
	}
}