using Unity.Mathematics;
using Unity.Burst;

namespace Gravitas2D
{
	[BurstCompile]
	public static class EquationsOfMotion
	{
		[BurstCompile]
		public static void CalculateDerivative(
			float time,
			in StateVector x,
			in VehicleModel vehicleModel,
			in double2 gravity_w,
			in double2 externalForce,
			in double externalMoment,
			out StateDerivative dx)
		{
			double2 vel_w = x.Velocity;
			float rot_w = x.Orientation;
			float omega_b = x.AngularVelocity; 

			// Precompute trigonometric bounds for vector rotation
			math.sincos(rot_w, out float s_r_f, out float c_r_f);
			double c_r = c_r_f;
			double s_r = s_r_f;

			// Transform world velocity to body velocity
			double u_b = vel_w.x * c_r + vel_w.y * s_r;
			double v_b = -vel_w.x * s_r + vel_w.y * c_r;

			// --- Forces ---
			
			// Linear Damping
			double Fx_damping_b = -vehicleModel.LinearDamping.x * u_b;
			double Fy_damping_b = -vehicleModel.LinearDamping.y * v_b;

			// Thrust Forces (Body Frame)
			double Fx_thrust_b = 0f;
			double Fy_thrust_b = 0f;

			// Total Internal Forces in Body Frame
			double2 internalForce_b = new double2(
				Fx_damping_b + Fx_thrust_b,
				Fy_damping_b + Fy_thrust_b
			);

			// Internal Acceleration in Body Frame
			double2 internalAccel_b = internalForce_b * vehicleModel.InvMass;

			// Transform Body Acceleration to World Acceleration and add Gravity + World External Forces
			double2 accel_w = new double2(
				internalAccel_b.x * c_r - internalAccel_b.y * s_r + gravity_w.x + (externalForce.x * vehicleModel.InvMass),
				internalAccel_b.x * s_r + internalAccel_b.y * c_r + gravity_w.y + (externalForce.y * vehicleModel.InvMass)
			);

			// --- Moments ---
			
			// Thrust Moments (Body Frame)
			double thrustMoment_b = 0f;

			// Angular Damping Moments
			double dampingMoment_b = -vehicleModel.AngularDamping * omega_b;
			
			// Total Moments and Angular Acceleration (Torque is frame-invariant in 2D)
			double totalMoment = externalMoment + thrustMoment_b + dampingMoment_b;
			float angularAccel = (float)(totalMoment * vehicleModel.InvInertiaTensor);

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