using UnityEngine;
using Unity.Mathematics;
using Unity.Burst;

namespace Gravitas
{
	[BurstCompile]
	public static class EquationsOfMotion
	{

		[BurstCompile]
		public static void CalculateDerivative(
			float time,
			in StateVector x,
			in VehicleModel vmod,
			in double3 gravity_w,
			in float3 externalForce,
			in float3 externalMoment,
			out StateDerivative dx)
		{
			// --- 1. Extract Current State ---
			double3 pos_w = x.position;         // World position
			double3 vel_w = x.velocity;         // World velocity
			quaternion rot_w = x.orientation;   // World orientation
			float3 omega_b = x.angularVelocity; // Body-frame angular rates (p, q, r)

			// --- 2. Calculate Altitude & Atmospheric Properties ---
			double dist_from_center = math.length(pos_w);
			//double altitude = dist_from_center;
			//float rho = amod.density;

			// --- 3. Calculate Air Data (relative to air, assuming no wind for simplicity) ---
			// Transform world velocity to body velocity
			float3 vel_b = math.rotate(math.inverse(rot_w), (float3)vel_w); // Casting to float should be okay here, due to lower speeds in atmosphere

			float u_b = vel_b.x;
			float v_b = vel_b.y;
			float w_b = vel_b.z;

			float V_airspeed = math.length(vel_b);
			//float qbar = 0.5f * rho * V_airspeed * V_airspeed; // Dynamic pressure

			// Angle of Attack (alpha) and Sideslip (beta)
			float alpha = (math.abs(u_b) < 1e-6f) ? 0f : math.atan2(w_b, u_b); // atan2 handles quadrants
			float beta = (V_airspeed < 1e-6f) ? 0f : math.asin(v_b / V_airspeed);

			// --- 4. Calculate Forces ---
			// Gravity (Round Earth) - acting towards planet center in world frame
			// double grav_mag = amod.mu / (dist_from_center * dist_from_center);
			// double3 gravity_w = (dist_from_center > 1e-6) ? -math.normalize(x.position) * grav_mag : double3.zero;

			// Transform world gravity to body frame
			//double3 gravity_b = math.rotate(math.inverse(rot_w), (float3)gravity_w); // Need to fix this casting issue again

			// Aerodynamic Forces (Wind Axes -> Body Axes)
			// Implement CD, CL, CY lookup based on alpha, beta, Mach etc. here if needed.
			float F_drag = 0f; // Example: = CD * qbar * vmod.A_ref_m2;
			float F_side = 0f; // Example: = CY * qbar * vmod.A_ref_m2;
			float F_lift = 0f; // Example: = CL * qbar * vmod.A_ref_m2;

			// Wind-to-Body transformation (using alpha, beta)
			float sa = math.sin(alpha);
			float ca = math.cos(alpha);
			float sb = math.sin(beta);
			float cb = math.cos(beta);

			// Force components in body frame from aero (assuming standard wind axis definition)
			float Fx_aero_b = -F_drag * ca * cb + F_side * ca * sb + F_lift * sa;
			float Fy_aero_b = -F_drag * sb - F_side * cb;
			float Fz_aero_b = -F_drag * sa * cb + F_side * sa * sb - F_lift * ca;

			// Thrust Forces (Add if needed, resolve in body frame)
			float Fx_thrust_b = 0f;
			float Fy_thrust_b = 0f;
			float Fz_thrust_b = 0f;

			// Linear Damping forces
			float Fx_damping_b = -vmod.angularDamping.x * u_b;
			float Fy_damping_b = -vmod.angularDamping.y * v_b;
			float Fz_damping_b = -vmod.angularDamping.z * w_b;

			// Thrust Forces (Add if needed, resolve in body frame)
			float Fx_external_b = externalForce.x;
			float Fy_external_b = externalForce.y;
			float Fz_external_b = externalForce.z;

			// Total Forces in Body Frame
			double3 totalForce_b = new double3(
				Fx_aero_b + Fx_thrust_b + Fx_damping_b + Fx_external_b,
				Fy_aero_b + Fy_thrust_b + Fy_damping_b + Fy_external_b,
				Fz_aero_b + Fz_thrust_b + Fz_damping_b + Fz_external_b
			);

			// --- 5. Calculate Linear Acceleration ---
			// Acceleration in Body Frame
			double3 accel_b = totalForce_b * vmod.invMass;

			// Transform Body Acceleration to World Acceleration
			//double3 accel_w = math.rotate(rot_w, (float3)accel_b) + gravity_w; // Need to fix this casting issue again
			RotateDouble3(rot_w, accel_b, out double3 accel_w);
			accel_w += gravity_w;

			// --- 6. Calculate Moments ---
			float p_b = omega_b.x;
			float q_b = omega_b.y;
			float r_b = omega_b.z;

			// Aerodynamic Moments (Body Frame)
			// float L_aero_b = AeroCoefficients.Cl_brick(vmod.Clp, vmod.Clr, p_b, r_b, vmod.b_m, V_airspeed) * qbar * vmod.A_ref_m2 * vmod.b_m;
			// float M_aero_b = AeroCoefficients.Cm_brick(vmod.Cmq, q_b, vmod.c_m, V_airspeed) * qbar * vmod.A_ref_m2 * vmod.c_m;
			// float N_aero_b = AeroCoefficients.Cn_brick(vmod.Cnp, vmod.Cnr, p_b, r_b, vmod.b_m, V_airspeed) * qbar * vmod.A_ref_m2 * vmod.b_m;
			float L_aero_b = 0f;
			float M_aero_b = 0f;
			float N_aero_b = 0f;

			// Thrust Moments (Add if needed, resolve in body frame)
			float L_thrust_b = 0f;
			float M_thrust_b = 0f;
			float N_thrust_b = 0f;

			// Angular Damping Moments
			float L_damping_b = -vmod.angularDamping.x * p_b;
			float M_damping_b = -vmod.angularDamping.y * q_b;
			float N_damping_b = -vmod.angularDamping.z * r_b;

			// External Moments
			float L_external_b = externalMoment.x;
			float M_external_b = externalMoment.y;
			float N_external_b = externalMoment.z;

			// Total Moments in Body Frame
			float L_total_b = L_aero_b + L_thrust_b + L_damping_b + L_external_b;
			float M_total_b = M_aero_b + M_thrust_b + M_damping_b + M_external_b;
			float N_total_b = N_aero_b + N_thrust_b + N_damping_b + N_external_b;

			// --- 7. Calculate Angular Acceleration (Euler's Equations) ---
			// Using inertia tensor components (assuming Jxy = Jyx = Jyz = Jzy = 0 for simplicity, except Jxz)
			// float Jxx = vmod.InertiaTensor.c0.x;
			// float Jyy = vmod.InertiaTensor.c1.y;
			// float Jzz = vmod.InertiaTensor.c2.z;
			// float Jxz = vmod.InertiaTensor.c0.z;

			// Precompute Gamma_inv if using the specific Python formulation (matrix inversion is more general)
			// double Gamma_inv = 1.0 / (Jxx * Jzz - Jxz * Jxz);
			// dx[3] = (Jxz * (Jxx - Jyy + Jzz) * p_b * q_b - (Jzz * (Jzz - Jyy) + Jxz*Jxz) * q_b * r_b + Jzz * L_total_b + Jxz * N_total_b) * Gamma_inv;
			// dx[4] = ((Jzz - Jxx) * p_b * r_b - Jxz * (p_b*p_b - r_b*r_b) + M_total_b) / Jyy;
			// dx[5] = ((Jxx * (Jxx - Jyy) + Jxz*Jxz) * p_b * q_b - Jxz * (Jxx - Jyy + Jzz) * q_b * r_b + Jxz * L_total_b + Jxx * N_total_b) * Gamma_inv;

			// Alternative formulation using matrix inversion (more general, easier if J matrix is available)
			// Inertia tensor matrix (example, assuming principal axes align with body)
			// If not, use the full matrix:
			// float3x3 InertiaTensor = new float3x3(
			// 	Jxx, 0, -Jxz,
			// 	0, Jyy, 0,
			// 	-Jxz, 0, Jzz
			// );

			// Angular momentum H = J * omega
			float3 H_b = math.mul(vmod.InertiaTensor, omega_b);
			// Rate of change of H = moments - omega x H
			float3 H_dot_b = new float3(L_total_b, M_total_b, N_total_b) - math.cross(omega_b, H_b);
			// Angular acceleration alpha = J^-1 * H_dot
			float3 angularAccel_b = math.mul(vmod.InvInertiaTensor, H_dot_b);


			// --- 8. Assemble State Derivative Output ---
			dx = new StateDerivative
			{
				// Linear Derivatives (World Frame)
				velocity = vel_w,              // d(pos_w)/dt = vel_w
				acceleration = accel_w,          // d(vel_w)/dt = accel_w

				// Angular Derivatives (Body Frame - used by integrator helpers)
				angularVelocity = omega_b,         // Passed through for quaternion integration
				angularAcceleration = angularAccel_b // d(omega_b)/dt = angularAccel_b
			};
		}


		[BurstCompile]
		public static void RotateDouble3(in quaternion q, in double3 v, out double3 v_rot)
		{
			double qx = (double)q.value.x;
			double qy = (double)q.value.y;
			double qz = (double)q.value.z;
			double qw = (double)q.value.w;

			double sqx = qx * qx;
			double sqy = qy * qy;
			double sqz = qz * qz;
			double qxy = qx * qy;
			double qzw = qz * qw;
			double qxz = qx * qz;
			double qyw = qy * qw;
			double qyz = qy * qz;
			double qxw = qx * qw;

			// Apply the standard (non-transposed) rotation matrix: R_q * v
			// Note the sign differences compared to the inverse version
			v_rot.x = (1.0 - 2.0 * (sqy + sqz)) * v.x + (2.0 * (qxy - qzw)) * v.y + (2.0 * (qxz + qyw)) * v.z;
			v_rot.y = (2.0 * (qxy + qzw)) * v.x + (1.0 - 2.0 * (sqx + sqz)) * v.y + (2.0 * (qyz - qxw)) * v.z;
			v_rot.z = (2.0 * (qxz - qyw)) * v.x + (2.0 * (qyz + qxw)) * v.y + (1.0 - 2.0 * (sqx + sqy)) * v.z;
		}
	}
}
