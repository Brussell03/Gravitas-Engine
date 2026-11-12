using System.Collections.Generic;
using Gravitas2D;
using Unity.Mathematics;

namespace GravitasDemo
{
	public class InterceptController
	{
		public bool IsEnabled { get; private set; }
		private NavigationCapability _manager;

		public double2 TargetOffset { get; set; }
		public float ArrivalThreshold { get; set; } = 10.0f; // Meters

		private float _lastFlightTime = -1f;

		public InterceptController(NavigationCapability manager)
		{
			_manager = manager;
		}

		public void Clear()
		{
			IsEnabled = false;
			_lastFlightTime = -1f;
			TargetOffset = double2.zero;
		}

		public bool CheckCompleted()
		{
			// Use the actual guidance aim point (including rendezvous clearance) and its
			// matching velocity. The previous check used the target centre for entity targets
			// and compared position against velocity for position targets.
			double distanceToTarget = math.distance(_manager.OwnerEntity.Position, _manager.TargetPosition);
			double relativeSpeed = math.distance(_manager.OwnerEntity.Velocity, _manager.TargetVelocity);

			return distanceToTarget <= ArrivalThreshold;
		}

		/// <summary>
		/// Statically computes the gravity-compensated quartic time of flight for an intercept trajectory.
		/// </summary>
		public static float ComputeInterceptTimeOfFlight(double2 currentPos, double2 currentVel, double2 targetPos, double2 targetVel, double mass, float maxForce, double mu, PhysicsBody parentSoI, float initialGuess = -1f)
		{
			double2 R = targetPos - currentPos;
			double2 V = targetVel - currentVel;
			double distance = math.length(R);

			if (distance <= 1.0 || double.IsNaN(distance)) return 0f;

			float acceleration = (float)(maxForce / mass);
			if (acceleration <= 0f || float.IsNaN(acceleration)) return 0f;

			double2 gravityAcc = double2.zero;
			if (parentSoI != null)
			{
				double2 rVec = currentPos - parentSoI.Position;
				double rDist = math.length(rVec);
				if (rDist > 1.0)
				{
					gravityAcc = -mu * rVec / (rDist * rDist * rDist);
				}
			}
			else
			{
				double rDist = math.length(currentPos);
				if (rDist > 1.0)
				{
					gravityAcc = -mu * currentPos / (rDist * rDist * rDist);
				}
			}

			float t = (initialGuess > 0f && distance > 10.0) ? initialGuess : (float)math.sqrt(2.0 * distance / acceleration);
			if (float.IsNaN(t) || float.IsInfinity(t) || t <= 0.001f) t = 0.01f;

			bool converged = false;
			float bestT = t;

			for (int i = 0; i < 12; i++)
			{
				if (double.IsNaN(t) || double.IsInfinity(t) || t <= 0.0) break;

				bestT = t;
				double2 R_corrected = R - (0.5 * gravityAcc * (t * t));

				double V_sq = math.lengthsq(V);
				double R_dot_V = math.dot(R_corrected, V);
				double R_sq = math.lengthsq(R_corrected);

				double t2 = t * t;
				double t3 = t2 * t;
				double t4 = t2 * t2;

				double f = 0.25 * acceleration * acceleration * t4 - V_sq * t2 - 2.0 * R_dot_V * t - R_sq;
				double df = acceleration * acceleration * t3 - 2.0 * V_sq * t - 2.0 * R_dot_V;

				if (math.abs(df) < 1e-6) break;

				float dtStep = (float)(f / df);
				if (float.IsNaN(dtStep) || float.IsInfinity(dtStep)) break;

				t -= dtStep;

				if (math.abs(dtStep) < 1e-4 || i == 11)
				{
					converged = true;
					if (t <= 0.001f) t = 0.001f;
					bestT = t;
					break;
				}
			}

			if (!converged && bestT > 0f && !float.IsNaN(bestT) && !float.IsInfinity(bestT))
			{
				t = bestT;
			}

			if (converged && t > 0f && !float.IsNaN(t) && !float.IsInfinity(t))
			{
				return t;
			}

			return (float)math.sqrt(2.0 * distance / acceleration);
		}

		/// <summary>
		/// Calculates the required burn orientation using a gravity-compensated quartic time-of-flight solver.
		/// </summary>
		public bool ComputeInterceptTrajectory(out float timeOfFlight, out float burnOrientation)
		{
			burnOrientation = 0f;
			timeOfFlight = _lastFlightTime > 0f ? _lastFlightTime : 1.0f;

			double mass = _manager.OwnerEntity.Mass;
			double2 currentPos = _manager.OwnerEntity.Position;
			double2 currentVel = _manager.OwnerEntity.Velocity;
			double mu = _manager.OwnerEntity.PhysicsBody.GravParam;

			if (mass <= 0.0) return false;

			if (double.IsNaN(_manager.TargetPosition.x) || double.IsNaN(_manager.TargetPosition.y)) return false;

			double2 R = _manager.TargetPosition - currentPos;
			double2 V = _manager.TargetVelocity - currentVel;
			double distance = math.length(R);

			if (distance <= 1.0 || double.IsNaN(distance)) return false;

			float acceleration = (float)(_manager.MaxForce / mass);
			if (acceleration <= 0f || float.IsNaN(acceleration)) return false;

			PhysicsBody parentSoI = _manager.OwnerEntity.DominantEntity?.PhysicsBody ?? null;

			// Invoke the static helper to get the time of flight
			float t = ComputeInterceptTimeOfFlight(currentPos, currentVel, _manager.TargetPosition, _manager.TargetVelocity, mass, _manager.MaxForce, mu, parentSoI, _lastFlightTime);

			if (t > 0f && !float.IsNaN(t) && !float.IsInfinity(t))
			{
				_lastFlightTime = t;
				timeOfFlight = t;
			}
			else
			{
				timeOfFlight = _lastFlightTime > 0f ? _lastFlightTime : (float)math.sqrt(2.0 * distance / acceleration);
				t = timeOfFlight;
			}

			double2 gravityAcc = double2.zero;
			if (parentSoI != null)
			{
				double2 rVec = currentPos - parentSoI.Position;
				double rDist = math.length(rVec);
				if (rDist > 1.0)
				{
					gravityAcc = -mu * rVec / (rDist * rDist * rDist);
				}
			}
			else
			{
				double rDist = math.length(currentPos);
				if (rDist > 1.0)
				{
					gravityAcc = -mu * currentPos / (rDist * rDist * rDist);
				}
			}

			double2 R_corrected = R - (0.5 * gravityAcc * (t * t));
			double2 requiredVelChange = R_corrected + (V * t);

			double len = math.length(requiredVelChange);
			if (len > 1e-5 && !double.IsNaN(len) && !double.IsInfinity(len))
			{
				double2 burnDir = requiredVelChange / len;
				if (!double.IsNaN(burnDir.x) && !double.IsNaN(burnDir.y))
				{
					burnOrientation = math.atan2((float)burnDir.y, (float)burnDir.x);
					return true;
				}
			}

			if (distance > 1e-5)
			{
				double2 burnDir = R / distance;
				burnOrientation = math.atan2((float)burnDir.y, (float)burnDir.x);
				return true;
			}

			return false;
		}

		/// <summary>
		/// Numerically propagates the powered intercept trajectory over time to generate a renderable path.
		/// </summary>
		public static List<double2> PredictInterceptPath(double2 currentPos, double2 currentVel, double2 targetPos, double2 targetVel, double mass, float maxForce, double mu, double timeOfFlight, PhysicsBody parent, out double2 finalVel, int steps = 150, float arrivalThreshold = 10f)
		{
			List<double2> path = new List<double2>();
			finalVel = currentVel;
			if (mass <= 0.0) return path;

			double2 pos = currentPos;
			float acceleration = maxForce / (float)mass;

			double dt = timeOfFlight / steps;
			path.Add(pos);

			for (int i = 0; i < steps; i++)
			{
				double2 R = targetPos - pos;
				double distance = math.length(R);

				if (distance <= arrivalThreshold)
				{
					break;
				}

				// Re-evaluate intercept guidance dynamically at each step (matching ComputeInterceptTrajectory)
				double2 V = targetVel - finalVel;
				double2 gravityAcc = double2.zero;
				if (parent != null)
				{
					double2 rVec = pos - parent.Position;
					double rDist = math.length(rVec);
					if (rDist > 1.0)
					{
						gravityAcc = -mu * rVec / (rDist * rDist * rDist);
					}
				}

				double tRem = math.max(0.1, timeOfFlight - (i * dt));
				double2 R_corrected = R - (0.5 * gravityAcc * (tRem * tRem));
				double2 requiredVelChange = R_corrected + (V * tRem);

				double len = math.length(requiredVelChange);
				float burnAngle = 0f;
				if (len > 1e-5 && !double.IsNaN(len) && !double.IsInfinity(len))
				{
					double2 burnDir = requiredVelChange / len;
					burnAngle = math.atan2((float)burnDir.y, (float)burnDir.x);
				}
				else if (distance > 1e-5)
				{
					burnAngle = math.atan2((float)R.y, (float)R.x);
				}

				// 1. Compute thrust acceleration vector
				double2 thrustAcc = new double2(math.cos(burnAngle), math.sin(burnAngle)) * acceleration;

				double2 totalAcc = thrustAcc + gravityAcc;

				// 2. Integration step for vehicle
				finalVel += totalAcc * dt;
				pos += finalVel * dt;

				// Propagate target position and velocity forward
				double2 targetGravityAcc = double2.zero;
				if (parent != null)
				{
					double2 rVecTarget = targetPos - parent.Position;
					double rDistTarget = math.length(rVecTarget);
					if (rDistTarget > 1.0)
					{
						targetGravityAcc = -mu * rVecTarget / (rDistTarget * rDistTarget * rDistTarget);
					}
				}
				targetVel += targetGravityAcc * dt;
				targetPos += targetVel * dt;

				path.Add(pos);
			}

			return path;
		}
	}
}