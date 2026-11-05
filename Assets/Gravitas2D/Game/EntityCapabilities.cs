using System.Collections.Generic;
using Gravitas2D;
using Unity.Mathematics;
using UnityEngine;

namespace GravitasDemo
{
	public interface IEntityCapability { }

	public enum SensorType { Optical = 0, Infrared = 1, Radar = 2 }

	public class SensorCapability : IEntityCapability
	{
		public SensorType Type { get; set; }
		public double Range { get; set; }
		public float FieldOfViewDegrees { get; set; } = 360f; // Default omnidirectional
		public float OrientationOffset { get; set; } = 0f;    // Offset relative to entity orientation
		public float Sensitivity { get; set; } = 1.0f;        // Tuning multiplier for detection strength
	}

	public class LaunchCapability : IEntityCapability
	{
		public int ProjectileCompositionIndex { get; set; }
		public float ProjectileRadius { get; set; }
		public float FireSpeed { get; set; }
		public float ReloadTime { get; set; }
		public int Ammo { get; set; }
	}

	public class MissileCapability : IEntityCapability
	{
		public int ProjectileCompositionIndex { get; set; }
		public float ProjectileRadius { get; set; }
		public float FireSpeed { get; set; }
		public float ReloadTime { get; set; }
		public int Ammo { get; set; }
	}

	public class WarheadCapability : IEntityCapability
	{
		public double WarheadEnergy { get; set; } // Joules

		public static WarheadCapability Missile => new WarheadCapability
		{
			WarheadEnergy = MissileEnergy
		};

		public static double MissileEnergy => 1.25e12;
	}

	public class NavigationCapability : IEntityCapability
	{
		public Entity OwnerEntity { get; set; }
		public Entity TargetEntity { get; set; }
		public double2 TargetPosition { get; set; }
		public double2 TargetVelocity { get; set; }
		public float MaxForce { get; set; } = 10000f; // Newtons
		public bool IsNavigating { get; set; } = false;
		public float ArrivalThreshold { get; set; } = 10.0f; // Meters
		public float RendezvousVelocityThreshold { get; set; } = 0.25f; // m/s relative speed required for capture
		public bool RendezvousSolutionFailed { get; private set; }
		public bool IsRendezvousMode { get; set; } = false; // Toggle between Intercept and Rendezvous
		public bool IsStationKeepingMode { get; set; } = false; // Toggle for relative position hold / station-keeping
		public bool IsOrbitMatchMode { get; set; } = false; // Toggle for matching a reference orbit

		// Attitude / Maneuvering Control Properties
		public float TargetOrientation { get; set; } = 0f; // Radians
		public float MaxTorque { get; set; } = 1000f; // Newton-meters
		public bool IsEnabled { get; set; } = true;

		// Predictive attitude-tracking state. Derivatives are filtered because the commanded
		// burn vector can move discontinuously between guidance updates.
		private float _targetAngularVelocity;
		private float _targetAngularAcceleration;
		private bool _hasTargetOrientationSample;

		// Nominal attitude-loop tuning. The controller adapts these values to the actual
		// scaled physics step and current vehicle inertia / torque authority at runtime.
		public float AttitudePredictionLeadTime { get; set; } = 0.04f;
		public float AttitudeMaxPredictionHorizon { get; set; } = 0.15f;
		public float AttitudeTargetRateFilterTime { get; set; } = 0.025f;
		public float AttitudeTargetAccelerationFilterTime { get; set; } = 0.05f;
		public float AttitudeTargetRateLimit { get; set; } = 100f;
		public float AttitudeTargetAccelerationLimit { get; set; } = 5000f;
		public float AttitudeCaptureAngle { get; set; } = 0.025f;
		public float AttitudeSwitchingAngle { get; set; } = 0.001f;

		// Base burn tolerances. The effective angular tolerance is allowed a very small
		// increase at coarse simulation steps, capped at 0.005 rad (0.286 degrees).
		public float BurnAlignmentToleranceRadians { get; set; } = 0.002f;
		public float BurnAlignmentRateTolerance { get; set; } = 0.02f;
		public bool RequireAttitudeAlignmentForThrust { get; set; } = true;

		// Recomputed from the live physics model and simulation timestep. These are kept
		// separate from the public nominal settings so time-scale changes do not compound.
		private float _controlStep = 0.02f;
		private float _effectivePredictionLeadTime = 0.04f;
		private float _effectivePredictionHorizon = 0.15f;
		private float _effectiveRateFilterTime = 0.025f;
		private float _effectiveAccelerationFilterTime = 0.05f;
		private float _effectiveTargetRateLimit = 100f;
		private float _effectiveTargetAccelerationLimit = 5000f;
		private float _effectiveCaptureAngle = 0.025f;
		private float _effectiveSwitchingAngle = 0.001f;
		private float _effectiveBurnAlignmentTolerance = 0.002f;
		private float _effectiveBurnRateTolerance = 0.02f;
		private float _effectiveMaxAngularAcceleration;
		private float _effectiveNaturalFrequency;

		// Read-only diagnostics for inspecting the automatic tuning at runtime.
		public float EffectiveAttitudeTimeStep => _controlStep;
		public float EffectiveMaxAngularAcceleration => _effectiveMaxAngularAcceleration;
		public float EffectiveAttitudeNaturalFrequency => _effectiveNaturalFrequency;
		public float EffectiveBurnAlignmentToleranceRadians => _effectiveBurnAlignmentTolerance;
		public float EffectiveBurnAlignmentRateTolerance => _effectiveBurnRateTolerance;

		// Orbit Match Reference Parameters
		public Entity TargetOrbitEntity { get; set; }
		public double2 TargetOrbitPositionOffset { get; set; }
		public double2 TargetOrbitVelocity { get; set; }

		public float BurnOrientation { get; private set; }
		/// <summary>Requested thrust fraction for rendezvous guidance, in [0, 1].</summary>
		public float BurnThrottle { get; private set; } = 1f;

		// Desired velocity correction for the active orbit-match guidance update.
		// Translational thrust is scaled from this error so it tapers smoothly at capture.
		public double RequiredOrbitMatchDeltaV { get; private set; }

		// Approximate velocity-error response time. Smaller values are more aggressive;
		// thrust still saturates at MaxForce for large corrections.
		public float OrbitMatchResponseTime { get; set; } = 1.0f;

		// Low-pass the commanded thrust direction to suppress frame-to-frame sign flips.
		public float OrbitMatchDirectionResponseTime { get; set; } = 0.25f;
		private double2 _smoothedOrbitMatchBurnDirection;
		private bool _hasSmoothedOrbitMatchBurnDirection;

		// Rendezvous Clearance
		public float ClearanceDistance { get; set; } = 1000;

		private float _lastFlightTime = -1f;		
		private readonly RendezvousGuidanceSolver.PlanMemory _rendezvousPlanMemory = new RendezvousGuidanceSolver.PlanMemory();
		private Entity _rendezvousOffsetTarget;
		private double2 _rendezvousOffsetDirection;
		private bool _hasRendezvousOffsetDirection;

		public float TimeOfFlight { get; private set; }
		public double LastRendezvousSolveMilliseconds => _rendezvousPlanMemory.LastSolveMilliseconds;
		public int LastRendezvousTerminalEvaluations => _rendezvousPlanMemory.LastTerminalEvaluationCount;
		public int LastRendezvousIntegrationSteps => _rendezvousPlanMemory.LastVehicleIntegrationStepCount;
		public int LastRendezvousGravitySegments => _rendezvousPlanMemory.LastGravityEphemerisSegmentCount;

		public static NavigationCapability Missile => new NavigationCapability
		{
			MaxForce = MaxMissileForce, // 50g acceleration
			MaxTorque = 10000000
		};

		public static float MaxMissileForce => (float)2.5797e8; // 50g acceleration

		public void EndNavigation()
		{
			_lastFlightTime = -1f;
			TimeOfFlight = -1f;
			TargetEntity = null;
			TargetPosition = double2.zero;
			TargetVelocity = double2.zero;
			IsNavigating = false;
			IsRendezvousMode = false;
			IsStationKeepingMode = false;
			IsOrbitMatchMode = false;
			IsEnabled = false;
			TargetOrientation = OwnerEntity.Orientation;
			BurnOrientation = 0;
			BurnThrottle = 0f;
			RendezvousSolutionFailed = false;
			_targetAngularVelocity = 0f;
			_targetAngularAcceleration = 0f;
			_hasTargetOrientationSample = false;
			TargetOrbitEntity = null;
			TargetOrbitPositionOffset = double2.zero;
			TargetOrbitVelocity = double2.zero;
			RequiredOrbitMatchDeltaV = 0;
			_rendezvousOffsetTarget = null;
			_rendezvousOffsetDirection = double2.zero;
			_hasRendezvousOffsetDirection = false;
			_rendezvousPlanMemory.Reset();
		}

		/// <summary>
		/// Resolves the actual simulation-time step (including the time multiplier) and
		/// retunes the attitude loop from the current body inertia and configured torque.
		/// Call once at the start of each navigation evaluation.
		/// </summary>
		public float PrepareAttitudeControl(float fallbackDeltaTime)
		{
			float dt = fallbackDeltaTime;
			PhysicsManager2D manager = PhysicsManager2D.Instance;
			if (manager != null && manager.IsInitialized)
			{
				float scaledStep = manager.ScaledPhysicsTimeStep;
				if (IsFinite(scaledStep) && scaledStep > 0.00001f)
					dt = scaledStep;
			}

			if (!IsFinite(dt) || dt <= 0.00001f)
				dt = Mathf.Max(0.0001f, Time.fixedDeltaTime);

			_controlStep = dt;
			TuneAttitudeParameters(dt);
			return dt;
		}

		private void TuneAttitudeParameters(float dt)
		{
			dt = math.max(0.0001f, dt);

			double inertia = OwnerEntity != null && OwnerEntity.PhysicsBody != null
				? OwnerEntity.PhysicsBody.Inertia
				: 1.0;
			if (inertia <= 0.0 || double.IsNaN(inertia) || double.IsInfinity(inertia))
				inertia = 1.0;

			float torqueLimit = math.abs(MaxTorque);
			_effectiveMaxAngularAcceleration = torqueLimit > 0f
				? (float)(torqueLimit / inertia)
				: 0f;
			if (!IsFinite(_effectiveMaxAngularAcceleration))
				_effectiveMaxAngularAcceleration = 0f;

			// A prediction must cover at least one actual simulation step. At 10x speed,
			// for example, a 0.02 s fixed step may become a 0.2 s control interval.
			_effectivePredictionLeadTime = math.max(math.max(0f, AttitudePredictionLeadTime), dt);
			_effectivePredictionHorizon = math.max(
				math.max(0f, AttitudeMaxPredictionHorizon),
				_effectivePredictionLeadTime);

			// Derivative filters are never allowed to be shorter than the sampling interval.
			// This prevents noisy command-rate estimates when the time multiplier is increased.
			_effectiveRateFilterTime = math.max(math.max(0.001f, math.abs(AttitudeTargetRateFilterTime)), dt);
			_effectiveAccelerationFilterTime = math.max(
				math.max(0.001f, math.abs(AttitudeTargetAccelerationFilterTime)),
				1.5f * dt);

			// Limit the reference rate to a rate that the available angular acceleration can
			// reasonably brake over a half-turn, while retaining the user's setting as a cap.
			float authorityRate = _effectiveMaxAngularAcceleration > 0f
				? math.sqrt(2f * _effectiveMaxAngularAcceleration * Mathf.PI)
				: 0.1f;
			_effectiveTargetRateLimit = math.max(0.1f, math.min(
				math.max(0.1f, math.abs(AttitudeTargetRateLimit)),
				authorityRate));

			float authorityAccelerationLimit = math.max(1f, 4f * _effectiveMaxAngularAcceleration);
			_effectiveTargetAccelerationLimit = math.max(1f, math.min(
				math.max(1f, math.abs(AttitudeTargetAccelerationLimit)),
				authorityAccelerationLimit));

			_effectiveCaptureAngle = math.max(0.0001f, math.abs(AttitudeCaptureAngle));
			_effectiveSwitchingAngle = math.max(0.0001f, math.abs(AttitudeSwitchingAngle));

			// Discrete-time gain scheduling: the ideal continuous-time natural frequency is
			// based on available angular acceleration and capture angle, then limited so the
			// controller cannot demand an excessively fast response for this sampling interval.
			float authorityFrequency = _effectiveMaxAngularAcceleration > 0f
				? math.sqrt(2f * _effectiveMaxAngularAcceleration / _effectiveCaptureAngle)
				: 0f;
			float sampleLimitedFrequency = 0.65f / dt;
			_effectiveNaturalFrequency = math.min(authorityFrequency, sampleLimitedFrequency);
			if (!IsFinite(_effectiveNaturalFrequency))
				_effectiveNaturalFrequency = 0f;

			// Keep the requested precision at normal step sizes. Relax it only slightly when
			// the acceleration and the discrete step make sub-step angular resolution coarse.
			float stepAngularResolution = _effectiveMaxAngularAcceleration * dt * dt;
			float discretizationAllowance = math.min(0.005f, 0.02f * stepAngularResolution);
			_effectiveBurnAlignmentTolerance = math.max(
				math.max(0.00001f, math.abs(BurnAlignmentToleranceRadians)),
				discretizationAllowance);
			// The acceptable relative rate is linked to the vehicle's available braking
			// authority: at this rate, the ideal stopping distance is roughly the angular
			// alignment tolerance. Cap the model-derived allowance to avoid permitting a
			// visibly spinning vehicle to burn; the configured value remains a floor.
			float rateFromStoppingAngle = math.sqrt(
				2f * _effectiveMaxAngularAcceleration * _effectiveBurnAlignmentTolerance);
			float modelRateAllowance = math.min(0.25f, rateFromStoppingAngle);
			_effectiveBurnRateTolerance = math.max(
				math.max(0.00001f, math.abs(BurnAlignmentRateTolerance)),
				modelRateAllowance);
		}

		private static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);

		/// <summary>
		/// Updates target-angle kinematics. The command angle is not filtered; its rate and
		/// acceleration estimates are filtered using time constants adapted to the simulation step.
		/// </summary>
		public void UpdateTargetOrientation(float newTargetOrientation, float dt)
		{
			if (!IsFinite(newTargetOrientation)) return;
			dt = PrepareAttitudeControl(dt);

			if (dt <= 0.0001f)
			{
				TargetOrientation = newTargetOrientation;
				_targetAngularVelocity = 0f;
				_targetAngularAcceleration = 0f;
				_hasTargetOrientationSample = false;
				return;
			}

			float deltaAngle = WrapAngle(newTargetOrientation - TargetOrientation);
			TargetOrientation = newTargetOrientation;

			// The first sample establishes a reference. Do not interpret initialization as
			// instantaneous target motion, which would generate a false feedforward spike.
			if (!_hasTargetOrientationSample)
			{
				_targetAngularVelocity = 0f;
				_targetAngularAcceleration = 0f;
				_hasTargetOrientationSample = true;
				return;
			}

			float measuredOmega = math.clamp(deltaAngle / dt, -_effectiveTargetRateLimit, _effectiveTargetRateLimit);
			float previousOmega = _targetAngularVelocity;
			float rateBlend = 1f - math.exp(-dt / _effectiveRateFilterTime);
			_targetAngularVelocity = math.lerp(previousOmega, measuredOmega, rateBlend);

			float measuredAlpha = (_targetAngularVelocity - previousOmega) / dt;
			measuredAlpha = math.clamp(
				measuredAlpha,
				-_effectiveTargetAccelerationLimit,
				_effectiveTargetAccelerationLimit);
			float accelerationBlend = 1f - math.exp(-dt / _effectiveAccelerationFilterTime);
			_targetAngularAcceleration = math.lerp(_targetAngularAcceleration, measuredAlpha, accelerationBlend);
		}
		
		/// <summary>
		/// Predictive, rate-aware attitude control. Outside the capture region it uses a
		/// phase-plane braking law; close to the setpoint it transitions to damped PD
		/// control. Target acceleration is feedforward-compensated with torque headroom
		/// reserved for feedback.
		/// </summary>
		public float EvaluateAnalogTorque(float deltaTime)
		{
			if (!IsEnabled || OwnerEntity == null || OwnerEntity.PhysicsBody == null) return 0f;
			deltaTime = PrepareAttitudeControl(deltaTime);

			double inertia = OwnerEntity.PhysicsBody.Inertia;
			if (inertia <= 0.0 || double.IsNaN(inertia) || double.IsInfinity(inertia)) inertia = 1.0;
			float torqueLimit = math.abs(MaxTorque);
			if (torqueLimit <= 0f) return 0f;

			float alphaMax = (float)(torqueLimit / inertia);
			if (alphaMax <= 0f || !IsFinite(alphaMax)) return 0f;

			float predictionTime = math.min(
				math.max(_controlStep, _effectivePredictionLeadTime),
				_effectivePredictionHorizon);
			float bodyOmega = OwnerEntity.AngularVelocity;

			// Reserve at least half of the available angular acceleration for feedback/braking.
			float targetAlpha = math.clamp(_targetAngularAcceleration, -0.5f * alphaMax, 0.5f * alphaMax);
			float feedbackAlphaMax = math.max(0.5f * alphaMax, alphaMax - math.abs(targetAlpha));

			float predictedTargetAngle = TargetOrientation
				+ (_targetAngularVelocity * predictionTime)
				+ (0.5f * targetAlpha * predictionTime * predictionTime);
			float predictedBodyAngle = OwnerEntity.Orientation
				+ (bodyOmega * predictionTime)
				+ (0.5f * targetAlpha * predictionTime * predictionTime);
			float error = WrapAngle(predictedTargetAngle - predictedBodyAngle);
			float relativeOmega = bodyOmega - _targetAngularVelocity;

			// Phase-plane braking uses signed stopping angle, so braking begins before the
			// commanded orientation is crossed, including when the target itself is moving.
			// The switching layer grows with alpha * dt^2 (not dt alone) so its discrete-time
			// correction remains proportional to a fraction of the error at coarse time scales.
			float brakeAngle = (relativeOmega * math.abs(relativeOmega)) / (2f * feedbackAlphaMax);
			float phaseError = error - brakeAngle;
			float switchingWidth = math.max(
				_effectiveSwitchingAngle,
				feedbackAlphaMax * _controlStep * _controlStep);
			float phaseAlpha = feedbackAlphaMax * math.clamp(phaseError / switchingWidth, -1f, 1f);

			// Model- and step-aware terminal PD gains. The natural frequency is capped in
			// TuneAttitudeParameters so large time multipliers do not create discrete-time
			// overshoot that prevents the burn-alignment gate from ever being satisfied.
			float naturalFrequency = _effectiveNaturalFrequency;
			float kp = naturalFrequency * naturalFrequency;
			float kd = 2f * naturalFrequency; // critical damping
			float pdAlpha = math.clamp(kp * error - kd * relativeOmega, -feedbackAlphaMax, feedbackAlphaMax);

			float blend = math.clamp((math.abs(error) - _effectiveCaptureAngle) / _effectiveCaptureAngle, 0f, 1f);
			blend = blend * blend * (3f - 2f * blend);
			float feedbackAlpha = math.lerp(pdAlpha, phaseAlpha, blend);
			float commandedAlpha = targetAlpha + feedbackAlpha;
			return math.clamp((float)(inertia * commandedAlpha), -torqueLimit, torqueLimit);
		}

		/// <summary>Returns true when the force direction for the current physics step is aligned.</summary>
		public bool IsBurnAligned(float deltaTime)
		{
			if (OwnerEntity == null || OwnerEntity.PhysicsBody == null) return false;
			PrepareAttitudeControl(deltaTime);

			float error = WrapAngle(TargetOrientation - OwnerEntity.Orientation);
			float relativeOmega = OwnerEntity.AngularVelocity - _targetAngularVelocity;

			return math.abs(error) <= _effectiveBurnAlignmentTolerance
				&& math.abs(relativeOmega) <= _effectiveBurnRateTolerance;
		}

		private static float WrapAngle(float angle)
		{
			while (angle > Mathf.PI) angle -= 2f * Mathf.PI;
			while (angle < -Mathf.PI) angle += 2f * Mathf.PI;
			return angle;
		}

		public void SetTargetOrientation(float targetOrientation)
		{
			TargetOrientation = targetOrientation;
			_targetAngularVelocity = 0f;
			_targetAngularAcceleration = 0f;
			_hasTargetOrientationSample = false;
			IsNavigating = false; // Disable active transit so it acts purely as an attitude hold
			IsRendezvousMode = false;
			IsStationKeepingMode = false;
			IsOrbitMatchMode = false;
			IsEnabled = true;
		}

		/// <summary>
		/// Updates target position and velocity.
		/// </summary>
		private void UpdateTargetState()
		{
			if (TargetEntity == null || TargetEntity.IsDestroyed) return;

			if (IsRendezvousMode)
			{
				double targetRadius = TargetEntity.Radius;
				double targetClearance = TargetEntity.IsCelestial ? targetRadius * 0.1 : ClearanceDistance;

				// Choose the standoff side once per rendezvous. Recomputing the direction from
				// instantaneous target velocity every frame makes the aim point orbit around the
				// target, while its velocity was still reported as the target centre velocity.
				// A fixed inertial offset gives the solver a consistent moving terminal point.
				if (!_hasRendezvousOffsetDirection || _rendezvousOffsetTarget != TargetEntity)
				{
					double2 approachDirection;
					if (OwnerEntity.InGroup && OwnerEntity.Group.Leader != null)
						approachDirection = OwnerEntity.Group.Leader.Position - TargetEntity.Position;
					else if (math.lengthsq(TargetEntity.Velocity) > 1e-12)
						approachDirection = -TargetEntity.Velocity;
					else
						approachDirection = OwnerEntity.Position - TargetEntity.Position;

					_rendezvousOffsetDirection = math.lengthsq(approachDirection) > 1e-12
						? math.normalize(approachDirection)
						: new double2(1.0, 0.0);
					_rendezvousOffsetTarget = TargetEntity;
					_hasRendezvousOffsetDirection = true;
				}

				double approachBodyRadius = OwnerEntity.InGroup && OwnerEntity.Group.Leader != null
					? OwnerEntity.Group.Leader.Radius
					: OwnerEntity.Radius;
				double separation = targetRadius + approachBodyRadius + targetClearance;
				TargetPosition = TargetEntity.Position + separation * _rendezvousOffsetDirection;

				if (OwnerEntity.InGroup)
					TargetPosition += OwnerEntity.Group.GetFormationPosition(OwnerEntity);
			}
			else
			{
				TargetPosition = TargetEntity.Position;
				_rendezvousOffsetTarget = null;
				_hasRendezvousOffsetDirection = false;
			}

			TargetVelocity = TargetEntity.Velocity;
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
		public bool ComputeInterceptTrajectory()
		{
			BurnOrientation = 0f;
			TimeOfFlight = _lastFlightTime > 0f ? _lastFlightTime : 1.0f;

			double mass = OwnerEntity.Mass;
			double2 currentPos = OwnerEntity.Position;
			double2 currentVel = OwnerEntity.Velocity;
			double mu = OwnerEntity.PhysicsBody.GravParam;

			if (mass <= 0.0) return false;

			UpdateTargetState();

			if (double.IsNaN(TargetPosition.x) || double.IsNaN(TargetPosition.y)) return false;

			double2 R = TargetPosition - currentPos;
			double2 V = TargetVelocity - currentVel;
			double distance = math.length(R);

			if (distance <= 1.0 || double.IsNaN(distance)) return false;

			float acceleration = (float)(MaxForce / mass);
			if (acceleration <= 0f || float.IsNaN(acceleration)) return false;

			PhysicsBody parentSoI = OwnerEntity.DominantEntity?.PhysicsBody ?? null;

			// Invoke the static helper to get the time of flight
			float t = ComputeInterceptTimeOfFlight(currentPos, currentVel, TargetPosition, TargetVelocity, mass, MaxForce, mu, parentSoI, _lastFlightTime);

			if (t > 0f && !float.IsNaN(t) && !float.IsInfinity(t))
			{
				_lastFlightTime = t;
				TimeOfFlight = t;
			}
			else
			{
				TimeOfFlight = _lastFlightTime > 0f ? _lastFlightTime : (float)math.sqrt(2.0 * distance / acceleration);
				t = TimeOfFlight;
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
					BurnOrientation = math.atan2((float)burnDir.y, (float)burnDir.x);
					return true;
				}
			}

			if (distance > 1e-5)
			{
				double2 burnDir = R / distance;
				BurnOrientation = math.atan2((float)burnDir.y, (float)burnDir.x);
				return true;
			}

			return false;
		}

		/// <summary>
		/// Statically computes the time of flight / time-to-go for a rendezvous trajectory 
		/// using a gravity-augmented constant-acceleration profile.
		/// </summary>
		public static float ComputeRendezvousTimeOfFlight(double2 currentPos, double2 currentVel, double2 targetPos, double2 targetVel, double mass, float maxForce, double mu, PhysicsBody parentSoI, float arrivalThreshold = 10.0f)
		{
			double2 R = targetPos - currentPos;
			double2 V = targetVel - currentVel;
			double distance = math.length(R);
			double relSpeed = math.length(V);

			if (distance <= arrivalThreshold && relSpeed < 0.5) return 0f;

			float acceleration = (float)(maxForce / mass);
			if (acceleration <= 0f || float.IsNaN(acceleration)) return 1f;

			// Constant-acceleration rendezvous time of flight (bridges distance R and matches relative velocity V under max acceleration a)
			float t = (float)((relSpeed + math.sqrt(relSpeed * relSpeed + 4.0 * acceleration * distance)) / (2.0 * acceleration));

			if (float.IsNaN(t) || float.IsInfinity(t) || t <= 0.05f) t = 0.05f;
			return t;
		}

		public bool ComputeRendezvousTrajectory(float controlDeltaTime = 0.02f)
		{
			BurnThrottle = 0f;
			RendezvousSolutionFailed = false;
			if (OwnerEntity == null || OwnerEntity.IsDestroyed || OwnerEntity.PhysicsBody == null || OwnerEntity.Mass <= 0.0)
			{
				RendezvousSolutionFailed = true;
				return false;
			}

			UpdateTargetState();
			if (!IsFinite(TargetPosition.x) || !IsFinite(TargetPosition.y) ||
				!IsFinite(TargetVelocity.x) || !IsFinite(TargetVelocity.y))
			{
				RendezvousSolutionFailed = true;
				return false;
			}

			double2 relativePosition = TargetPosition - OwnerEntity.Position;
			double2 relativeVelocity = TargetVelocity - OwnerEntity.Velocity;
			double distance = math.length(relativePosition);
			double relativeSpeed = math.length(relativeVelocity);

			// Capture is a terminal-state condition, not merely a distance crossing. This prevents
			// the controller from declaring success while carrying substantial closing velocity.
			if (distance <= ArrivalThreshold && relativeSpeed <= RendezvousVelocityThreshold)
			{
				TimeOfFlight = 0f;
				return false;
			}

			if (MaxForce <= 0f || OwnerEntity.Mass <= 0.0)
			{
				RendezvousSolutionFailed = true;
				return false;
			}

			if (!RendezvousGuidanceSolver.TrySolve(
				OwnerEntity,
				TargetEntity,
				TargetPosition,
				TargetVelocity,
				MaxForce,
				ArrivalThreshold,
				RendezvousVelocityThreshold,
				controlDeltaTime,
				TimeOfFlight,
				out double2 commandedAcceleration,
				out float solvedTimeOfFlight,
				planMemory: _rendezvousPlanMemory))
			{
				RendezvousSolutionFailed = true;
				return false;
			}

			TimeOfFlight = solvedTimeOfFlight;
			double requestedAcceleration = math.length(commandedAcceleration);
			double maximumAcceleration = MaxForce / OwnerEntity.Mass;
			if (!IsFinite(requestedAcceleration) || requestedAcceleration <= 1e-10 || maximumAcceleration <= 0.0)
			{
				// A zero-thrust first interval is a valid MPC decision (coast before a later burn).
				// Keep the current attitude; the next navigation tick replans from the new state.
				BurnOrientation = OwnerEntity.Orientation;
				BurnThrottle = 0f;
				return true;
			}

			BurnOrientation = math.atan2((float)commandedAcceleration.y, (float)commandedAcceleration.x);
			BurnThrottle = math.clamp((float)(requestedAcceleration / maximumAcceleration), 0f, 1f);
			return true;
		}

		private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

		/// <summary>
		/// Computes the trajectory adjustment for relative position hold (station-keeping) relative to the target entity or position.
		/// </summary>
		public bool ComputeStationKeepingTrajectory()
		{
			double mass = OwnerEntity.Mass;
			double2 currentPos = OwnerEntity.Position;
			double2 currentVel = OwnerEntity.Velocity;

			if (mass <= 0.0) return false;

			double2 targetPos = TargetEntity != null && !TargetEntity.IsDestroyed ? TargetEntity.Position : TargetPosition;
			double2 targetVel = TargetEntity != null && !TargetEntity.IsDestroyed ? TargetEntity.Velocity : TargetVelocity;

			if (double.IsNaN(targetPos.x) || double.IsNaN(targetPos.y)) return false;

			double2 R = targetPos - currentPos; // Relative position error
			double2 V = currentVel - targetVel; // Relative velocity error

			// PD controller for maintaining relative position and nulling relative velocity
			double2 requiredVelChange = R - (V * 2.0);

			double len = math.length(requiredVelChange);
			if (len > 1e-5 && !double.IsNaN(len) && !double.IsInfinity(len))
			{
				double2 burnDir = requiredVelChange / len;
				if (!double.IsNaN(burnDir.x) && !double.IsNaN(burnDir.y))
				{
					BurnOrientation = math.atan2((float)burnDir.y, (float)burnDir.x);
					return true;
				}
			}

			return false;
		}
	
		/// <summary>
		/// Computes the trajectory to match a defined target orbit's energy and angular momentum invariants, 
		/// ensuring exact geometric orientation without rotational offset.
		/// </summary>
		public bool ComputeOrbitMatchTrajectory()
		{
			double mass = OwnerEntity.Mass;
			double2 currentPos = OwnerEntity.Position;
			double2 currentVel = OwnerEntity.Velocity;

			RequiredOrbitMatchDeltaV = 0.0;
			if (mass <= 0.0) return false;

			PhysicsBody parentSoI = OwnerEntity.DominantEntity?.PhysicsBody ?? null;
			if (parentSoI == null) return false;

			// Dynamically compute absolute target position from relative offset if manually placed
			double2 tPos = TargetOrbitEntity != null && !TargetOrbitEntity.IsDestroyed 
				? TargetOrbitEntity.Position 
				: parentSoI.Position + TargetOrbitPositionOffset;
			
			double2 tVel = TargetOrbitEntity != null && !TargetOrbitEntity.IsDestroyed 
				? TargetOrbitEntity.Velocity 
				: TargetOrbitVelocity;

			if (double.IsNaN(tPos.x) || double.IsNaN(tPos.y) || double.IsInfinity(tPos.x) || double.IsInfinity(tPos.y) ||
				double.IsNaN(tVel.x) || double.IsNaN(tVel.y) || double.IsInfinity(tVel.x) || double.IsInfinity(tVel.y))
				return false;

			double mu = OwnerEntity.PhysicsBody.GravParam;
			if (mu <= 0.0 || double.IsNaN(mu) || double.IsInfinity(mu)) return false;

			double2 rTargetRel = tPos - parentSoI.Position;
			double2 vTargetRel = tVel - parentSoI.Velocity;
			double rTargetDist = math.length(rTargetRel);
			if (rTargetDist <= 1.0) return false;

			double targetEnergy = 0.5 * math.lengthsq(vTargetRel) - (mu / rTargetDist);
			double targetH = (rTargetRel.x * vTargetRel.y) - (rTargetRel.y * vTargetRel.x);
			if (math.abs(targetEnergy) < 1e-12 || math.abs(targetH) < 1e-9) return false;

			double targetSemiMajorAxis = -mu / (2.0 * targetEnergy);
			if (double.IsNaN(targetSemiMajorAxis) || double.IsInfinity(targetSemiMajorAxis)) return false;

			double2 targetEccVec = new double2(
				(vTargetRel.y * targetH / mu) - (rTargetRel.x / rTargetDist),
				(-vTargetRel.x * targetH / mu) - (rTargetRel.y / rTargetDist));

			double2 rCurrRel = currentPos - parentSoI.Position;
			double2 vCurrRel = currentVel - parentSoI.Velocity;
			double rCurrDist = math.length(rCurrRel);
			if (rCurrDist <= 1.0) return false;

			double2 uR = rCurrRel / rCurrDist;
			double2 uTheta = new double2(-uR.y, uR.x);

			double targetP = (targetH * targetH) / mu;
			double radiusDenominator = 1.0 + math.dot(targetEccVec, uR);
			if (radiusDenominator <= 1e-6) return false;
			double targetRadiusAtCurrentAngle = targetP / radiusDenominator;

			double targetRadialVelocity = -(mu / targetH) * math.dot(targetEccVec, uTheta);

			double maxAccel = MaxForce / mass;
			if (maxAccel <= 0.0 || double.IsNaN(maxAccel) || double.IsInfinity(maxAccel)) return false;
			double radiusError = targetRadiusAtCurrentAngle - rCurrDist;
			double radialCorrectionSpeed = math.sqrt(maxAccel * math.abs(radiusError));
			double localCircularSpeed = math.sqrt(mu / rCurrDist);
			radialCorrectionSpeed = math.min(radialCorrectionSpeed, localCircularSpeed * 0.1);
			double radialCorrectionSign = radiusError > 0.0 ? 1.0 : radiusError < 0.0 ? -1.0 : 0.0;
			double vrDesired = targetRadialVelocity + radialCorrectionSign * radialCorrectionSpeed;
			double vThetaDesired = targetH / rCurrDist;

			double2 vDesiredRel = (uR * vrDesired) + (uTheta * vThetaDesired);
			double2 vDesired = vDesiredRel + parentSoI.Velocity;
			double2 requiredVelChange = vDesired - currentVel;

			double len = math.length(requiredVelChange);
			if (len > 1e-8 && !double.IsNaN(len) && !double.IsInfinity(len))
			{
				RequiredOrbitMatchDeltaV = len;
				double2 burnDir = requiredVelChange / len;
				if (!double.IsNaN(burnDir.x) && !double.IsNaN(burnDir.y) && !double.IsInfinity(burnDir.x) && !double.IsInfinity(burnDir.y))
				{
					BurnOrientation = math.atan2((float)burnDir.y, (float)burnDir.x);
					return true;
				}
			}

			return false;
		}

		public void ResetOrbitMatchGuidance()
		{
			RequiredOrbitMatchDeltaV = 0.0;
			_smoothedOrbitMatchBurnDirection = double2.zero;
			_hasSmoothedOrbitMatchBurnDirection = false;
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

		/// <summary>
		/// Numerically propagates the powered rendezvous trajectory over time to generate a renderable path.
		/// </summary>
		public List<double2> PredictRendezvousPath(double maxTime = -1.0, int steps = 150)
		{
			List<double2> path = new List<double2>();
			if (OwnerEntity == null || OwnerEntity.IsDestroyed || OwnerEntity.PhysicsBody == null ||
				OwnerEntity.Mass <= 0.0 || MaxForce <= 0f ||
				(TargetEntity != null && TargetEntity.IsDestroyed))
				return path;

			UpdateTargetState();
			if (!IsFinite(TargetPosition.x) || !IsFinite(TargetPosition.y) ||
				!IsFinite(TargetVelocity.x) || !IsFinite(TargetVelocity.y))
				return path;

			PhysicsManager2D manager = PhysicsManager2D.Instance;
			double controlStep = manager != null && manager.IsInitialized && manager.ScaledPhysicsTimeStep > 0f
				? manager.ScaledPhysicsTimeStep
				: math.max(0.0001f, Time.fixedDeltaTime);
			double predictionLimit = maxTime > 0.0 ? maxTime : 7.0 * 24.0 * 60.0 * 60.0;
			double preferredTime = TimeOfFlight > 0f ? TimeOfFlight : -1.0;

			RendezvousGuidanceSolver.TrySolve(
				OwnerEntity,
				TargetEntity,
				TargetPosition,
				TargetVelocity,
				MaxForce,
				ArrivalThreshold,
				RendezvousVelocityThreshold,
				controlStep,
				preferredTime,
				out _,
				out _,
				path,
				math.max(1, steps),
				predictionLimit);

			return path;
		}

	}
}