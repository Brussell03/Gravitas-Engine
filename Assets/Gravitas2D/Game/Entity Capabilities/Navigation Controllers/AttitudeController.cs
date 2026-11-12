using Gravitas2D;
using Unity.Mathematics;
using UnityEngine;

namespace GravitasDemo
{
	public class AttitudeController
	{
		public bool IsEnabled { get; private set; }
		protected NavigationCapability _manager;

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

		public AttitudeController(NavigationCapability manager)
		{
			_manager = manager;
		}

		public void Clear()
		{
			IsEnabled = false;
			_targetAngularVelocity = 0f;
			_targetAngularAcceleration = 0f;
			_hasTargetOrientationSample = false;
		}

		public bool CheckCompleted(float dt)
		{
			return IsBurnAligned(dt);
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

			double inertia = _manager.OwnerEntity != null && _manager.OwnerEntity.PhysicsBody != null
				? _manager.OwnerEntity.PhysicsBody.Inertia
				: 1.0;
			if (inertia <= 0.0 || double.IsNaN(inertia) || double.IsInfinity(inertia))
				inertia = 1.0;

			float torqueLimit = math.abs(_manager.MaxTorque);
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
				_manager.TargetOrientation = newTargetOrientation;
				_targetAngularVelocity = 0f;
				_targetAngularAcceleration = 0f;
				_hasTargetOrientationSample = false;
				return;
			}

			float deltaAngle = WrapAngle(newTargetOrientation - _manager.TargetOrientation);
			_manager.TargetOrientation = newTargetOrientation;

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
			if (!IsEnabled || _manager.OwnerEntity == null || _manager.OwnerEntity.PhysicsBody == null) return 0f;
			deltaTime = PrepareAttitudeControl(deltaTime);

			double inertia = _manager.OwnerEntity.PhysicsBody.Inertia;
			if (inertia <= 0.0 || double.IsNaN(inertia) || double.IsInfinity(inertia)) inertia = 1.0;
			float torqueLimit = math.abs(_manager.MaxTorque);
			if (torqueLimit <= 0f) return 0f;

			float alphaMax = (float)(torqueLimit / inertia);
			if (alphaMax <= 0f || !IsFinite(alphaMax)) return 0f;

			float predictionTime = math.min(
				math.max(_controlStep, _effectivePredictionLeadTime),
				_effectivePredictionHorizon);
			float bodyOmega = _manager.OwnerEntity.AngularVelocity;

			// Reserve at least half of the available angular acceleration for feedback/braking.
			float targetAlpha = math.clamp(_targetAngularAcceleration, -0.5f * alphaMax, 0.5f * alphaMax);
			float feedbackAlphaMax = math.max(0.5f * alphaMax, alphaMax - math.abs(targetAlpha));

			float predictedTargetAngle = _manager.TargetOrientation
				+ (_targetAngularVelocity * predictionTime)
				+ (0.5f * targetAlpha * predictionTime * predictionTime);
			float predictedBodyAngle = _manager.OwnerEntity.Orientation
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

		public void SetTargetOrientation(float targetOrientation)
		{
			_manager.TargetOrientation = targetOrientation;
			_targetAngularVelocity = 0f;
			_targetAngularAcceleration = 0f;
			_hasTargetOrientationSample = false;
			IsEnabled = true;
		}

		private static float WrapAngle(float angle)
		{
			while (angle > Mathf.PI) angle -= 2f * Mathf.PI;
			while (angle < -Mathf.PI) angle += 2f * Mathf.PI;
			return angle;
		}

		private static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);

		/// <summary>Returns true when the force direction for the current physics step is aligned.</summary>
		public bool IsBurnAligned(float deltaTime)
		{
			if (_manager.OwnerEntity == null || _manager.OwnerEntity.PhysicsBody == null) return false;
			PrepareAttitudeControl(deltaTime);

			float error = WrapAngle(_manager.TargetOrientation - _manager.OwnerEntity.Orientation);
			float relativeOmega = _manager.OwnerEntity.AngularVelocity - _targetAngularVelocity;

			return math.abs(error) <= _effectiveBurnAlignmentTolerance
				&& math.abs(relativeOmega) <= _effectiveBurnRateTolerance;
		}
	}
}