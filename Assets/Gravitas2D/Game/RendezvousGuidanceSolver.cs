using System;
using System.Collections.Generic;
using System.Diagnostics;
using Gravitas2D;
using Unity.Mathematics;

namespace GravitasDemo
{
	/// <summary>
	/// Receding-horizon rendezvous guidance. The optimizer predicts the powered vehicle and
	/// target trajectories through the live celestial gravity field, then solves for a bounded
	/// sequence of world-space accelerations that drives terminal relative position and velocity
	/// to zero. Only the first control interval is applied; the plan is recomputed every update.
	/// </summary>
	internal static class RendezvousGuidanceSolver
	{
		private const int ControlIntervals = 2;
		private const int OptimizerIterations = 3;
		private const int HorizonTrials = 5;
		private const int HorizonRefinementIterations = 3;
		private static readonly double[] HorizonFactors = { 0.55, 0.85, 1.15, 1.6, 2.3 };

		// A plan is owned by one NavigationCapability. After the first feasible solve, the
		// previous second-segment control is an excellent warm start for the next frame.
		// This avoids repeating the expensive horizon scan on every physics tick.
		internal sealed class PlanMemory
		{
			public bool HasPlan;
			public double2 FutureControl;
			public double LastSolveMilliseconds;
			public int LastTerminalEvaluationCount;
			public int LastVehicleIntegrationStepCount;
			public int LastGravityEphemerisSegmentCount;

			public void Reset()
			{
				HasPlan = false;
				FutureControl = double2.zero;
				LastSolveMilliseconds = 0.0;
				LastTerminalEvaluationCount = 0;
				LastVehicleIntegrationStepCount = 0;
				LastGravityEphemerisSegmentCount = 0;
			}
		}

		// All rendezvous solvers share one read-only view of the physics-layer celestial
		// ephemeris for the current completed physics state.
		private static GravityEphemerisView _sharedGravity;
		private static double _sharedGravityEpoch = double.NaN;
		private const double MinimumAcceleration = 1e-8;
		private const double MaximumPredictionHorizon = 7.0 * 24.0 * 60.0 * 60.0;
		private const int MaximumVehicleSubstepsPerInterval = 256;

		// Runtime diagnostics exposed through NavigationCapability for profiling in Unity.
		public static double LastSolveMilliseconds { get; private set; }
		public static int LastTerminalEvaluationCount { get; private set; }
		public static int LastVehicleIntegrationStepCount { get; private set; }
		public static int LastGravityEphemerisSegmentCount { get; private set; }

		private static void FinishTiming(Stopwatch stopwatch, PlanMemory planMemory)
		{
			stopwatch.Stop();
			LastSolveMilliseconds = stopwatch.Elapsed.TotalMilliseconds;
			if (planMemory != null)
			{
				planMemory.LastSolveMilliseconds = LastSolveMilliseconds;
				planMemory.LastTerminalEvaluationCount = LastTerminalEvaluationCount;
				planMemory.LastVehicleIntegrationStepCount = LastVehicleIntegrationStepCount;
				planMemory.LastGravityEphemerisSegmentCount = LastGravityEphemerisSegmentCount;
			}
		}

		private struct DynamicState
		{
			public double2 Position;
			public double2 Velocity;
			public double2 TargetCenterPosition;
			public double2 TargetCenterVelocity;

			// Forward sensitivities of vehicle x/y/vx/vy to the four control components
			// [a0x, a0y, a1x, a1y]. They allow a full shooting Jacobian from one rollout.
			public double4 PositionSensitivityX;
			public double4 PositionSensitivityY;
			public double4 VelocitySensitivityX;
			public double4 VelocitySensitivityY;
		}

		private struct Derivative
		{
			public double2 Position;
			public double2 Velocity;
			public double2 TargetPosition;
			public double2 TargetVelocity;
			public double4 PositionSensitivityX;
			public double4 PositionSensitivityY;
			public double4 VelocitySensitivityX;
			public double4 VelocitySensitivityY;
		}

		private struct TerminalEvaluation
		{
			public TerminalError Error;
			public double4 Row0;
			public double4 Row1;
			public double4 Row2;
			public double4 Row3;
		}

		private struct PlanResult
		{
			public double2 FirstAcceleration;
			public double PositionMiss;
			public double VelocityMiss;
			public double Score;
			public double Horizon;
			public bool Valid;
			public bool Feasible;
			public double2[] Controls;
			public double[] Durations;
		}

		/// <summary>
		/// Finds a bounded receding-horizon control. Returns false only when the input state is
		/// invalid; the caller should use its completion test before invoking this method.
		/// </summary>
		public static bool TrySolve(
			Entity owner,
			Entity targetEntity,
			double2 targetPosition,
			double2 targetVelocity,
			double maxForce,
			double arrivalThreshold,
			double velocityThreshold,
			double controlStep,
			double previousTimeOfFlight,
			out double2 commandedAcceleration,
			out float timeOfFlight,
			List<double2> predictedPath = null,
			int pathSamples = 150,
			double maxPredictionTime = MaximumPredictionHorizon,
			PlanMemory planMemory = null)
		{
			Stopwatch stopwatch = Stopwatch.StartNew();
			LastTerminalEvaluationCount = 0;
			LastVehicleIntegrationStepCount = 0;
			LastGravityEphemerisSegmentCount = 0;
			commandedAcceleration = double2.zero;
			timeOfFlight = 0f;

			if (owner == null || owner.IsDestroyed || owner.PhysicsBody == null ||
				owner.Mass <= 0.0 || maxForce <= 0.0 ||
				!IsFinite(owner.Position) || !IsFinite(owner.Velocity) ||
				!IsFinite(targetPosition) || !IsFinite(targetVelocity))
			{
				FinishTiming(stopwatch, planMemory);
				return false;
			}

			double maxAcceleration = maxForce / owner.Mass;
			if (!IsFinite(maxAcceleration) || maxAcceleration <= MinimumAcceleration)
			{
				FinishTiming(stopwatch, planMemory);
				return false;
			}

			controlStep = math.max(1e-4, IsFinite(controlStep) ? controlStep : 0.02);
			maxPredictionTime = IsFinite(maxPredictionTime)
				? math.clamp(maxPredictionTime, controlStep * 8.0, MaximumPredictionHorizon)
				: MaximumPredictionHorizon;
			arrivalThreshold = math.max(0.01, arrivalThreshold);
			velocityThreshold = math.max(0.01, velocityThreshold);

			double2 initialRelativePosition = targetPosition - owner.Position;
			double2 initialRelativeVelocity = targetVelocity - owner.Velocity;
			double initialDistance = math.length(initialRelativePosition);
			double initialRelativeSpeed = math.length(initialRelativeVelocity);
			if (!IsFinite(initialDistance) || !IsFinite(initialRelativeSpeed))
			{
				FinishTiming(stopwatch, planMemory);
				return false;
			}

			double distanceTime = 2.0 * math.sqrt(math.max(0.0, initialDistance) / maxAcceleration);
			double velocityTime = 2.0 * initialRelativeSpeed / maxAcceleration;
			double seedTime = math.max(controlStep * 12.0, math.max(distanceTime, velocityTime));
			if (IsFinite(previousTimeOfFlight) && previousTimeOfFlight > controlStep * 4.0)
				seedTime = math.max(seedTime * 0.65, math.min(seedTime, previousTimeOfFlight * 0.9));
			seedTime = math.clamp(seedTime, math.min(controlStep * 8.0, maxPredictionTime / 2.8), maxPredictionTime / 2.8);

			double positionTolerance = math.max(0.001, arrivalThreshold * 0.10);
			double velocityTolerance = math.max(0.001, math.min(velocityThreshold * 0.5, 0.25));
			double positionScale = math.max(positionTolerance, initialDistance * 0.01);
			double velocityScale = math.max(velocityTolerance, initialRelativeSpeed * 0.01);

			double2 targetCenterPosition = targetEntity != null ? targetEntity.Position : targetPosition;
			double2 targetCenterVelocity = targetEntity != null ? targetEntity.Velocity : targetVelocity;
			double2 targetOffset = targetPosition - targetCenterPosition;
			int ownerIndex = owner.PhysicsBody.Index;
			int targetIndex = targetEntity != null ? targetEntity.PhysicsBody.Index : -1;
			bool targetIsDynamic = targetEntity != null;

			PhysicsManager2D manager = PhysicsManager2D.Instance;
			double maximumTrialHorizon = math.clamp(seedTime * HorizonFactors[HorizonTrials - 1],
				controlStep * 4.0, maxPredictionTime);
			if (planMemory != null && planMemory.HasPlan && previousTimeOfFlight > controlStep * 4.0 && predictedPath == null)
				maximumTrialHorizon = math.max(maximumTrialHorizon,
					math.min(maxPredictionTime, previousTimeOfFlight * 2.0));

			GravityEphemerisView gravity = manager != null && manager.IsInitialized
				? GetSharedGravityEphemeris(manager, maximumTrialHorizon)
				: default;
			LastGravityEphemerisSegmentCount = gravity.SegmentCount;

			int targetGravitySource = targetEntity != null && targetEntity.IsCelestial
				? gravity.FindSource(targetIndex)
				: -1;
			DynamicState initialState = new DynamicState
			{
				Position = owner.Position,
				Velocity = owner.Velocity,
				TargetCenterPosition = targetCenterPosition,
				TargetCenterVelocity = targetCenterVelocity
			};

			PlanResult best = default;
			bool foundFeasible = false;
			double minimumHorizon = controlStep * 4.0;

			// Steady-state MPC: keep the previously solved arrival time, subtract the elapsed
			// control interval, and warm-start with the old coast/burn segment. A prior feasible
			// two-segment solution naturally shifts to [old segment 2, old segment 2]. This turns
			// the normal per-tick workload into one bounded shooting solve instead of 5 scans + 3
			// binary-search solves. Re-expand only when the warm-started plan loses feasibility.
			bool useWarmStart = planMemory != null && planMemory.HasPlan &&
				previousTimeOfFlight > controlStep * 4.0 && predictedPath == null;
			if (useWarmStart)
			{
				double horizon = math.clamp(previousTimeOfFlight - controlStep, minimumHorizon, maxPredictionTime);
				for (int trial = 0; trial < 3; trial++)
				{
					if (trial == 1) horizon = math.min(maxPredictionTime, horizon * 1.25);
					else if (trial == 2) horizon = math.min(maxPredictionTime, horizon * 1.5);
					double[] durations = MakeControlDurations(horizon, controlStep);
					double2[] warmControls = { planMemory.FutureControl, planMemory.FutureControl };
					PlanResult candidate = OptimizeHorizon(
						initialState, targetIsDynamic, targetGravitySource,
						ownerIndex, targetIndex, targetOffset, durations, gravity,
						maxAcceleration, positionScale, velocityScale, positionTolerance, velocityTolerance,
						warmControls);
					if (candidate.Valid && (!best.Valid || candidate.Score < best.Score)) best = candidate;
					if (candidate.Feasible)
					{
						best = candidate;
						foundFeasible = true;
						break;
					}
				}
			}
			else
			{
				double lowerInfeasibleHorizon = minimumHorizon;
				double upperFeasibleHorizon = 0.0;
				for (int trial = 0; trial < HorizonTrials; trial++)
				{
					double horizon = math.clamp(seedTime * HorizonFactors[trial], minimumHorizon, maxPredictionTime);
					double[] durations = MakeControlDurations(horizon, controlStep);
					PlanResult candidate = OptimizeHorizon(
						initialState, targetIsDynamic, targetGravitySource,
						ownerIndex, targetIndex, targetOffset, durations, gravity,
						maxAcceleration, positionScale, velocityScale, positionTolerance, velocityTolerance);
					if (candidate.Valid && (!best.Valid || candidate.Score < best.Score)) best = candidate;
					if (candidate.Feasible)
					{
						best = candidate;
						foundFeasible = true;
						upperFeasibleHorizon = horizon;
						break;
					}
					lowerInfeasibleHorizon = horizon;
				}

				if (foundFeasible && upperFeasibleHorizon > lowerInfeasibleHorizon + controlStep)
				{
					for (int i = 0; i < HorizonRefinementIterations; i++)
					{
						double horizon = 0.5 * (lowerInfeasibleHorizon + upperFeasibleHorizon);
						double[] durations = MakeControlDurations(horizon, controlStep);
						PlanResult candidate = OptimizeHorizon(
							initialState, targetIsDynamic, targetGravitySource,
							ownerIndex, targetIndex, targetOffset, durations, gravity,
							maxAcceleration, positionScale, velocityScale, positionTolerance, velocityTolerance,
							best.Controls);
						if (candidate.Feasible)
						{
							best = candidate;
							upperFeasibleHorizon = horizon;
						}
						else lowerInfeasibleHorizon = horizon;
					}
				}
			}

			if (best.Valid && IsFinite(best.FirstAcceleration))
			{
				commandedAcceleration = ClampMagnitude(best.FirstAcceleration, maxAcceleration);
				timeOfFlight = (float)math.min(best.Horizon, float.MaxValue);
				if (planMemory != null)
				{
					planMemory.FutureControl = best.Controls != null && best.Controls.Length > 1
						? best.Controls[1]
						: best.FirstAcceleration;
					planMemory.HasPlan = best.Feasible || useWarmStart;
				}
			}
			else
			{
				double2 ownerGravity = gravity.AccelerationAt(owner.Position, 0.0, ownerIndex);
				double2 targetGravity = targetIsDynamic
					? gravity.AccelerationAt(targetCenterPosition, 0.0, targetIndex)
					: double2.zero;
				double omega = math.sqrt(maxAcceleration / math.max(initialDistance, arrivalThreshold));
				omega = math.min(omega, 0.25 / controlStep);
				commandedAcceleration = (targetGravity - ownerGravity)
					+ initialRelativePosition * (omega * omega)
					+ initialRelativeVelocity * (2.0 * omega);
				commandedAcceleration = ClampMagnitude(commandedAcceleration, maxAcceleration);
				timeOfFlight = (float)math.min(seedTime, float.MaxValue);
				if (planMemory != null) planMemory.Reset();
			}

			if (predictedPath != null)
			{
				if (best.Valid && best.Controls != null && best.Durations != null)
					BuildPredictedPath(predictedPath, pathSamples, initialState, best.Controls, best.Durations,
						gravity, targetIsDynamic, targetGravitySource, ownerIndex, targetIndex);
				else
					BuildPredictedPath(predictedPath, pathSamples, initialState,
						MakeConstantControls(commandedAcceleration, timeOfFlight, controlStep),
						MakeControlDurations(math.max(controlStep * 4.0, timeOfFlight), controlStep),
						gravity, targetIsDynamic, targetGravitySource, ownerIndex, targetIndex);
			}

			bool success = IsFinite(commandedAcceleration) && IsFinite(timeOfFlight);
			FinishTiming(stopwatch, planMemory);
			return success;
		}

		private static GravityEphemerisView GetSharedGravityEphemeris(PhysicsManager2D manager, double requestedHorizon)
		{
			double epoch = manager.SimulationElapsedTime;
			if (!_sharedGravity.IsValid || epoch != _sharedGravityEpoch ||
				requestedHorizon > _sharedGravity.Coverage + 1e-6)
			{
				_sharedGravity = manager.GetGravityEphemerisView(requestedHorizon);
				_sharedGravityEpoch = epoch;
			}
			return _sharedGravity;
		}

		private static PlanResult OptimizeHorizon(
			DynamicState initialState,
			bool targetIsDynamic,
			int targetGravitySource,
			int ownerIndex,
			int targetIndex,
			double2 targetOffset,
			double[] durations,
			GravityEphemerisView gravity,
			double maxAcceleration,
			double positionScale,
			double velocityScale,
			double positionTolerance,
			double velocityTolerance,
			double2[] warmStartControls = null)
		{
			int count = durations.Length;
			double horizon = 0.0;
			for (int i = 0; i < count; i++) horizon += durations[i];
			double2[] controls = new double2[count];

			if (warmStartControls != null && warmStartControls.Length == count)
			{
				for (int i = 0; i < count; i++) controls[i] = ClampMagnitude(warmStartControls[i], maxAcceleration);
			}
			else
			{
				// Analytic double-integrator seed. It is corrected by the gravity-aware
				// variational shooting solve below.
				double2 initialRelativePosition = initialState.TargetCenterPosition + targetOffset - initialState.Position;
				double2 initialRelativeVelocity = initialState.TargetCenterVelocity - initialState.Velocity;
				double2 targetGravity = targetIsDynamic
					? gravity.AccelerationAt(initialState.TargetCenterPosition + targetOffset, 0.0, targetIndex)
					: double2.zero;
				double2 initialDifferentialGravity = targetGravity - gravity.AccelerationAt(initialState.Position, 0.0, ownerIndex);
				double h1 = durations[0];
				double h2 = durations[1];
				double a = h1 * (0.5 * h1 + h2);
				double b = 0.5 * h2 * h2;
				double determinant = (h1 * b) - (h2 * a);
				double2 requiredVelocityIntegral = initialRelativeVelocity + initialDifferentialGravity * horizon;
				double2 requiredPositionIntegral = initialRelativePosition
					+ initialRelativeVelocity * horizon
					+ 0.5 * initialDifferentialGravity * horizon * horizon;
				if (math.abs(determinant) > 1e-18)
				{
					controls[0] = ClampMagnitude((requiredVelocityIntegral * b - requiredPositionIntegral * h2) / determinant, maxAcceleration);
					controls[1] = ClampMagnitude((requiredPositionIntegral * h1 - requiredVelocityIntegral * a) / determinant, maxAcceleration);
				}
			}

			TerminalEvaluation evaluation = EvaluateTerminalWithJacobian(
				initialState, controls, durations, gravity, targetIsDynamic,
				targetGravitySource, ownerIndex, targetIndex, targetOffset, positionScale, velocityScale);
			double bestScore = Score(evaluation.Error, controls, maxAcceleration);

			for (int iteration = 0; iteration < OptimizerIterations; iteration++)
			{
				TerminalError error = evaluation.Error;
				if (error.PositionMiss <= positionTolerance && error.VelocityMiss <= velocityTolerance)
					break;

				// Analytic variational Jacobian: integrate the four control sensitivities together
				// with the nominal trajectory. This replaces four additional finite-difference
				// rollouts per optimizer iteration.
				double[,] normal = new double[4, 4];
				double4[] rows = { evaluation.Row0, evaluation.Row1, evaluation.Row2, evaluation.Row3 };
				double trace = 0.0;
				for (int row = 0; row < 4; row++)
				{
					for (int col = 0; col < 4; col++) normal[row, col] = math.dot(rows[row], rows[col]);
					trace += normal[row, row];
				}

				double damping = math.max(1e-10, trace * 1e-8);
				double[] rhs = { error.X, error.Y, error.Vx, error.Vy };
				for (int i = 0; i < 4; i++) normal[i, i] += damping;
				if (!SolveLinear4(normal, rhs)) break;

				// delta = -J^T (J J^T + lambda I)^-1 e
				double2[] delta = new double2[count];
				for (int i = 0; i < count; i++)
				{
					int xColumn = i * 2;
					int yColumn = xColumn + 1;
					double dx = 0.0;
					double dy = 0.0;
					for (int row = 0; row < 4; row++)
					{
						dx -= GetComponent(rows[row], xColumn) * rhs[row];
						dy -= GetComponent(rows[row], yColumn) * rhs[row];
					}
					delta[i] = new double2(dx, dy);
				}

				bool accepted = false;
				for (int lineSearch = 0; lineSearch < 2; lineSearch++)
				{
					double alpha = lineSearch == 0 ? 1.0 : 0.5;
					double2[] candidateControls = new double2[count];
					for (int i = 0; i < count; i++)
						candidateControls[i] = ClampMagnitude(controls[i] + delta[i] * alpha, maxAcceleration);

					TerminalError candidateError = EvaluateTerminal(
						initialState, candidateControls, durations, gravity, targetIsDynamic,
						targetGravitySource, ownerIndex, targetIndex, targetOffset, positionScale, velocityScale);
					double candidateScore = Score(candidateError, candidateControls, maxAcceleration);
					if (candidateScore < bestScore - math.max(1e-12, bestScore * 1e-10))
					{
						controls = candidateControls;
						bestScore = candidateScore;
						evaluation = EvaluateTerminalWithJacobian(
							initialState, controls, durations, gravity, targetIsDynamic,
							targetGravitySource, ownerIndex, targetIndex, targetOffset, positionScale, velocityScale);
						accepted = true;
						break;
					}
				}
				if (!accepted) break;
			}

			TerminalError finalError = evaluation.Error;
			return new PlanResult
			{
				FirstAcceleration = controls[0],
				PositionMiss = finalError.PositionMiss,
				VelocityMiss = finalError.VelocityMiss,
				Score = Score(finalError, controls, maxAcceleration),
				Horizon = horizon,
				Valid = IsFinite(controls[0]) && IsFinite(finalError.PositionMiss) && IsFinite(finalError.VelocityMiss),
				Feasible = finalError.PositionMiss <= positionTolerance && finalError.VelocityMiss <= velocityTolerance,
				Controls = (double2[])controls.Clone(),
				Durations = (double[])durations.Clone()
			};
		}

		private static double GetComponent(double4 value, int index)
		{
			switch (index)
			{
				case 0: return value.x;
				case 1: return value.y;
				case 2: return value.z;
				default: return value.w;
			}
		}

		private static double2[] MakeConstantControls(double2 acceleration, double horizon, double controlStep)
		{
			double[] durations = MakeControlDurations(math.max(controlStep * 4.0, horizon), controlStep);
			double2[] controls = new double2[durations.Length];
			for (int i = 0; i < controls.Length; i++) controls[i] = acceleration;
			return controls;
		}

		private static void BuildPredictedPath(
			List<double2> output,
			int sampleCount,
			DynamicState initialState,
			double2[] controls,
			double[] durations,
			GravityEphemerisView gravity,
			bool targetIsDynamic,
			int targetGravitySource,
			int ownerIndex,
			int targetIndex)
		{
			output.Clear();
			output.Add(initialState.Position);
			if (controls == null || durations == null || controls.Length == 0 || controls.Length != durations.Length)
				return;

			sampleCount = math.max(1, sampleCount);
			double horizon = 0.0;
			for (int i = 0; i < durations.Length; i++) horizon += durations[i];

			DynamicState state = initialState;
			int controlIndex = 0;
			double controlRemaining = durations[0];
			double currentTime = 0.0;
			for (int sample = 1; sample <= sampleCount; sample++)
			{
				double sampleTime = horizon * sample / sampleCount;
				while (currentTime < sampleTime - 1e-10 && controlIndex < controls.Length)
				{
					if (controlRemaining <= 1e-10)
					{
						controlIndex++;
						if (controlIndex >= controls.Length) break;
						controlRemaining = durations[controlIndex];
					}

					double segment = math.min(sampleTime - currentTime, controlRemaining);
					IntegrateInterval(ref state, currentTime, segment, controls[controlIndex], gravity,
						targetIsDynamic, targetGravitySource, ownerIndex, targetIndex);
					currentTime += segment;
					controlRemaining -= segment;
				}
				output.Add(state.Position);
			}
		}

		private struct TerminalError
		{
			public double X, Y, Vx, Vy;
			public double PositionMiss, VelocityMiss;
		}

		private static TerminalError EvaluateTerminal(
			DynamicState initialState,
			double2[] controls,
			double[] durations,
			GravityEphemerisView gravity,
			bool targetIsDynamic,
			int targetGravitySource,
			int ownerIndex,
			int targetIndex,
			double2 targetOffset,
			double positionScale,
			double velocityScale)
		{
			LastTerminalEvaluationCount++;
			DynamicState state = initialState;
			double time = 0.0;
			for (int i = 0; i < controls.Length; i++)
			{
				IntegrateInterval(ref state, time, durations[i], controls[i], gravity,
					targetIsDynamic, targetGravitySource, ownerIndex, targetIndex);
				time += durations[i];
			}

			double2 predictedTargetPosition;
			double2 predictedTargetVelocity;
			if (targetGravitySource >= 0)
			{
				gravity.SourceStateAt(targetGravitySource, time, out predictedTargetPosition, out predictedTargetVelocity);
			}
			else
			{
				predictedTargetPosition = state.TargetCenterPosition;
				predictedTargetVelocity = state.TargetCenterVelocity;
			}

			predictedTargetPosition += targetOffset;
			double2 positionError = predictedTargetPosition - state.Position;
			double2 velocityError = predictedTargetVelocity - state.Velocity;
			return new TerminalError
			{
				X = positionError.x / positionScale,
				Y = positionError.y / positionScale,
				Vx = velocityError.x / velocityScale,
				Vy = velocityError.y / velocityScale,
				PositionMiss = math.length(positionError),
				VelocityMiss = math.length(velocityError)
			};
		}

		private static TerminalEvaluation EvaluateTerminalWithJacobian(
			DynamicState initialState,
			double2[] controls,
			double[] durations,
			GravityEphemerisView gravity,
			bool targetIsDynamic,
			int targetGravitySource,
			int ownerIndex,
			int targetIndex,
			double2 targetOffset,
			double positionScale,
			double velocityScale)
		{
			LastTerminalEvaluationCount++;
			DynamicState state = initialState;
			double time = 0.0;
			for (int i = 0; i < controls.Length; i++)
			{
				IntegrateIntervalWithSensitivity(ref state, time, durations[i], controls[i], i,
					gravity, targetIsDynamic, targetGravitySource, ownerIndex, targetIndex);
				time += durations[i];
			}

			double2 predictedTargetPosition;
			double2 predictedTargetVelocity;
			if (targetGravitySource >= 0)
				gravity.SourceStateAt(targetGravitySource, time, out predictedTargetPosition, out predictedTargetVelocity);
			else
			{
				predictedTargetPosition = state.TargetCenterPosition;
				predictedTargetVelocity = state.TargetCenterVelocity;
			}

			predictedTargetPosition += targetOffset;
			double2 positionError = predictedTargetPosition - state.Position;
			double2 velocityError = predictedTargetVelocity - state.Velocity;
			return new TerminalEvaluation
			{
				Error = new TerminalError
				{
					X = positionError.x / positionScale,
					Y = positionError.y / positionScale,
					Vx = velocityError.x / velocityScale,
					Vy = velocityError.y / velocityScale,
					PositionMiss = math.length(positionError),
					VelocityMiss = math.length(velocityError)
				},
				Row0 = -state.PositionSensitivityX / positionScale,
				Row1 = -state.PositionSensitivityY / positionScale,
				Row2 = -state.VelocitySensitivityX / velocityScale,
				Row3 = -state.VelocitySensitivityY / velocityScale
			};
		}

		private static void IntegrateIntervalWithSensitivity(
			ref DynamicState state,
			double time,
			double step,
			double2 control,
			int controlIndex,
			GravityEphemerisView gravity,
			bool targetIsDynamic,
			int targetGravitySource,
			int ownerIndex,
			int targetIndex)
		{
			double maximumStep = step > 3600.0 ? 30.0 : step > 600.0 ? 15.0 : 5.0;
			int substeps = (int)math.clamp(math.ceil(step / maximumStep), 1.0, MaximumVehicleSubstepsPerInterval);
			LastVehicleIntegrationStepCount += substeps;
			double substep = step / substeps;
			for (int i = 0; i < substeps; i++)
				IntegrateSingleStepWithSensitivity(ref state, time + i * substep, substep,
					control, controlIndex, gravity, targetIsDynamic, targetGravitySource, ownerIndex, targetIndex);
		}

		private static void IntegrateSingleStepWithSensitivity(
			ref DynamicState state,
			double time,
			double step,
			double2 control,
			int controlIndex,
			GravityEphemerisView gravity,
			bool targetIsDynamic,
			int targetGravitySource,
			int ownerIndex,
			int targetIndex)
		{
			Derivative k1 = EvaluateDerivativeWithSensitivity(state, time, control, controlIndex, gravity,
				targetIsDynamic, targetGravitySource, ownerIndex, targetIndex);
			DynamicState s2 = AddScaled(state, k1, step * 0.5);
			Derivative k2 = EvaluateDerivativeWithSensitivity(s2, time + step * 0.5, control, controlIndex, gravity,
				targetIsDynamic, targetGravitySource, ownerIndex, targetIndex);
			DynamicState s3 = AddScaled(state, k2, step * 0.5);
			Derivative k3 = EvaluateDerivativeWithSensitivity(s3, time + step * 0.5, control, controlIndex, gravity,
				targetIsDynamic, targetGravitySource, ownerIndex, targetIndex);
			DynamicState s4 = AddScaled(state, k3, step);
			Derivative k4 = EvaluateDerivativeWithSensitivity(s4, time + step, control, controlIndex, gravity,
				targetIsDynamic, targetGravitySource, ownerIndex, targetIndex);

			state.Position += (step / 6.0) * (k1.Position + 2.0 * k2.Position + 2.0 * k3.Position + k4.Position);
			state.Velocity += (step / 6.0) * (k1.Velocity + 2.0 * k2.Velocity + 2.0 * k3.Velocity + k4.Velocity);
			state.PositionSensitivityX += (step / 6.0) * (k1.PositionSensitivityX + 2.0 * k2.PositionSensitivityX + 2.0 * k3.PositionSensitivityX + k4.PositionSensitivityX);
			state.PositionSensitivityY += (step / 6.0) * (k1.PositionSensitivityY + 2.0 * k2.PositionSensitivityY + 2.0 * k3.PositionSensitivityY + k4.PositionSensitivityY);
			state.VelocitySensitivityX += (step / 6.0) * (k1.VelocitySensitivityX + 2.0 * k2.VelocitySensitivityX + 2.0 * k3.VelocitySensitivityX + k4.VelocitySensitivityX);
			state.VelocitySensitivityY += (step / 6.0) * (k1.VelocitySensitivityY + 2.0 * k2.VelocitySensitivityY + 2.0 * k3.VelocitySensitivityY + k4.VelocitySensitivityY);

			if (targetIsDynamic && targetGravitySource < 0)
			{
				state.TargetCenterPosition += (step / 6.0) * (k1.TargetPosition + 2.0 * k2.TargetPosition + 2.0 * k3.TargetPosition + k4.TargetPosition);
				state.TargetCenterVelocity += (step / 6.0) * (k1.TargetVelocity + 2.0 * k2.TargetVelocity + 2.0 * k3.TargetVelocity + k4.TargetVelocity);
			}
			else if (!targetIsDynamic)
				state.TargetCenterPosition += state.TargetCenterVelocity * step;
		}

		private static Derivative EvaluateDerivativeWithSensitivity(
			DynamicState state,
			double time,
			double2 control,
			int controlIndex,
			GravityEphemerisView gravity,
			bool targetIsDynamic,
			int targetGravitySource,
			int ownerIndex,
			int targetIndex)
		{
			gravity.AccelerationAndGradientAt(state.Position, time, ownerIndex,
				out double2 acceleration, out double gxx, out double gxy, out double gyy);
			double2 targetAcceleration = double2.zero;
			if (targetIsDynamic && targetGravitySource < 0)
				targetAcceleration = gravity.AccelerationAt(state.TargetCenterPosition, time, targetIndex);

			double4 controlX = controlIndex == 0 ? new double4(1.0, 0.0, 0.0, 0.0) : new double4(0.0, 0.0, 1.0, 0.0);
			double4 controlY = controlIndex == 0 ? new double4(0.0, 1.0, 0.0, 0.0) : new double4(0.0, 0.0, 0.0, 1.0);
			return new Derivative
			{
				Position = state.Velocity,
				Velocity = acceleration + control,
				TargetPosition = state.TargetCenterVelocity,
				TargetVelocity = targetAcceleration,
				PositionSensitivityX = state.VelocitySensitivityX,
				PositionSensitivityY = state.VelocitySensitivityY,
				VelocitySensitivityX = gxx * state.PositionSensitivityX + gxy * state.PositionSensitivityY + controlX,
				VelocitySensitivityY = gxy * state.PositionSensitivityX + gyy * state.PositionSensitivityY + controlY
			};
		}

		private static void IntegrateInterval(
			ref DynamicState state,
			double time,
			double step,
			double2 control,
			GravityEphemerisView gravity,
			bool targetIsDynamic,
			int targetGravitySource,
			int ownerIndex,
			int targetIndex)
		{
			// Keep close-in predictions highly resolved. For long interplanetary horizons,
			// scale the RK4 step while retaining enough samples per typical orbital timescale.
			double maximumStep = step > 3600.0 ? 30.0 : step > 600.0 ? 15.0 : 5.0;
			int substeps = (int)math.clamp(math.ceil(step / maximumStep), 1.0, MaximumVehicleSubstepsPerInterval);
			LastVehicleIntegrationStepCount += substeps;
			double substep = step / substeps;
			for (int i = 0; i < substeps; i++)
				IntegrateSingleStep(ref state, time + i * substep, substep, control, gravity,
					targetIsDynamic, targetGravitySource, ownerIndex, targetIndex);
		}

		private static void IntegrateSingleStep(
			ref DynamicState state,
			double time,
			double step,
			double2 control,
			GravityEphemerisView gravity,
			bool targetIsDynamic,
			int targetGravitySource,
			int ownerIndex,
			int targetIndex)
		{
			Derivative k1 = EvaluateDerivative(state, time, control, gravity, targetIsDynamic,
				targetGravitySource, ownerIndex, targetIndex);
			DynamicState s2 = AddScaled(state, k1, step * 0.5);
			Derivative k2 = EvaluateDerivative(s2, time + step * 0.5, control, gravity, targetIsDynamic,
				targetGravitySource, ownerIndex, targetIndex);
			DynamicState s3 = AddScaled(state, k2, step * 0.5);
			Derivative k3 = EvaluateDerivative(s3, time + step * 0.5, control, gravity, targetIsDynamic,
				targetGravitySource, ownerIndex, targetIndex);
			DynamicState s4 = AddScaled(state, k3, step);
			Derivative k4 = EvaluateDerivative(s4, time + step, control, gravity, targetIsDynamic,
				targetGravitySource, ownerIndex, targetIndex);

			state.Position += (step / 6.0) * (k1.Position + 2.0 * k2.Position + 2.0 * k3.Position + k4.Position);
			state.Velocity += (step / 6.0) * (k1.Velocity + 2.0 * k2.Velocity + 2.0 * k3.Velocity + k4.Velocity);

			// Celestial targets are read directly from the same integrated source ephemeris used
			// by the vehicle gravity model. Non-celestial targets are propagated as ballistic bodies.
			if (targetIsDynamic && targetGravitySource < 0)
			{
				state.TargetCenterPosition += (step / 6.0) * (k1.TargetPosition + 2.0 * k2.TargetPosition + 2.0 * k3.TargetPosition + k4.TargetPosition);
				state.TargetCenterVelocity += (step / 6.0) * (k1.TargetVelocity + 2.0 * k2.TargetVelocity + 2.0 * k3.TargetVelocity + k4.TargetVelocity);
			}
			else if (!targetIsDynamic)
			{
				state.TargetCenterPosition += state.TargetCenterVelocity * step;
			}
		}

		private static Derivative EvaluateDerivative(
			DynamicState state,
			double time,
			double2 control,
			GravityEphemerisView gravity,
			bool targetIsDynamic,
			int targetGravitySource,
			int ownerIndex,
			int targetIndex)
		{
			double2 targetAcceleration = double2.zero;
			if (targetIsDynamic && targetGravitySource < 0)
				targetAcceleration = gravity.AccelerationAt(state.TargetCenterPosition, time, targetIndex);

			return new Derivative
			{
				Position = state.Velocity,
				Velocity = gravity.AccelerationAt(state.Position, time, ownerIndex) + control,
				TargetPosition = state.TargetCenterVelocity,
				TargetVelocity = targetAcceleration
			};
		}

		private static DynamicState AddScaled(DynamicState state, Derivative derivative, double scale)
		{
			return new DynamicState
			{
				Position = state.Position + derivative.Position * scale,
				Velocity = state.Velocity + derivative.Velocity * scale,
				TargetCenterPosition = state.TargetCenterPosition + derivative.TargetPosition * scale,
				TargetCenterVelocity = state.TargetCenterVelocity + derivative.TargetVelocity * scale,
				PositionSensitivityX = state.PositionSensitivityX + derivative.PositionSensitivityX * scale,
				PositionSensitivityY = state.PositionSensitivityY + derivative.PositionSensitivityY * scale,
				VelocitySensitivityX = state.VelocitySensitivityX + derivative.VelocitySensitivityX * scale,
				VelocitySensitivityY = state.VelocitySensitivityY + derivative.VelocitySensitivityY * scale
			};
		}

		private static double[] MakeControlDurations(double horizon, double controlStep)
		{
			double[] durations = new double[ControlIntervals];
			double first = math.min(controlStep, horizon * 0.5);
			durations[0] = first;
			double remainder = math.max(1e-6, horizon - first);
			double rest = remainder / (ControlIntervals - 1);
			for (int i = 1; i < durations.Length; i++) durations[i] = rest;
			return durations;
		}

		private static double Score(TerminalError error, double2[] controls, double maxAcceleration)
		{
			double terminal = error.X * error.X + error.Y * error.Y + error.Vx * error.Vx + error.Vy * error.Vy;
			double effort = 0.0;
			for (int i = 0; i < controls.Length; i++)
				effort += math.lengthsq(controls[i]) / (maxAcceleration * maxAcceleration);
			return terminal + 1e-7 * effort / math.max(1, controls.Length);
		}

		private static bool SolveLinear4(double[,] matrix, double[] rhs)
		{
			for (int column = 0; column < 4; column++)
			{
				int pivot = column;
				double pivotMagnitude = math.abs(matrix[column, column]);
				for (int row = column + 1; row < 4; row++)
				{
					double magnitude = math.abs(matrix[row, column]);
					if (magnitude > pivotMagnitude)
					{
						pivot = row;
						pivotMagnitude = magnitude;
					}
				}
				if (pivotMagnitude < 1e-18 || !IsFinite(pivotMagnitude)) return false;

				if (pivot != column)
				{
					for (int j = column; j < 4; j++)
					{
						double swap = matrix[column, j];
						matrix[column, j] = matrix[pivot, j];
						matrix[pivot, j] = swap;
					}
					double rhsSwap = rhs[column];
					rhs[column] = rhs[pivot];
					rhs[pivot] = rhsSwap;
				}

				double divisor = matrix[column, column];
				for (int j = column; j < 4; j++) matrix[column, j] /= divisor;
				rhs[column] /= divisor;

				for (int row = 0; row < 4; row++)
				{
					if (row == column) continue;
					double factor = matrix[row, column];
					for (int j = column; j < 4; j++) matrix[row, j] -= factor * matrix[column, j];
					rhs[row] -= factor * rhs[column];
				}
			}
			return true;
		}

		private static double2 ClampMagnitude(double2 value, double maximum)
		{
			double magnitude = math.length(value);
			if (!IsFinite(magnitude)) return double2.zero;
			return magnitude > maximum && magnitude > 0.0 ? value * (maximum / magnitude) : value;
		}

		private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
		private static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
		private static bool IsFinite(double2 value) => IsFinite(value.x) && IsFinite(value.y);

	}
}