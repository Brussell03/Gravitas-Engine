using System;
using System.Collections.Generic;
using System.Diagnostics;
using Gravitas2D;
using Unity.Mathematics;

namespace GravitasDemo
{
	/// <summary>
	/// Receding-horizon rendezvous guidance. The optimizer predicts the powered vehicle and
	/// target trajectories through the live celestial gravity field, then solves a two-phase
	/// accelerate/brake trajectory that drives terminal relative position and velocity to zero.
	/// Only the current control step is applied; the plan is recomputed every update.
	/// </summary>
	internal static class RendezvousGuidanceSolver
	{
		private const int ControlIntervals = 2;
		private const int OptimizerIterations = 5;
		private const int HorizonRefinementIterations = 10;
		private const int WarmStartExpansionTrials = 4;
		private const double HorizonExpansionFactor = 1.35;

		// A plan is owned by one NavigationCapability. Both phase controls are retained so
		// the next receding-horizon solve starts from the previous accelerate/brake solution.
		internal sealed class PlanMemory
		{
			public bool HasPlan;
			public double2 CurrentControl;
			public double2 FutureControl;
			public double FirstPhaseDuration;
			public double LastSolveMilliseconds;
			public int LastTerminalEvaluationCount;
			public int LastVehicleIntegrationStepCount;
			public int LastGravityEphemerisSegmentCount;

			public void Reset()
			{
				HasPlan = false;
				CurrentControl = double2.zero;
				FutureControl = double2.zero;
				FirstPhaseDuration = 0.0;
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
		/// Value-only vehicle data required by the rendezvous predictor. GravityBodyIndex is
		/// the physics body's index used to exclude the vehicle from its own gravity field;
		/// use -1 when predicting a vehicle that is not registered in the physics world.
		/// </summary>
		internal struct VehicleInput
		{
			public double2 Position;
			public double2 Velocity;
			public double Mass;
			public int GravityBodyIndex;

			public VehicleInput(double2 position, double2 velocity, double mass, int gravityBodyIndex = -1)
			{
				Position = position;
				Velocity = velocity;
				Mass = mass;
				GravityBodyIndex = gravityBodyIndex;
			}
		}

		/// <summary>
		/// Value-only target data. CenterPosition/CenterVelocity describe the body's centre;
		/// PositionOffset is added to its predicted centre position at arrival. For a free-space
		/// waypoint, set IsDynamic=false, GravityBodyIndex=-1, and use the waypoint as the centre
		/// position with zero offset. A non-celestial dynamic body is propagated through gravity;
		/// a celestial body is sampled from the shared gravity ephemeris.
		/// </summary>
		internal struct TargetInput
		{
			public double2 CenterPosition;
			public double2 CenterVelocity;
			public double2 PositionOffset;
			public bool IsDynamic;
			public bool IsCelestial;
			public int GravityBodyIndex;

			public TargetInput(
				double2 centerPosition,
				double2 centerVelocity,
				double2 positionOffset,
				bool isDynamic = false,
				bool isCelestial = false,
				int gravityBodyIndex = -1)
			{
				CenterPosition = centerPosition;
				CenterVelocity = centerVelocity;
				PositionOffset = positionOffset;
				IsDynamic = isDynamic;
				IsCelestial = isCelestial;
				GravityBodyIndex = gravityBodyIndex;
			}
		}

		/// <summary>
		/// Finds a bounded receding-horizon control using only value data; no Entity instances
		/// are required. Returns false when the supplied state or vehicle model is invalid.
		/// </summary>
		public static bool TrySolve(
			VehicleInput vehicle,
			TargetInput target,
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

			if (vehicle.Mass <= 0.0 || maxForce <= 0.0 ||
				!IsFinite(vehicle.Position) || !IsFinite(vehicle.Velocity) ||
				!IsFinite(target.CenterPosition) || !IsFinite(target.CenterVelocity) ||
				!IsFinite(target.PositionOffset))
			{
				FinishTiming(stopwatch, planMemory);
				return false;
			}

			double maxAcceleration = maxForce / vehicle.Mass;
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

			double2 targetPosition = target.CenterPosition + target.PositionOffset;
			double2 targetVelocity = target.CenterVelocity;
			double2 initialRelativePosition = targetPosition - vehicle.Position;
			double2 initialRelativeVelocity = targetVelocity - vehicle.Velocity;
			double initialDistance = math.length(initialRelativePosition);
			double initialRelativeSpeed = math.length(initialRelativeVelocity);
			if (!IsFinite(initialDistance) || !IsFinite(initialRelativeSpeed))
			{
				FinishTiming(stopwatch, planMemory);
				return false;
			}

			// The initial estimate is only used to bracket the minimum-time solve. The final
			// time is found by repeatedly testing the complete two-phase trajectory.
			double distanceTime = 2.0 * math.sqrt(math.max(0.0, initialDistance) / maxAcceleration);
			double velocityTime = 2.0 * initialRelativeSpeed / maxAcceleration;
			double seedTime = math.max(controlStep * 8.0, distanceTime + velocityTime);
			double minimumHorizon = controlStep * 4.0;
			seedTime = math.clamp(seedTime, minimumHorizon, maxPredictionTime);

			double positionTolerance = math.max(0.001, arrivalThreshold * 0.10);
			double velocityTolerance = math.max(0.001, math.min(velocityThreshold * 0.5, 0.25));
			double positionScale = math.max(positionTolerance, initialDistance * 0.01);
			double velocityScale = math.max(velocityTolerance, initialRelativeSpeed * 0.01);

			double2 targetCenterPosition = target.CenterPosition;
			double2 targetCenterVelocity = target.CenterVelocity;
			double2 targetOffset = target.PositionOffset;
			int ownerIndex = vehicle.GravityBodyIndex;
			int targetIndex = target.GravityBodyIndex;
			bool targetIsDynamic = target.IsDynamic;

			// Keep the ephemeris coverage large enough for the initial bracket and any warm-start
			// expansion, without eagerly requesting the full seven-day safety horizon.
			bool useWarmStart = planMemory != null && planMemory.HasPlan &&
				previousTimeOfFlight > controlStep * 4.0 && predictedPath == null;
			double previousRemaining = useWarmStart
				? math.max(minimumHorizon, previousTimeOfFlight - controlStep)
				: 0.0;
			double searchMaximumHorizon = math.min(maxPredictionTime,
				math.max(seedTime * 4.0, previousRemaining * 2.0));
			searchMaximumHorizon = math.max(minimumHorizon, searchMaximumHorizon);

			PhysicsManager2D manager = PhysicsManager2D.Instance;
			GravityEphemerisView gravity = manager != null && manager.IsInitialized
				? GetSharedGravityEphemeris(manager, searchMaximumHorizon)
				: default;
			LastGravityEphemerisSegmentCount = gravity.SegmentCount;

			int targetGravitySource = target.IsDynamic && target.IsCelestial
				? gravity.FindSource(targetIndex)
				: -1;
			DynamicState initialState = new DynamicState
			{
				Position = vehicle.Position,
				Velocity = vehicle.Velocity,
				TargetCenterPosition = targetCenterPosition,
				TargetCenterVelocity = targetCenterVelocity
			};

			// Used only by the bounded fallback controller. Normal TOF and phase selection
			// use an RK4-propagated coast state inside OptimizeHorizon.
			double2 ownerGravity = gravity.AccelerationAt(vehicle.Position, 0.0, ownerIndex);
			double2 targetGravity = targetIsDynamic
				? gravity.AccelerationAt(targetCenterPosition + targetOffset, 0.0, targetIndex)
				: double2.zero;
			double2 differentialGravity = targetGravity - ownerGravity;

			PlanResult best = default;
			bool foundFeasible = false;
			bool hasInfeasibleLowerBound = false;
			double lowerInfeasibleHorizon = minimumHorizon;
			double upperFeasibleHorizon = 0.0;
			// Advance the cached phase schedule by the elapsed control interval. If the old
			// accelerate phase ended during that interval, the previous braking vector becomes
			// the current warm-start vector; otherwise retain the current/future pair.
			double remainingFirstPhase = useWarmStart
				? planMemory.FirstPhaseDuration - controlStep
				: 0.0;
			double2[] warmControls = useWarmStart
				? (remainingFirstPhase > controlStep * 0.05
					? new double2[] { planMemory.CurrentControl, planMemory.FutureControl }
					: new double2[] { planMemory.FutureControl, planMemory.FutureControl })
				: null;

			if (useWarmStart)
			{
				// Start from the previous feasible arrival time, shifted by elapsed simulation
				// time. A normal warm-start solve is one optimization pass; only re-expand and
				// refine the horizon if model error makes the shifted estimate infeasible.
				double horizon = previousRemaining;
				for (int trial = 0; trial <= WarmStartExpansionTrials; trial++)
				{
					PlanResult candidate = OptimizeHorizon(
						initialState, targetIsDynamic, targetGravitySource,
						ownerIndex, targetIndex, targetOffset, horizon, controlStep, gravity,
						maxAcceleration, positionScale, velocityScale, positionTolerance, velocityTolerance,
						warmControls);
					if (candidate.Valid && (!best.Valid || candidate.Score < best.Score)) best = candidate;
					if (candidate.Feasible)
					{
						best = candidate;
						foundFeasible = true;
						upperFeasibleHorizon = horizon;
						break;
					}

					lowerInfeasibleHorizon = horizon;
					hasInfeasibleLowerBound = true;
					if (trial < WarmStartExpansionTrials)
						horizon = math.min(searchMaximumHorizon, math.max(horizon + controlStep, horizon * HorizonExpansionFactor));
					if (horizon >= searchMaximumHorizon - 1e-9 && trial < WarmStartExpansionTrials)
						break;
				}

			}
			else
			{
				// Establish a true infeasible/feasible bracket. The lower bound is checked
				// explicitly instead of being inferred from a coarse seed-time multiplier.
				PlanResult minimumCandidate = OptimizeHorizon(
					initialState, targetIsDynamic, targetGravitySource,
					ownerIndex, targetIndex, targetOffset, minimumHorizon, controlStep, gravity,
					maxAcceleration, positionScale, velocityScale, positionTolerance, velocityTolerance);
				if (minimumCandidate.Valid) best = minimumCandidate;
				if (minimumCandidate.Feasible)
				{
					best = minimumCandidate;
					foundFeasible = true;
					upperFeasibleHorizon = minimumHorizon;
				}
				else
				{
					hasInfeasibleLowerBound = true;
					double horizon = math.max(seedTime, minimumHorizon * 1.25);
					for (int trial = 0; trial < 8; trial++)
					{
						horizon = math.min(searchMaximumHorizon, horizon);
						PlanResult candidate = OptimizeHorizon(
							initialState, targetIsDynamic, targetGravitySource,
							ownerIndex, targetIndex, targetOffset, horizon, controlStep, gravity,
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
						if (horizon >= searchMaximumHorizon - 1e-9) break;
						horizon = math.min(searchMaximumHorizon, horizon * HorizonExpansionFactor);
					}
				}
			}

			// Refine a bounded minimum-time bracket. Feasibility is evaluated using the actual
			// gravity-integrated vehicle and target states, not the constant-gravity seed model.
			if (foundFeasible && hasInfeasibleLowerBound &&
				upperFeasibleHorizon > lowerInfeasibleHorizon + controlStep * 0.05)
			{
				for (int i = 0; i < HorizonRefinementIterations; i++)
				{
					double horizon = 0.5 * (lowerInfeasibleHorizon + upperFeasibleHorizon);
					PlanResult candidate = OptimizeHorizon(
						initialState, targetIsDynamic, targetGravitySource,
						ownerIndex, targetIndex, targetOffset, horizon, controlStep, gravity,
						maxAcceleration, positionScale, velocityScale, positionTolerance, velocityTolerance,
						best.Controls);
					if (candidate.Feasible)
					{
						best = candidate;
						upperFeasibleHorizon = horizon;
					}
					else lowerInfeasibleHorizon = horizon;

					double timeTolerance = math.max(controlStep * 0.1, upperFeasibleHorizon * 1e-4);
					if (upperFeasibleHorizon - lowerInfeasibleHorizon <= timeTolerance) break;
				}
			}

			if (best.Valid && IsFinite(best.FirstAcceleration))
			{
				commandedAcceleration = ClampMagnitude(best.FirstAcceleration, maxAcceleration);
				timeOfFlight = (float)math.min(best.Horizon, float.MaxValue);
				if (planMemory != null)
				{
					planMemory.CurrentControl = best.Controls != null && best.Controls.Length > 0
						? best.Controls[0]
						: best.FirstAcceleration;
					planMemory.FutureControl = best.Controls != null && best.Controls.Length > 1
						? best.Controls[1]
						: best.FirstAcceleration;
					planMemory.FirstPhaseDuration = best.Durations != null && best.Durations.Length > 0
						? best.Durations[0]
						: 0.0;
					planMemory.HasPlan = best.Feasible;
				}
			}
			else
			{
				double omega = math.sqrt(maxAcceleration / math.max(initialDistance, arrivalThreshold));
				omega = math.min(omega, 0.25 / controlStep);
				commandedAcceleration = differentialGravity
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
						MakeConstantControls(commandedAcceleration),
						MakeEqualPhaseDurations(math.max(controlStep * 4.0, timeOfFlight)),
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
			double horizon,
			double controlStep,
			GravityEphemerisView gravity,
			double maxAcceleration,
			double positionScale,
			double velocityScale,
			double positionTolerance,
			double velocityTolerance,
			double2[] warmStartControls = null)
		{
			// Propagate the no-thrust reference trajectory through the same gravity ephemeris
			// used by the powered rollout. This gives the phase scheduler a much better estimate
			// than assuming the departure gravity vector remains constant for the entire flight.
			GetCoastRelativeState(initialState, horizon, gravity, targetIsDynamic,
				targetGravitySource, ownerIndex, targetIndex, targetOffset,
				out double2 coastRelativePosition, out double2 coastRelativeVelocity);
			double[] durations = MakeControlDurations(
				horizon, controlStep, coastRelativePosition, coastRelativeVelocity);
			int count = ControlIntervals;
			double2[] controls = new double2[count];

			if (warmStartControls != null && warmStartControls.Length == count)
			{
				for (int i = 0; i < count; i++) controls[i] = ClampMagnitude(warmStartControls[i], maxAcceleration);
			}
			else
			{
				// Two-phase double-integrator seed based on the gravity-propagated coast endpoint.
				// The variational shooting solve then corrects for gravity changes caused by the
				// powered trajectory departing from that coast reference.
				if (TryGetTwoPhaseAccelerations(
					coastRelativePosition, coastRelativeVelocity,
					durations[0], durations[1],
					out double2 firstSeed, out double2 secondSeed))
				{
					controls[0] = ClampMagnitude(firstSeed, maxAcceleration);
					controls[1] = ClampMagnitude(secondSeed, maxAcceleration);
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
				for (int lineSearch = 0; lineSearch < 3; lineSearch++)
				{
					double alpha = lineSearch == 0 ? 1.0 : lineSearch == 1 ? 0.5 : 0.25;
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

		private static double2[] MakeConstantControls(double2 acceleration)
		{
			return new double2[] { acceleration, acceleration };
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

		private static void GetCoastRelativeState(
			DynamicState initialState,
			double horizon,
			GravityEphemerisView gravity,
			bool targetIsDynamic,
			int targetGravitySource,
			int ownerIndex,
			int targetIndex,
			double2 targetOffset,
			out double2 relativePosition,
			out double2 relativeVelocity)
		{
			DynamicState coastState = initialState;
			IntegrateInterval(ref coastState, 0.0, horizon, double2.zero, gravity,
				targetIsDynamic, targetGravitySource, ownerIndex, targetIndex);

			double2 targetPosition;
			double2 targetVelocity;
			if (targetGravitySource >= 0)
				gravity.SourceStateAt(targetGravitySource, horizon, out targetPosition, out targetVelocity);
			else
			{
				targetPosition = coastState.TargetCenterPosition;
				targetVelocity = coastState.TargetCenterVelocity;
			}

			relativePosition = targetPosition + targetOffset - coastState.Position;
			relativeVelocity = targetVelocity - coastState.Velocity;
		}

		/// <summary>
		/// Splits the prediction into a sustained acceleration phase and a braking phase.
		/// The split is selected from the two-phase constant-acceleration boundary solution,
		/// minimizing the larger required phase acceleration. The phase boundary is quantized
		/// to guidance-step boundaries so the receding-horizon controller can switch cleanly.
		/// </summary>
		private static double[] MakeControlDurations(
			double horizon,
			double controlStep,
			double2 coastRelativePosition,
			double2 coastRelativeVelocity)
		{
			double minimumPhase = math.min(controlStep, horizon * 0.5);
			double lower = minimumPhase;
			double upper = horizon - minimumPhase;
			if (upper <= lower + 1e-9)
				return new double[] { horizon * 0.5, horizon * 0.5 };

			// Coarse global scan avoids assuming a perfectly unimodal cost when the initial
			// relative velocity is strongly transverse to the line of sight.
			const int samples = 13;
			double sampleStep = (upper - lower) / (samples - 1);
			double bestDuration = horizon * 0.5;
			double bestCost = double.PositiveInfinity;
			for (int i = 0; i < samples; i++)
			{
				double candidateDuration = lower + sampleStep * i;
				double cost = TwoPhaseAccelerationCost(
					coastRelativePosition, coastRelativeVelocity,
					horizon, candidateDuration);
				if (cost < bestCost)
				{
					bestCost = cost;
					bestDuration = candidateDuration;
				}
			}

			double refineLower = math.max(lower, bestDuration - sampleStep);
			double refineUpper = math.min(upper, bestDuration + sampleStep);
			const double goldenRatio = 0.6180339887498948482;
			double x1 = refineUpper - goldenRatio * (refineUpper - refineLower);
			double x2 = refineLower + goldenRatio * (refineUpper - refineLower);
			double f1 = TwoPhaseAccelerationCost(coastRelativePosition, coastRelativeVelocity,
				horizon, x1);
			double f2 = TwoPhaseAccelerationCost(coastRelativePosition, coastRelativeVelocity,
				horizon, x2);
			for (int i = 0; i < 18; i++)
			{
				if (f1 <= f2)
				{
					refineUpper = x2;
					x2 = x1;
					f2 = f1;
					x1 = refineUpper - goldenRatio * (refineUpper - refineLower);
					f1 = TwoPhaseAccelerationCost(coastRelativePosition, coastRelativeVelocity,
						horizon, x1);
				}
				else
				{
					refineLower = x1;
					x1 = x2;
					f1 = f2;
					x2 = refineLower + goldenRatio * (refineUpper - refineLower);
					f2 = TwoPhaseAccelerationCost(coastRelativePosition, coastRelativeVelocity,
						horizon, x2);
				}
			}

			double refinedDuration = f1 <= f2 ? x1 : x2;
			refinedDuration = math.clamp(refinedDuration, lower, upper);

			// Phase transitions must happen on a guidance update. Otherwise a continuous-time
			// switch (for example 1.4 control steps after launch) cannot be represented by the
			// controller, which holds one acceleration for the whole next physics interval.
			int maximumWholeSteps = math.max(1, (int)math.floor((upper + 1e-9) / controlStep));
			int lowerSteps = math.clamp((int)math.floor(refinedDuration / controlStep), 1, maximumWholeSteps);
			int upperSteps = math.min(maximumWholeSteps, lowerSteps + 1);
			double lowerDuration = lowerSteps * controlStep;
			double upperDuration = upperSteps * controlStep;
			bestDuration = TwoPhaseAccelerationCost(coastRelativePosition, coastRelativeVelocity,
				horizon, lowerDuration) <= TwoPhaseAccelerationCost(
				coastRelativePosition, coastRelativeVelocity, horizon, upperDuration)
				? lowerDuration
				: upperDuration;
			return new double[] { bestDuration, horizon - bestDuration };
		}

		// Equal-duration split used only for fallback path visualization, where no solved
		// two-phase timing is available.
		private static double[] MakeEqualPhaseDurations(double horizon)
		{
			double half = math.max(1e-6, horizon * 0.5);
			return new double[] { half, math.max(1e-6, horizon - half) };
		}

		private static double TwoPhaseAccelerationCost(
			double2 coastRelativePosition,
			double2 coastRelativeVelocity,
			double horizon,
			double firstDuration)
		{
			double secondDuration = horizon - firstDuration;
			if (firstDuration <= 1e-9 || secondDuration <= 1e-9)
				return double.PositiveInfinity;

			if (!TryGetTwoPhaseAccelerations(coastRelativePosition, coastRelativeVelocity,
				firstDuration, secondDuration, out double2 first, out double2 second))
				return double.PositiveInfinity;
			double firstMagnitude = math.length(first);
			double secondMagnitude = math.length(second);
			if (!IsFinite(firstMagnitude) || !IsFinite(secondMagnitude))
				return double.PositiveInfinity;
			return math.max(firstMagnitude, secondMagnitude);
		}

		private static bool TryGetTwoPhaseAccelerations(
			double2 relativePosition,
			double2 relativeVelocity,
			double firstDuration,
			double secondDuration,
			out double2 firstAcceleration,
			out double2 secondAcceleration)
		{
			firstAcceleration = double2.zero;
			secondAcceleration = double2.zero;
			// relativePosition/Velocity are the coast-propagated target-minus-vehicle residuals.
			// The two vehicle-control phases must supply exactly that terminal displacement
			// and velocity change to bring the relative state to zero.
			double2 requiredVelocityChange = relativeVelocity;
			double2 requiredPositionChange = relativePosition;
			double firstPositionWeight = firstDuration * (0.5 * firstDuration + secondDuration);
			double secondPositionWeight = 0.5 * secondDuration * secondDuration;
			double determinant = firstDuration * secondPositionWeight
				- secondDuration * firstPositionWeight;
			if (math.abs(determinant) <= 1e-18 || !IsFinite(determinant)) return false;

			firstAcceleration = (requiredVelocityChange * secondPositionWeight
				- requiredPositionChange * secondDuration) / determinant;
			secondAcceleration = (requiredPositionChange * firstDuration
				- requiredVelocityChange * firstPositionWeight) / determinant;
			return IsFinite(firstAcceleration) && IsFinite(secondAcceleration);
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