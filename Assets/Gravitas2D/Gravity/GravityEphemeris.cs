using System;
using Unity.Collections;
using Unity.Mathematics;

namespace Gravitas2D
{
	/// <summary>
	/// Maintains a rolling, fixed-horizon prediction of all celestial gravity sources.
	/// Samples are stored at a configurable cadence and generated with coupled N-body RK4.
	/// Queries are read-only and use cubic Hermite interpolation between stored samples.
	/// </summary>
	public sealed class GravityEphemeris
	{
		private const double MinimumDistanceSquared = 1e-12;
		private const double TimeEpsilon = 1e-9;

		private bool _isInitialized;
		private double _epoch;
		private double _sampleInterval;
		private double _integrationStep;
		private double _horizon;
		private double _gravitationalConstant;
		private int _capacity;
		private int _sourceCount;
		private long _firstSampleIndex;
		private long _lastSampleIndex;

		// Source identity and gravity parameters are fixed for the lifetime of a simulation.
		private int[] _bodyIndices = Array.Empty<int>();
		private double[] _mu = Array.Empty<double>();

		// Ring-buffered ephemeris samples. Each sample stores one position and velocity per source.
		private double2[] _positionSamples = Array.Empty<double2>();
		private double2[] _velocitySamples = Array.Empty<double2>();

		// Working state and RK4 stage storage. These arrays are reused for every appended sample.
		private double2[] _workingPositions = Array.Empty<double2>();
		private double2[] _workingVelocities = Array.Empty<double2>();
		private double2[] _stage2Positions = Array.Empty<double2>();
		private double2[] _stage2Velocities = Array.Empty<double2>();
		private double2[] _stage3Positions = Array.Empty<double2>();
		private double2[] _stage3Velocities = Array.Empty<double2>();
		private double2[] _stage4Positions = Array.Empty<double2>();
		private double2[] _stage4Velocities = Array.Empty<double2>();
		private double2[] _k1Position = Array.Empty<double2>();
		private double2[] _k1Velocity = Array.Empty<double2>();
		private double2[] _k2Position = Array.Empty<double2>();
		private double2[] _k2Velocity = Array.Empty<double2>();
		private double2[] _k3Position = Array.Empty<double2>();
		private double2[] _k3Velocity = Array.Empty<double2>();
		private double2[] _k4Position = Array.Empty<double2>();
		private double2[] _k4Velocity = Array.Empty<double2>();
		private double2[] _stageAccelerations = Array.Empty<double2>();
		private GravitySourceData[] _stageGravitySources = Array.Empty<GravitySourceData>();

		public bool IsInitialized => _isInitialized;
		public int SourceCount => _sourceCount;
		public int StoredSampleCount => _isInitialized ? (int)(_lastSampleIndex - _firstSampleIndex + 1) : 0;
		public int SegmentCount => math.max(0, StoredSampleCount - 1);
		public double Epoch => _epoch;
		public double SampleInterval => _sampleInterval;
		public double IntegrationStep => _integrationStep;
		public double Horizon => _horizon;
		public double StartTime => _isInitialized ? SampleTime(_firstSampleIndex) : 0.0;
		public double EndTime => _isInitialized ? SampleTime(_lastSampleIndex) : 0.0;

		/// <summary>
		/// Ensures the ephemeris covers [simulationTime, simulationTime + requestedHorizon].
		/// Old samples are discarded only after they are no longer needed for interpolation.
		/// The future tail is extended only when the requested horizon advances past it.
		/// </summary>
		internal void EnsureCoverage(
			double simulationTime,
			double requestedHorizon,
			double sampleInterval,
			double integrationStep,
			NativeArray<StateVector> bodyStates,
			NativeArray<VehicleModel> bodyModels,
			NativeList<int> gravitationalBodyIndices,
			double gravitationalConstant)
		{
			if (!IsFinite(simulationTime)) return;
			if (!gravitationalBodyIndices.IsCreated || gravitationalBodyIndices.Length == 0)
			{
				Clear();
				return;
			}

			sampleInterval = IsFinite(sampleInterval) ? math.max(1e-3, sampleInterval) : 60.0;
			integrationStep = IsFinite(integrationStep) ? math.max(1e-3, integrationStep) : sampleInterval;
			requestedHorizon = IsFinite(requestedHorizon) ? math.max(sampleInterval, requestedHorizon) : sampleInterval;
			integrationStep = math.min(integrationStep, sampleInterval);

			bool sourceSetChanged = !SourcesMatch(bodyModels, gravitationalBodyIndices, gravitationalConstant);
			bool settingsChanged = !_isInitialized ||
				math.abs(_sampleInterval - sampleInterval) > TimeEpsilon ||
				math.abs(_integrationStep - integrationStep) > TimeEpsilon ||
				requestedHorizon > _horizon + TimeEpsilon;
			bool timeDiscontinuity = _isInitialized &&
				(simulationTime < _epoch - TimeEpsilon || simulationTime > EndTime + TimeEpsilon);

			if (settingsChanged || sourceSetChanged || timeDiscontinuity)
			{
				InitializeFromCurrentState(
					simulationTime, requestedHorizon, sampleInterval, integrationStep,
					bodyStates, bodyModels, gravitationalBodyIndices, gravitationalConstant);
			}
			else
			{
				PruneOldSamples(simulationTime);
			}

			if (!_isInitialized || _sourceCount == 0) return;

			double requiredEndTime = simulationTime + _horizon;
			long requiredSampleIndex = (long)math.ceil((requiredEndTime - _epoch) / _sampleInterval - 1e-12);
			while (_lastSampleIndex < requiredSampleIndex)
				AppendNextSample();
		}

		/// <summary>Creates a lightweight time-relative view for one navigation solve.</summary>
		internal GravityEphemerisView CreateView(double referenceTime)
		{
			return _isInitialized && _sourceCount > 0
				? new GravityEphemerisView(this, referenceTime)
				: default;
		}

		/// <summary>Clears all stored samples and releases managed buffers for a new simulation.</summary>
		public void Clear()
		{
			_isInitialized = false;
			_epoch = 0.0;
			_sampleInterval = 0.0;
			_integrationStep = 0.0;
			_horizon = 0.0;
			_gravitationalConstant = 0.0;
			_capacity = 0;
			_sourceCount = 0;
			_firstSampleIndex = 0;
			_lastSampleIndex = 0;

			_bodyIndices = Array.Empty<int>();
			_mu = Array.Empty<double>();
			_positionSamples = Array.Empty<double2>();
			_velocitySamples = Array.Empty<double2>();
			_workingPositions = Array.Empty<double2>();
			_workingVelocities = Array.Empty<double2>();
			_stage2Positions = Array.Empty<double2>();
			_stage2Velocities = Array.Empty<double2>();
			_stage3Positions = Array.Empty<double2>();
			_stage3Velocities = Array.Empty<double2>();
			_stage4Positions = Array.Empty<double2>();
			_stage4Velocities = Array.Empty<double2>();
			_k1Position = Array.Empty<double2>();
			_k1Velocity = Array.Empty<double2>();
			_k2Position = Array.Empty<double2>();
			_k2Velocity = Array.Empty<double2>();
			_k3Position = Array.Empty<double2>();
			_k3Velocity = Array.Empty<double2>();
			_k4Position = Array.Empty<double2>();
			_k4Velocity = Array.Empty<double2>();
			_stageAccelerations = Array.Empty<double2>();
			_stageGravitySources = Array.Empty<GravitySourceData>();
		}

		private bool SourcesMatch(
			NativeArray<VehicleModel> bodyModels,
			NativeList<int> gravitationalBodyIndices,
			double gravitationalConstant)
		{
			if (!_isInitialized || !gravitationalBodyIndices.IsCreated ||
				gravitationalBodyIndices.Length != _sourceCount ||
				math.abs(_gravitationalConstant - gravitationalConstant) > 1e-18)
				return false;

			for (int i = 0; i < _sourceCount; i++)
			{
				int bodyIndex = gravitationalBodyIndices[i];
				if (bodyIndex < 0 || bodyIndex >= bodyModels.Length || bodyIndex != _bodyIndices[i])
					return false;
				double mu = gravitationalConstant * bodyModels[bodyIndex].Mass;
				if (math.abs(mu - _mu[i]) > math.max(1e-12, math.abs(_mu[i]) * 1e-12))
					return false;
			}
			return true;
		}

		private void InitializeFromCurrentState(
			double epoch,
			double horizon,
			double sampleInterval,
			double integrationStep,
			NativeArray<StateVector> bodyStates,
			NativeArray<VehicleModel> bodyModels,
			NativeList<int> gravitationalBodyIndices,
			double gravitationalConstant)
		{
			Clear();
			_epoch = epoch;
			_horizon = horizon;
			_sampleInterval = sampleInterval;
			_integrationStep = integrationStep;
			_gravitationalConstant = gravitationalConstant;

			if (!bodyStates.IsCreated || !bodyModels.IsCreated || !gravitationalBodyIndices.IsCreated ||
				gravitationalConstant <= 0.0)
			{
				_isInitialized = true;
				return;
			}

			_sourceCount = gravitationalBodyIndices.Length;
			_bodyIndices = new int[_sourceCount];
			_mu = new double[_sourceCount];
			_workingPositions = new double2[_sourceCount];
			_workingVelocities = new double2[_sourceCount];

			for (int i = 0; i < _sourceCount; i++)
			{
				int bodyIndex = gravitationalBodyIndices[i];
				if (bodyIndex < 0 || bodyIndex >= bodyStates.Length || bodyIndex >= bodyModels.Length)
				{
					Clear();
					_isInitialized = true;
					return;
				}

				_bodyIndices[i] = bodyIndex;
				_mu[i] = gravitationalConstant * bodyModels[bodyIndex].Mass;
				_workingPositions[i] = bodyStates[bodyIndex].Position;
				_workingVelocities[i] = bodyStates[bodyIndex].Velocity;
			}

			_capacity = CalculateCapacity(horizon, sampleInterval, _sourceCount);
			int totalSamples = checked(_capacity * _sourceCount);
			_positionSamples = new double2[totalSamples];
			_velocitySamples = new double2[totalSamples];
			AllocateIntegrationBuffers(_sourceCount);

			_firstSampleIndex = 0;
			_lastSampleIndex = 0;
			StoreSample(0);
			_isInitialized = true;
		}

		private static int CalculateCapacity(double horizon, double sampleInterval, int sourceCount)
		{
			double samples = math.ceil(horizon / sampleInterval) + 3.0;
			if (samples > int.MaxValue) throw new InvalidOperationException("Gravity ephemeris horizon requires too many samples.");
			int capacity = math.max(4, (int)samples);
			if (sourceCount > 0 && capacity > int.MaxValue / sourceCount)
				throw new InvalidOperationException("Gravity ephemeris buffer exceeds the maximum supported array size.");
			return capacity;
		}

		private void AllocateIntegrationBuffers(int count)
		{
			_stage2Positions = new double2[count];
			_stage2Velocities = new double2[count];
			_stage3Positions = new double2[count];
			_stage3Velocities = new double2[count];
			_stage4Positions = new double2[count];
			_stage4Velocities = new double2[count];
			_k1Position = new double2[count];
			_k1Velocity = new double2[count];
			_k2Position = new double2[count];
			_k2Velocity = new double2[count];
			_k3Position = new double2[count];
			_k3Velocity = new double2[count];
			_k4Position = new double2[count];
			_k4Velocity = new double2[count];
			_stageAccelerations = new double2[count];
			_stageGravitySources = new GravitySourceData[count];
		}

		private void PruneOldSamples(double simulationTime)
		{
			if (_sourceCount == 0 || _sampleInterval <= 0.0) return;
			double coordinate = (simulationTime - _epoch) / _sampleInterval;
			long currentLowerSample = (long)Math.Floor(coordinate);
			long keepFrom = Math.Max(_firstSampleIndex, currentLowerSample - 1);
			if (keepFrom > _firstSampleIndex) _firstSampleIndex = keepFrom;
		}

		private void AppendNextSample()
		{
			int steps = math.max(1, (int)math.ceil(_sampleInterval / _integrationStep));
			double dt = _sampleInterval / steps;
			for (int step = 0; step < steps; step++) IntegrateRK4Step(dt);

			_lastSampleIndex++;
			StoreSample(_lastSampleIndex);
		}

		private void IntegrateRK4Step(double dt)
		{
			EvaluateAccelerations(_workingPositions, _stageAccelerations);
			for (int i = 0; i < _sourceCount; i++)
			{
				_k1Position[i] = _workingVelocities[i];
				_k1Velocity[i] = _stageAccelerations[i];
				_stage2Positions[i] = _workingPositions[i] + _k1Position[i] * (0.5 * dt);
				_stage2Velocities[i] = _workingVelocities[i] + _k1Velocity[i] * (0.5 * dt);
			}

			EvaluateAccelerations(_stage2Positions, _stageAccelerations);
			for (int i = 0; i < _sourceCount; i++)
			{
				_k2Position[i] = _stage2Velocities[i];
				_k2Velocity[i] = _stageAccelerations[i];
				_stage3Positions[i] = _workingPositions[i] + _k2Position[i] * (0.5 * dt);
				_stage3Velocities[i] = _workingVelocities[i] + _k2Velocity[i] * (0.5 * dt);
			}

			EvaluateAccelerations(_stage3Positions, _stageAccelerations);
			for (int i = 0; i < _sourceCount; i++)
			{
				_k3Position[i] = _stage3Velocities[i];
				_k3Velocity[i] = _stageAccelerations[i];
				_stage4Positions[i] = _workingPositions[i] + _k3Position[i] * dt;
				_stage4Velocities[i] = _workingVelocities[i] + _k3Velocity[i] * dt;
			}

			EvaluateAccelerations(_stage4Positions, _stageAccelerations);
			for (int i = 0; i < _sourceCount; i++)
			{
				_k4Position[i] = _stage4Velocities[i];
				_k4Velocity[i] = _stageAccelerations[i];
			}

			double scale = dt / 6.0;
			for (int i = 0; i < _sourceCount; i++)
			{
				_workingPositions[i] += scale * (_k1Position[i] + 2.0 * _k2Position[i] + 2.0 * _k3Position[i] + _k4Position[i]);
				_workingVelocities[i] += scale * (_k1Velocity[i] + 2.0 * _k2Velocity[i] + 2.0 * _k3Velocity[i] + _k4Velocity[i]);
			}
		}

		private void EvaluateAccelerations(double2[] positions, double2[] output)
		{
			for (int i = 0; i < _sourceCount; i++)
			{
				_stageGravitySources[i] = new GravitySourceData
				{
					Position = positions[i],
					Mu = _mu[i]
				};
			}

			for (int i = 0; i < _sourceCount; i++)
			{
				GravitationalAcceleration.CalculateGravitationalAcceleration(
					positions[i], _bodyIndices, _stageGravitySources, _bodyIndices[i], out output[i]);
			}
		}

		private void StoreSample(long sampleIndex)
		{
			int slot = GetSlot(sampleIndex);
			int offset = slot * _sourceCount;
			Array.Copy(_workingPositions, 0, _positionSamples, offset, _sourceCount);
			Array.Copy(_workingVelocities, 0, _velocitySamples, offset, _sourceCount);
		}

		private int GetSlot(long sampleIndex)
		{
			long slot = sampleIndex % _capacity;
			if (slot < 0) slot += _capacity;
			return (int)slot;
		}

		private double SampleTime(long sampleIndex) => _epoch + sampleIndex * _sampleInterval;

		private bool TryGetSegment(double absoluteTime, out long lowerSampleIndex, out double u)
		{
			lowerSampleIndex = 0;
			u = 0.0;
			if (!_isInitialized || _sourceCount == 0 || _lastSampleIndex <= _firstSampleIndex || !IsFinite(absoluteTime))
				return false;

			double coordinate = (absoluteTime - _epoch) / _sampleInterval;
			if (coordinate < _firstSampleIndex - 1e-10 || coordinate > _lastSampleIndex + 1e-10)
				return false;

			if (coordinate <= _firstSampleIndex)
			{
				lowerSampleIndex = _firstSampleIndex;
				u = 0.0;
			}
			else if (coordinate >= _lastSampleIndex)
			{
				lowerSampleIndex = _lastSampleIndex - 1;
				u = 1.0;
			}
			else
			{
				lowerSampleIndex = (long)Math.Floor(coordinate);
				u = math.clamp(coordinate - lowerSampleIndex, 0.0, 1.0);
			}
			return true;
		}

		private double2 InterpolatePosition(int sourceIndex, long lowerSampleIndex, double u)
		{
			int slot0 = GetSlot(lowerSampleIndex);
			int slot1 = GetSlot(lowerSampleIndex + 1);
			int i0 = slot0 * _sourceCount + sourceIndex;
			int i1 = slot1 * _sourceCount + sourceIndex;
			double u2 = u * u;
			double u3 = u2 * u;
			double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
			double h10 = u3 - 2.0 * u2 + u;
			double h01 = -2.0 * u3 + 3.0 * u2;
			double h11 = u3 - u2;
			return h00 * _positionSamples[i0] + (h10 * _sampleInterval) * _velocitySamples[i0]
				+ h01 * _positionSamples[i1] + (h11 * _sampleInterval) * _velocitySamples[i1];
		}

		private double2 InterpolateVelocity(int sourceIndex, long lowerSampleIndex, double u)
		{
			int slot0 = GetSlot(lowerSampleIndex);
			int slot1 = GetSlot(lowerSampleIndex + 1);
			int i0 = slot0 * _sourceCount + sourceIndex;
			int i1 = slot1 * _sourceCount + sourceIndex;
			double u2 = u * u;
			double dh00 = (6.0 * u2 - 6.0 * u) / _sampleInterval;
			double dh10 = 3.0 * u2 - 4.0 * u + 1.0;
			double dh01 = (-6.0 * u2 + 6.0 * u) / _sampleInterval;
			double dh11 = 3.0 * u2 - 2.0 * u;
			return dh00 * _positionSamples[i0] + dh10 * _velocitySamples[i0]
				+ dh01 * _positionSamples[i1] + dh11 * _velocitySamples[i1];
		}

		internal int FindSourceIndex(int physicsBodyIndex)
		{
			for (int i = 0; i < _sourceCount; i++)
				if (_bodyIndices[i] == physicsBodyIndex) return i;
			return -1;
		}

		internal int GetPhysicsBodyIndex(int sourceIndex)
		{
			return sourceIndex >= 0 && sourceIndex < _sourceCount ? _bodyIndices[sourceIndex] : -1;
		}

		internal double CoverageFrom(double referenceTime)
		{
			return _isInitialized ? math.max(0.0, EndTime - referenceTime) : 0.0;
		}

		internal bool TryGetSourceState(int sourceIndex, double absoluteTime, out double2 position, out double2 velocity)
		{
			position = double2.zero;
			velocity = double2.zero;
			if (sourceIndex < 0 || sourceIndex >= _sourceCount ||
				!TryGetSegment(absoluteTime, out long segment, out double u)) return false;

			position = InterpolatePosition(sourceIndex, segment, u);
			velocity = InterpolateVelocity(sourceIndex, segment, u);
			return true;
		}

		internal bool TryGetAcceleration(double2 position, double absoluteTime, int excludedBodyIndex, out double2 acceleration)
		{
			acceleration = double2.zero;
			if (!TryGetSegment(absoluteTime, out long segment, out double u)) return false;

			for (int i = 0; i < _sourceCount; i++)
			{
				if (_bodyIndices[i] == excludedBodyIndex) continue;
				double2 delta = InterpolatePosition(i, segment, u) - position;
				double distanceSquared = math.lengthsq(delta);
				if (distanceSquared <= MinimumDistanceSquared) continue;
				double inverseDistanceCubed = 1.0 / (distanceSquared * math.sqrt(distanceSquared));
				acceleration += delta * (_mu[i] * inverseDistanceCubed);
			}
			return true;
		}

		internal bool TryGetAccelerationAndGradient(
			double2 position, double absoluteTime, int excludedBodyIndex,
			out double2 acceleration, out double gxx, out double gxy, out double gyy)
		{
			acceleration = double2.zero;
			gxx = gxy = gyy = 0.0;
			if (!TryGetSegment(absoluteTime, out long segment, out double u)) return false;

			for (int i = 0; i < _sourceCount; i++)
			{
				if (_bodyIndices[i] == excludedBodyIndex) continue;
				double2 delta = InterpolatePosition(i, segment, u) - position;
				double r2 = math.lengthsq(delta);
				if (r2 <= MinimumDistanceSquared) continue;
				double invR = math.rsqrt(r2);
				double invR3 = invR / r2;
				double invR5Scale3 = 3.0 * _mu[i] * invR3 / r2;
				double scaleInvR3 = _mu[i] * invR3;
				acceleration += delta * scaleInvR3;
				gxx += invR5Scale3 * delta.x * delta.x - scaleInvR3;
				gxy += invR5Scale3 * delta.x * delta.y;
				gyy += invR5Scale3 * delta.y * delta.y - scaleInvR3;
			}
			return true;
		}

		private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);
	}

	/// <summary>
	/// Read-only view of a shared gravity ephemeris, with all query times expressed as offsets
	/// from the simulation epoch captured when the view was created.
	/// </summary>
	public readonly struct GravityEphemerisView
	{
		private readonly GravityEphemeris _ephemeris;
		private readonly double _referenceTime;

		internal GravityEphemerisView(GravityEphemeris ephemeris, double referenceTime)
		{
			_ephemeris = ephemeris;
			_referenceTime = referenceTime;
		}

		public bool IsValid => _ephemeris != null && _ephemeris.IsInitialized && _ephemeris.SourceCount > 0;
		public int SegmentCount => IsValid ? _ephemeris.SegmentCount : 0;
		public double Coverage => IsValid ? _ephemeris.CoverageFrom(_referenceTime) : 0.0;
		public int FindSource(int physicsBodyIndex) => IsValid ? _ephemeris.FindSourceIndex(physicsBodyIndex) : -1;
		public int GetPhysicsBodyIndex(int sourceIndex) => IsValid ? _ephemeris.GetPhysicsBodyIndex(sourceIndex) : -1;

		public double2 AccelerationAt(double2 position, double timeOffset, int excludedBodyIndex)
		{
			return IsValid && _ephemeris.TryGetAcceleration(position, _referenceTime + timeOffset,
				excludedBodyIndex, out double2 acceleration) ? acceleration : double2.zero;
		}

		public void AccelerationAndGradientAt(
			double2 position, double timeOffset, int excludedBodyIndex,
			out double2 acceleration, out double gxx, out double gxy, out double gyy)
		{
			if (IsValid && _ephemeris.TryGetAccelerationAndGradient(position, _referenceTime + timeOffset,
				excludedBodyIndex, out acceleration, out gxx, out gxy, out gyy)) return;
			acceleration = double2.zero;
			gxx = gxy = gyy = 0.0;
		}

		public void SourceStateAt(int sourceIndex, double timeOffset, out double2 position, out double2 velocity)
		{
			if (IsValid && _ephemeris.TryGetSourceState(sourceIndex, _referenceTime + timeOffset,
				out position, out velocity)) return;
			position = double2.zero;
			velocity = double2.zero;
		}
	}
}