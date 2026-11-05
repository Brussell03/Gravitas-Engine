using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Gravitas2D;
using System;
using System.Collections.Generic;
using System.Linq;
using GravitasDemo.Rendering;
using GravitasDemo.Interaction;

namespace GravitasDemo
{
	public class EntitiesManager : MonoBehaviour
	{
		public static EntitiesManager Instance { get; private set; }

		private bool _isInitialized;

		// Collections
		private readonly List<Entity> _allEntities = new List<Entity>();
		private readonly HashSet<Entity> _knownEntities = new HashSet<Entity>();
		private readonly HashSet<Entity> _observedEntities = new HashSet<Entity>();
		private readonly HashSet<Entity> _visibleEntities = new HashSet<Entity>();
		private readonly HashSet<Entity> _navigableEntities = new HashSet<Entity>();
		private readonly HashSet<Entity> _sensorEntities = new HashSet<Entity>();
		private readonly HashSet<Entity> _autonomousEntitiesSet = new HashSet<Entity>();
		private readonly List<Entity> _autonomousEntitiesList = new List<Entity>();
		private Dictionary<string, Entity> _entityLookup = new Dictionary<string, Entity>();
		private Dictionary<PhysicsBody, Entity> _entityBodyLookup = new Dictionary<PhysicsBody, Entity>();

		// Visual Mappings
		private readonly Dictionary<Entity, EntityMapObject> _activeVisuals = new Dictionary<Entity, EntityMapObject>();
		private readonly Dictionary<Entity, CelestialMapObject> _activeCelestialVisuals = new Dictionary<Entity, CelestialMapObject>();
		private readonly List<Entity> _entitiesToRemove = new List<Entity>();

		// Job System Variables & Time-Slicing
		private JobHandle _filterJobHandle;
		private bool _isJobRunning = false;
		// private int _scheduledEntityCount = 0;
		private int _currentFrameSlice = 0;
		private const int TimeSliceCount = 1; // Distributes sensor checks across 4 frames
		
		// Synchronized Native Arrays (Packed & Contiguous)
		private NativeArray<double2> _jobPositions;
		private NativeArray<double> _jobRadii;
		private NativeArray<double> _jobMasses;
		private NativeArray<bool> _jobIsOwned;
		private NativeArray<bool> _jobIsKnown;
		private NativeArray<bool> _jobIsObservedResults;
		private NativeArray<bool> _jobIsVisibleResults;
		
		// Flattened Sensor Native Arrays
		private struct SensorEntry
		{
			public Entity Owner;
			public int NativeIndex;
		}
		private readonly List<SensorEntry> _sensorEntries = new List<SensorEntry>();
		private NativeArray<double2> _jobSensorPositions;
		private NativeArray<float> _jobSensorHeadings;
		private NativeArray<double> _jobSensorRanges;
		private NativeArray<float> _jobSensorFOVs;
		private NativeArray<int> _jobSensorTypes;
		private NativeArray<float> _jobSensorSensitivities;
		
		private Entity[] _processingEntities = new Entity[0];

		public IReadOnlyCollection<Entity> KnownEntities => _knownEntities;
		public IReadOnlyCollection<Entity> ObservedEntities => _observedEntities;
		public IReadOnlyCollection<Entity> VisibleEntities => _visibleEntities;
		public IReadOnlyCollection<Entity> NavigableEntities => _navigableEntities;
		public IReadOnlyCollection<Entity> SensorEntities => _sensorEntities;
		public IReadOnlyCollection<Entity> AutonomousEntities => _autonomousEntitiesSet;
		public Dictionary<string, Entity> EntityLookup => _entityLookup;
		public Dictionary<PhysicsBody, Entity> EntityBodyLookup => _entityBodyLookup;
		public bool IsInitialized => _isInitialized;

		private void Start()
		{
			if (Instance != null && Instance != this)
			{
				Destroy(gameObject);
				return;
			}
			Instance = this;

			Debug.Log($"[{GetType().Name}] Initialized");

			_isInitialized = true;

			// PhysicsManager2D.Instance.OnPhysicsUpdated += UpdateSensors;
			PhysicsManager2D.Instance.OnPhysicsUpdated += UpdateAutonomies;
			PhysicsManager2D.Instance.OnSimulationOriginatedBody += EntityCreation.CreateWithExistingBody;
		}

		private void OnDestroy()
		{
			if (_isJobRunning) _filterJobHandle.Complete();
			DisposeNativeArrays();

			if (Instance == this) Instance = null;

			if (PhysicsManager2D.Instance)
			{
				// PhysicsManager2D.Instance.OnPhysicsUpdated -= UpdateSensors;
				PhysicsManager2D.Instance.OnPhysicsUpdated -= UpdateAutonomies;
				PhysicsManager2D.Instance.OnSimulationOriginatedBody -= EntityCreation.CreateWithExistingBody;
			}
		}

		public bool RegisterEntity(Entity entity)
		{
			if (entity == null || _entityLookup.ContainsKey(entity.Name)) return false;

			if (_isJobRunning) _filterJobHandle.Complete();

			int newIndex = _allEntities.Count;
			EnsureNativeArrayCapacity(newIndex + 1);

			_allEntities.Add(entity);
			_processingEntities[newIndex] = entity;
			_entityLookup.Add(entity.Name, entity);
			_entityBodyLookup.Add(entity.PhysicsBody, entity);

			// Populate static native properties once upon registration
			_jobPositions[newIndex] = entity.Position;
			_jobRadii[newIndex] = entity.Radius;
			_jobMasses[newIndex] = entity.Mass;
			_jobIsOwned[newIndex] = entity.IsOwned;
			_jobIsKnown[newIndex] = entity.IsKnown;
			_jobIsObservedResults[newIndex] = entity.IsObserved;
			_jobIsVisibleResults[newIndex] = false;

			if (entity.IsOwned || entity.IsKnown)
			{
				_knownEntities.Add(entity);
			}

			if (entity.HasCapability<NavigationCapability>())
			{
				_navigableEntities.Add(entity);
			}
			if (entity.HasCapability<SensorCapability>())
			{
				_sensorEntities.Add(entity);
				RegisterEntitySensors(entity);
			}

			return true;
		}

		public void UnregisterEntity(Entity entity)
		{
			if (entity == null || !_entityLookup.ContainsKey(entity.Name)) return;

			if (_isJobRunning) _filterJobHandle.Complete();

			int index = _allEntities.IndexOf(entity);
			if (index < 0) return;

			// Unregister associated sensors first
			UnregisterEntitySensors(entity);

			int lastIndex = _allEntities.Count - 1;
			if (index < lastIndex)
			{
				// Swap-and-pop to maintain dense, contiguous native arrays without gaps
				Entity lastEntity = _allEntities[lastIndex];
				_allEntities[index] = lastEntity;
				_processingEntities[index] = lastEntity;

				_jobPositions[index] = _jobPositions[lastIndex];
				_jobRadii[index] = _jobRadii[lastIndex];
				_jobMasses[index] = _jobMasses[lastIndex];
				_jobIsOwned[index] = _jobIsOwned[lastIndex];
				_jobIsKnown[index] = _jobIsKnown[lastIndex];
				_jobIsObservedResults[index] = _jobIsObservedResults[lastIndex];
				_jobIsVisibleResults[index] = _jobIsVisibleResults[lastIndex];
			}

			_allEntities.RemoveAt(lastIndex);
			_processingEntities[lastIndex] = null;

			_knownEntities.Remove(entity);
			_observedEntities.Remove(entity);
			_visibleEntities.Remove(entity);
			_navigableEntities.Remove(entity);
			_sensorEntities.Remove(entity);
			_autonomousEntitiesSet.Remove(entity);
			_autonomousEntitiesList.Remove(entity);
			_entityLookup.Remove(entity.Name);
			_entityBodyLookup.Remove(entity.PhysicsBody);
			
			if (_activeVisuals.TryGetValue(entity, out var normalVisual))
			{
				normalVisual.Deinitialize();
				_activeVisuals.Remove(entity);
			}

			if (_activeCelestialVisuals.TryGetValue(entity, out var celestialVisual))
			{
				celestialVisual.Deactivate();
				_activeCelestialVisuals.Remove(entity);
			}
		}

		private void RegisterEntitySensors(Entity entity)
		{
			foreach (var sensor in entity.GetCapabilities<SensorCapability>())
			{
				int sensorIndex = _sensorEntries.Count;
				EnsureSensorNativeArrayCapacity(sensorIndex + 1);

				_sensorEntries.Add(new SensorEntry { Owner = entity, NativeIndex = sensorIndex });
				UpdateSensorDataInNative(sensorIndex, entity, sensor);
			}
		}

		private void UnregisterEntitySensors(Entity entity)
		{
			for (int i = _sensorEntries.Count - 1; i >= 0; i--)
			{
				if (_sensorEntries[i].Owner == entity)
				{
					int lastIndex = _sensorEntries.Count - 1;
					if (i < lastIndex)
					{
						// Swap-and-pop for sensor native arrays
						var lastEntry = _sensorEntries[lastIndex];
						_sensorEntries[i] = lastEntry;

						_jobSensorPositions[i] = _jobSensorPositions[lastIndex];
						_jobSensorHeadings[i] = _jobSensorHeadings[lastIndex];
						_jobSensorRanges[i] = _jobSensorRanges[lastIndex];
						_jobSensorFOVs[i] = _jobSensorFOVs[lastIndex];
						_jobSensorTypes[i] = _jobSensorTypes[lastIndex];
						_jobSensorSensitivities[i] = _jobSensorSensitivities[lastIndex];
					}
					_sensorEntries.RemoveAt(lastIndex);
				}
			}
		}

		private void UpdateSensorDataInNative(int nativeIndex, Entity entity, SensorCapability sensor)
		{
			_jobSensorPositions[nativeIndex] = entity.Position;
			_jobSensorHeadings[nativeIndex] = entity.Orientation + sensor.OrientationOffset;
			_jobSensorRanges[nativeIndex] = sensor.Range;
			_jobSensorFOVs[nativeIndex] = sensor.FieldOfViewDegrees;
			_jobSensorTypes[nativeIndex] = (int)sensor.Type;
			_jobSensorSensitivities[nativeIndex] = sensor.Sensitivity;
		}

		public void NotifyCapabilityAdded(Entity entity, IEntityCapability capability)
		{
			if (capability == null) return;

			if (capability is NavigationCapability)
			{
				_navigableEntities.Add(entity);
			}
			else if (capability is SensorCapability sensor)
			{
				_sensorEntities.Add(entity);
				RegisterEntitySensors(entity);
			}
		}

		public void NotifyAutonomous(Entity entity, bool isAutonomyRunning)
		{
			if (entity == null) return;

			if (isAutonomyRunning && _autonomousEntitiesSet.Add(entity))
			{
				_autonomousEntitiesList.Add(entity);
			}
			else if (_autonomousEntitiesSet.Remove(entity))
			{
				_autonomousEntitiesList.Remove(entity);
			}
		}

		public EntityMapObject GetVisualForEntity(Entity entity)
		{
			// Normal entities only. Celestial entities use GetCelestialVisualForEntity.
			return entity != null && !entity.IsCelestial &&
				_activeVisuals.TryGetValue(entity, out var visual) ? visual : null;
		}

		public CelestialMapObject GetCelestialVisualForEntity(Entity entity)
		{
			return entity != null && entity.IsCelestial &&
				_activeCelestialVisuals.TryGetValue(entity, out var visual) ? visual : null;
		}

		public MapObject GetMapObjectForEntity(Entity entity)
		{
			if (entity == null)
				return null;

			return entity.IsCelestial
				? (MapObject)GetCelestialVisualForEntity(entity)
				: GetVisualForEntity(entity);
		}

		public void ClearEntities()
		{
			if (_isJobRunning) _filterJobHandle.Complete();
			_isJobRunning = false;
			
			for (int i = _allEntities.Count - 1; i >= 0; i--)
			{
				_allEntities[i].RemoveManual();
			}

			foreach (var visual in _activeVisuals.Values)
				visual.Deinitialize();
			_activeVisuals.Clear();

			foreach (var visual in _activeCelestialVisuals.Values)
				visual.Deactivate();
			_activeCelestialVisuals.Clear();

			_allEntities.Clear();
			_knownEntities.Clear();
			_observedEntities.Clear();
			_visibleEntities.Clear();
			_navigableEntities.Clear();
			_autonomousEntitiesSet.Clear();
			_autonomousEntitiesList.Clear();
			_sensorEntities.Clear();
			_entityLookup.Clear();
			_entityBodyLookup.Clear();
			_sensorEntries.Clear();

			// DisposeNativeArrays();
		}

		private void DisposeNativeArrays()
		{
			if (_jobPositions.IsCreated) _jobPositions.Dispose();
			if (_jobRadii.IsCreated) _jobRadii.Dispose();
			if (_jobMasses.IsCreated) _jobMasses.Dispose();
			if (_jobIsOwned.IsCreated) _jobIsOwned.Dispose();
			if (_jobIsKnown.IsCreated) _jobIsKnown.Dispose();
			if (_jobSensorPositions.IsCreated) _jobSensorPositions.Dispose();
			if (_jobSensorHeadings.IsCreated) _jobSensorHeadings.Dispose();
			if (_jobSensorRanges.IsCreated) _jobSensorRanges.Dispose();
			if (_jobSensorFOVs.IsCreated) _jobSensorFOVs.Dispose();
			if (_jobSensorTypes.IsCreated) _jobSensorTypes.Dispose();
			if (_jobSensorSensitivities.IsCreated) _jobSensorSensitivities.Dispose();
			if (_jobIsObservedResults.IsCreated) _jobIsObservedResults.Dispose();
			if (_jobIsVisibleResults.IsCreated) _jobIsVisibleResults.Dispose();
		}

		private void UpdateAutonomies()
		{
			if (PhysicsManager2D.Instance == null || !PhysicsManager2D.Instance.IsInitialized || _autonomousEntitiesList.Count == 0) return;
			float dt = PhysicsManager2D.Instance.ScaledPhysicsTimeStep;

			for (int i = _autonomousEntitiesList.Count - 1; i >= 0; i--)
			{
				var entity = _autonomousEntitiesList[i];
				if (entity == null || entity.IsDestroyed) continue;
				entity.EvaluateAutonomy(dt);
			}
		}

		private void UpdateSensors()
		{
			if (PhysicsManager2D.Instance == null || !PhysicsManager2D.Instance.IsInitialized || _sensorEntities.Count == 0) return;

			foreach (var entity in _sensorEntities)
			{
				if (entity == null || entity.IsDestroyed) continue;
				entity.UpdateObservedEntities();
			}
		}

		void LateUpdate()
		{
			if (PhysicsManager2D.Instance == null || !PhysicsManager2D.Instance.IsInitialized || _allEntities.Count == 0 || (GameManager.Instance?.IsPaused ?? true)) return;

			if (_isJobRunning)
			{
				if (_filterJobHandle.IsCompleted)
				{
					CompleteFilterJob();
					ScheduleFilterJob();
				}
			}
			else
			{
				ScheduleFilterJob();
			}
		}

		private void EnsureNativeArrayCapacity(int requiredCapacity)
		{
			if (!_jobPositions.IsCreated || _jobPositions.Length < requiredCapacity)
			{
				int newCapacity = Mathf.Max(Mathf.CeilToInt(requiredCapacity * 1.3f), 64);

				var newPos = new NativeArray<double2>(newCapacity, Allocator.Persistent);
				var newRadii = new NativeArray<double>(newCapacity, Allocator.Persistent);
				var newMasses = new NativeArray<double>(newCapacity, Allocator.Persistent);
				var newOwned = new NativeArray<bool>(newCapacity, Allocator.Persistent);
				var newKnown = new NativeArray<bool>(newCapacity, Allocator.Persistent);
				var newObserved = new NativeArray<bool>(newCapacity, Allocator.Persistent);
				var newVisible = new NativeArray<bool>(newCapacity, Allocator.Persistent);

				if (_jobPositions.IsCreated)
				{
					NativeArray<double2>.Copy(_jobPositions, newPos, _allEntities.Count);
					NativeArray<double>.Copy(_jobRadii, newRadii, _allEntities.Count);
					NativeArray<double>.Copy(_jobMasses, newMasses, _allEntities.Count);
					NativeArray<bool>.Copy(_jobIsOwned, newOwned, _allEntities.Count);
					NativeArray<bool>.Copy(_jobIsKnown, newKnown, _allEntities.Count);
					NativeArray<bool>.Copy(_jobIsObservedResults, newObserved, _allEntities.Count);
					NativeArray<bool>.Copy(_jobIsVisibleResults, newVisible, _allEntities.Count);

					_jobPositions.Dispose();
					_jobRadii.Dispose();
					_jobMasses.Dispose();
					_jobIsOwned.Dispose();
					_jobIsKnown.Dispose();
					_jobIsObservedResults.Dispose();
					_jobIsVisibleResults.Dispose();
				}

				_jobPositions = newPos;
				_jobRadii = newRadii;
				_jobMasses = newMasses;
				_jobIsOwned = newOwned;
				_jobIsKnown = newKnown;
				_jobIsObservedResults = newObserved;
				_jobIsVisibleResults = newVisible;

				Array.Resize(ref _processingEntities, newCapacity);
			}
		}

		private void EnsureSensorNativeArrayCapacity(int requiredCapacity)
		{
			if (!_jobSensorPositions.IsCreated || _jobSensorPositions.Length < requiredCapacity)
			{
				int newCapacity = Mathf.Max(Mathf.CeilToInt(requiredCapacity * 1.3f), 32);

				var newPos = new NativeArray<double2>(newCapacity, Allocator.Persistent);
				var newHead = new NativeArray<float>(newCapacity, Allocator.Persistent);
				var newRange = new NativeArray<double>(newCapacity, Allocator.Persistent);
				var newFOV = new NativeArray<float>(newCapacity, Allocator.Persistent);
				var newType = new NativeArray<int>(newCapacity, Allocator.Persistent);
				var newSens = new NativeArray<float>(newCapacity, Allocator.Persistent);

				if (_jobSensorPositions.IsCreated)
				{
					NativeArray<double2>.Copy(_jobSensorPositions, newPos, _sensorEntries.Count);
					NativeArray<float>.Copy(_jobSensorHeadings, newHead, _sensorEntries.Count);
					NativeArray<double>.Copy(_jobSensorRanges, newRange, _sensorEntries.Count);
					NativeArray<float>.Copy(_jobSensorFOVs, newFOV, _sensorEntries.Count);
					NativeArray<int>.Copy(_jobSensorTypes, newType, _sensorEntries.Count);
					NativeArray<float>.Copy(_jobSensorSensitivities, newSens, _sensorEntries.Count);

					_jobSensorPositions.Dispose();
					_jobSensorHeadings.Dispose();
					_jobSensorRanges.Dispose();
					_jobSensorFOVs.Dispose();
					_jobSensorTypes.Dispose();
					_jobSensorSensitivities.Dispose();
				}

				_jobSensorPositions = newPos;
				_jobSensorHeadings = newHead;
				_jobSensorRanges = newRange;
				_jobSensorFOVs = newFOV;
				_jobSensorTypes = newType;
				_jobSensorSensitivities = newSens;
			}
		}
		
		private void ScheduleFilterJob()
		{
			int entityCount = _allEntities.Count;
			if (entityCount == 0 || SimulationRenderer.Instance == null) return;

			// Advance time-slice index
			_currentFrameSlice = (_currentFrameSlice + 1) % TimeSliceCount;

			// Synchronize dynamic position, known status, mass, and radius every frame to prevent stale state bugs
			for (int i = 0; i < entityCount; i++)
			{
				var entity = _allEntities[i];
				_jobPositions[i] = entity.Position;
				_jobIsKnown[i] = entity.IsKnown;
				_jobMasses[i] = entity.Mass;
				_jobRadii[i] = entity.Radius;
			}

			for (int i = 0; i < _sensorEntries.Count; i++)
			{
				var entry = _sensorEntries[i];
				foreach (var sensor in entry.Owner.GetCapabilities<SensorCapability>())
				{
					// Update live sensor transforms
					_jobSensorPositions[i] = entry.Owner.Position;
					_jobSensorHeadings[i] = entry.Owner.Orientation + sensor.OrientationOffset;
					break; // Assuming one sensor per entry mapping
				}
			}

			Bounds2D bufferedBounds = SimulationRenderer.Instance.ViewBounds;
			bufferedBounds.Scale(1.0f + SimulationRenderer.Instance.BoundsBufferPercentage, 0);

			var job = new FilterEntitiesJob
			{
				Positions = _jobPositions,
				Radii = _jobRadii,
				Masses = _jobMasses,
				IsOwned = _jobIsOwned,
				IsKnown = _jobIsKnown,
				SensorPositions = _jobSensorPositions.GetSubArray(0, _sensorEntries.Count),
				SensorHeadings = _jobSensorHeadings.GetSubArray(0, _sensorEntries.Count),
				SensorRanges = _jobSensorRanges.GetSubArray(0, _sensorEntries.Count),
				SensorFOVs = _jobSensorFOVs.GetSubArray(0, _sensorEntries.Count),
				SensorTypes = _jobSensorTypes.GetSubArray(0, _sensorEntries.Count),
				SensorSensitivities = _jobSensorSensitivities.GetSubArray(0, _sensorEntries.Count),
				BufferedBounds = bufferedBounds,
				CurrentFrameSlice = _currentFrameSlice,
				TimeSliceCount = TimeSliceCount,
				IsObserved = _jobIsObservedResults,
				IsVisible = _jobIsVisibleResults
			};

			_filterJobHandle = job.Schedule(entityCount, 64);
			_isJobRunning = true;
		}

		private void CompleteFilterJob()
		{
			_filterJobHandle.Complete();
			_isJobRunning = false;
			
			_observedEntities.Clear();
			_visibleEntities.Clear();
			_entitiesToRemove.Clear();

			int count = _allEntities.Count;

			// Gather protected entities from interaction controller
			HashSet<Entity> protectedEntities = new HashSet<Entity>();
			var interactionController = SimulationInteractionController.Instance;
			if (interactionController != null)
			{
				if (interactionController.TargetedEntity != null) protectedEntities.Add(interactionController.TargetedEntity);
				if (interactionController.TrackedEntity != null) protectedEntities.Add(interactionController.TrackedEntity);
				
				foreach (var mapObj in interactionController.SelectedMapObjects)
				{
					if (mapObj != null && mapObj.Entity != null)
						protectedEntities.Add(mapObj.Entity);
				}
				foreach (var mapObj in interactionController.PinnedMapObjects)
				{
					if (mapObj != null && mapObj.Entity != null)
						protectedEntities.Add(mapObj.Entity);
				}
			}

			// Process results directly via array references without any .Contains() overhead
			for (int i = 0; i < count; i++)
			{
				Entity entity = _processingEntities[i];
				if (entity == null) continue;
				
				// if (_jobIsObservedResults[i])
				// {
				// 	if (_observedEntities.Add(entity))
				// 	{
				// 		entity.MarkObserved();
				// 		_knownEntities.Add(entity);
				// 	}
				// }
				// else
				// {
				// 	entity.MarkUnobserved();
				// }
				
				if (i % TimeSliceCount == _currentFrameSlice)
				{
					if (_jobIsObservedResults[i])
					{
						if (_observedEntities.Add(entity))
						{
							entity.MarkObserved();
							_knownEntities.Add(entity);
						}
					}
					else
					{
						entity.MarkUnobserved();
					}
				}

				if (_jobIsVisibleResults[i] || protectedEntities.Contains(entity))
				{
					_visibleEntities.Add(entity);

					if (entity.IsCelestial)
					{
						// Celestials retain individual UI Graphics and unique material instances.
						if (!_activeCelestialVisuals.ContainsKey(entity))
						{
							CelestialMapObject celestialVisual = SimulationRenderer.Instance.GetCelestialVisualFromPool(entity);
							if (celestialVisual != null)
								_activeCelestialVisuals.Add(entity, celestialVisual);
						}

						// Defensive cleanup if an entity's classification ever changes.
						if (_activeVisuals.TryGetValue(entity, out var staleNormalVisual))
						{
							staleNormalVisual.Deinitialize();
							_activeVisuals.Remove(entity);
						}
					}
					else
					{
						// Pure managed data allocation: no Instantiate, Transform, RectTransform, or GameObject.
						if (!_activeVisuals.ContainsKey(entity))
						{
							EntityMapObject newVisual = new EntityMapObject();
							newVisual.InitializeBody(entity);
							_activeVisuals.Add(entity, newVisual);
						}

						if (_activeCelestialVisuals.TryGetValue(entity, out var staleCelestialVisual))
						{
							staleCelestialVisual.Deactivate();
							_activeCelestialVisuals.Remove(entity);
						}
					}
				}
			}

			_entitiesToRemove.Clear();
			foreach (var kvp in _activeVisuals)
			{
				if (!_visibleEntities.Contains(kvp.Key))
					_entitiesToRemove.Add(kvp.Key);
			}
			foreach (var kvp in _activeCelestialVisuals)
			{
				if (!_visibleEntities.Contains(kvp.Key) && !_entitiesToRemove.Contains(kvp.Key))
					_entitiesToRemove.Add(kvp.Key);
			}

			foreach (var entity in _entitiesToRemove)
			{
				if (_activeVisuals.TryGetValue(entity, out var normalVisual))
				{
					normalVisual.Deinitialize();
					_activeVisuals.Remove(entity);
				}

				if (_activeCelestialVisuals.TryGetValue(entity, out var celestialVisual))
				{
					celestialVisual.Deactivate();
					_activeCelestialVisuals.Remove(entity);
				}
			}
		}
	}

	[BurstCompile]
	public struct FilterEntitiesJob : IJobParallelFor
	{
		[ReadOnly] public NativeArray<double2> Positions;
		[ReadOnly] public NativeArray<double> Radii;
		[ReadOnly] public NativeArray<double> Masses;
		[ReadOnly] public NativeArray<bool> IsOwned;
		[ReadOnly] public NativeArray<bool> IsKnown;

		[ReadOnly] public NativeArray<double2> SensorPositions;
		[ReadOnly] public NativeArray<float> SensorHeadings;
		[ReadOnly] public NativeArray<double> SensorRanges;
		[ReadOnly] public NativeArray<float> SensorFOVs;
		[ReadOnly] public NativeArray<int> SensorTypes;
		[ReadOnly] public NativeArray<float> SensorSensitivities;

		[ReadOnly] public Bounds2D BufferedBounds;
		[ReadOnly] public int CurrentFrameSlice;
		[ReadOnly] public int TimeSliceCount;
		
		public NativeArray<bool> IsObserved;
		[WriteOnly] public NativeArray<bool> IsVisible;

		public void Execute(int index)
		{
			bool owned = IsOwned[index];
			bool known = owned || IsKnown[index];
			bool observed = IsObserved[index]; // Preserve previous state for non-evaluated time slices

			double2 targetPos = Positions[index];
			double targetRadius = Radii[index];
			double targetMass = Masses[index];
			
			// Time-slice sensor evaluation
			if (!owned && (index % TimeSliceCount == CurrentFrameSlice))
			{
				observed = false;

				for (int i = 0; i < SensorPositions.Length; i++)
				{
					double2 sensorPos = SensorPositions[i];
					double range = SensorRanges[i];
					double rangeSq = range * range;

					// Squared distance pre-filtering avoids expensive square root calculations
					double2 dir = targetPos - sensorPos;
					double distSq = math.lengthsq(dir);

					if (distSq > rangeSq || distSq <= 1e-10) continue;

					// Check Angle Swatch (Field of View)
					float fov = SensorFOVs[i];
					if (fov < 360f)
					{
						float angleToTarget = math.atan2((float)dir.y, (float)dir.x);
						float heading = SensorHeadings[i];
						float deltaAngle = angleToTarget - heading;

						// Fast angle wrapping without loops
						deltaAngle = math.fmod(deltaAngle + (float)math.PI, 2.0f * (float)math.PI);
						if (deltaAngle < 0f) deltaAngle += 2.0f * (float)math.PI;
						deltaAngle -= (float)math.PI;

						if (math.abs(deltaAngle) > (fov * 0.5f * math.PI / 180.0f))
						{
							continue; // Outside sensor field of view cone
						}
					}

					// Evaluate Distance Fall-off & Detection Strength by Sensor Type
					int sensorType = SensorTypes[i];
					float sensitivity = SensorSensitivities[i];
					double detectionScore = 0.0;

					switch (sensorType)
					{
						case 0: // Optical: Proportional to apparent size (radius^2 / dist^2)
							detectionScore = (targetRadius * targetRadius) / (distSq) * 1e4 * sensitivity;
							break;
						case 1: // Infrared: Proportional to thermal signature (mass / dist^2)
							detectionScore = (targetMass / 1000.0) / (distSq) * 1e8 * sensitivity;
							break;
						case 2: // Radar: Active reflection fall-off (radius^2 / dist^4)
							double distSqSq = distSq * distSq;
							detectionScore = (targetRadius * targetRadius) / (distSqSq) * 1e15 * sensitivity;
							break;
					}

					// Threshold for successful contact detection
					if (detectionScore >= 1.0)
					{
						observed = true;
						break;
					}
				}
			}

			IsObserved[index] = observed;
			
			if (observed || known)
			{
				IsVisible[index] = BufferedBounds.IsWithinBounds(targetPos, targetRadius);
			}
			else
			{
				IsVisible[index] = false;
			}
		}
	}
}