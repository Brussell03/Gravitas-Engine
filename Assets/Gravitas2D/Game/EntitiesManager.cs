using UnityEngine;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Gravitas2D;
using System;
using System.Collections.Generic;
using System.Linq;

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
		private Dictionary<string, Entity> _entityLookup = new Dictionary<string, Entity>();
		private Dictionary<PhysicsBody, Entity> _entityBodyLookup = new Dictionary<PhysicsBody, Entity>();

		// Visual Mappings
		private readonly Dictionary<Entity, EntityMapObject> _activeVisuals = new Dictionary<Entity, EntityMapObject>();
		private readonly List<Entity> _entitiesToRemove = new List<Entity>();

		// Job System Variables
		private JobHandle _filterJobHandle;
		private bool _isJobRunning = false;
		private int _scheduledEntityCount = 0;
		
		private NativeArray<double2> _jobPositions;
		private NativeArray<double> _jobRadii;
		private NativeArray<double> _jobMasses;
		private NativeArray<bool> _jobIsOwned;
		private NativeArray<bool> _jobIsKnown;
		
		// Flattened Sensor Arrays
		private NativeArray<double2> _jobSensorPositions;
		private NativeArray<float> _jobSensorHeadings;
		private NativeArray<double> _jobSensorRanges;
		private NativeArray<float> _jobSensorFOVs;
		private NativeArray<int> _jobSensorTypes;
		private NativeArray<float> _jobSensorSensitivities;
		
		private NativeArray<bool> _jobIsObservedResults;
		private NativeArray<bool> _jobIsVisibleResults;
		
		private Entity[] _processingEntities = new Entity[0];

		public IReadOnlyCollection<Entity> KnownEntities => _knownEntities;
		public IReadOnlyCollection<Entity> ObservedEntities => _observedEntities;
		public IReadOnlyCollection<Entity> VisibleEntities => _visibleEntities;
		public IReadOnlyCollection<Entity> NavigableEntities => _navigableEntities;
		public IReadOnlyCollection<Entity> SensorEntities => _sensorEntities;
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
			PhysicsManager2D.Instance.OnPhysicsUpdated += UpdateNavigations;
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
				PhysicsManager2D.Instance.OnPhysicsUpdated -= UpdateNavigations;
				PhysicsManager2D.Instance.OnSimulationOriginatedBody -= EntityCreation.CreateWithExistingBody;
			}
		}

		public bool RegisterEntity(Entity entity)
		{
			if (entity == null) return false;

			if (!_allEntities.Contains(entity))
			{
				_allEntities.Add(entity);
				_entityLookup.Add(entity.Name, entity);
				_entityBodyLookup.Add(entity.PhysicsBody, entity);

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
				}

				return true;
			}

			return false;
		}

		public void UnregisterEntity(Entity entity)
		{
			if (entity == null) return;
			_allEntities.Remove(entity);
			_knownEntities.Remove(entity);
			_observedEntities.Remove(entity);
			_visibleEntities.Remove(entity);
			_navigableEntities.Remove(entity);
			_sensorEntities.Remove(entity);
			_entityLookup.Remove(entity.Name);
			_entityBodyLookup.Remove(entity.PhysicsBody);
			
			if (_activeVisuals.TryGetValue(entity, out var visual))
			{
				visual.Deactivate();
				_activeVisuals.Remove(entity);
			}
		}

		public void NotifyCapabilityAdded(Entity entity, IEntityCapability capability)
		{
			if (capability is NavigationCapability)
			{
				_navigableEntities.Add(entity);
			}
			else if (capability is SensorCapability)
			{
				_sensorEntities.Add(entity);
			}
		}

		public EntityMapObject GetVisualForEntity(Entity entity)
		{
			return _activeVisuals.TryGetValue(entity, out var bodyVisual) ? bodyVisual : null;
		}

		public void ClearEntities()
		{
			_filterJobHandle.Complete();
			_isJobRunning = false;
			
			for (int i = _allEntities.Count - 1; i >= 0; i--)
			{
				_allEntities[i].RemoveManual();
			}

			DisposeNativeArrays();
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

		private void UpdateNavigations()
		{
			if (PhysicsManager2D.Instance == null || !PhysicsManager2D.Instance.IsInitialized || _navigableEntities.Count == 0) return;

			float dt = PhysicsManager2D.Instance.ScaledPhysicsTimeStep;

			foreach (var entity in _navigableEntities)
			{
				if (entity == null || entity.IsDestroyed) continue;
				entity.EvaluateNavigation(dt);
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
			if (PhysicsManager2D.Instance == null || !PhysicsManager2D.Instance.IsInitialized || _allEntities == null || _allEntities.Count == 0 || (GameManager.Instance?.IsPaused ?? true)) return;

			// Manage async filter cycle
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

		private void EnsureNativeArrayCapacities(int entityCount, int sensorCount)
		{
			if (!_jobPositions.IsCreated || _jobPositions.Length < entityCount)
			{
				DisposeNativeArrays();
				
				int paddedCount = Mathf.CeilToInt(entityCount * 1.2f);
				int paddedSensors = Mathf.Max(32, Mathf.CeilToInt(sensorCount * 1.5f));
				
				_jobPositions = new NativeArray<double2>(paddedCount, Allocator.Persistent);
				_jobRadii = new NativeArray<double>(paddedCount, Allocator.Persistent);
				_jobMasses = new NativeArray<double>(paddedCount, Allocator.Persistent);
				_jobIsOwned = new NativeArray<bool>(paddedCount, Allocator.Persistent);
				_jobIsKnown = new NativeArray<bool>(paddedCount, Allocator.Persistent);
				
				_jobSensorPositions = new NativeArray<double2>(paddedSensors, Allocator.Persistent);
				_jobSensorHeadings = new NativeArray<float>(paddedSensors, Allocator.Persistent);
				_jobSensorRanges = new NativeArray<double>(paddedSensors, Allocator.Persistent);
				_jobSensorFOVs = new NativeArray<float>(paddedSensors, Allocator.Persistent);
				_jobSensorTypes = new NativeArray<int>(paddedSensors, Allocator.Persistent);
				_jobSensorSensitivities = new NativeArray<float>(paddedSensors, Allocator.Persistent);
				
				_jobIsObservedResults = new NativeArray<bool>(paddedCount, Allocator.Persistent);
				_jobIsVisibleResults = new NativeArray<bool>(paddedCount, Allocator.Persistent);
				
				Array.Resize(ref _processingEntities, paddedCount);
			}
			else if (_jobSensorPositions.Length < sensorCount)
			{
				ResizeSensorArrays(Mathf.CeilToInt(sensorCount * 1.5f));
			}
		}

		private void ResizeSensorArrays(int newCapacity)
		{
			var newPos = new NativeArray<double2>(newCapacity, Allocator.Persistent);
			var newHead = new NativeArray<float>(newCapacity, Allocator.Persistent);
			var newRange = new NativeArray<double>(newCapacity, Allocator.Persistent);
			var newFOV = new NativeArray<float>(newCapacity, Allocator.Persistent);
			var newType = new NativeArray<int>(newCapacity, Allocator.Persistent);
			var newSens = new NativeArray<float>(newCapacity, Allocator.Persistent);

			if (_jobSensorPositions.IsCreated)
			{
				NativeArray<double2>.Copy(_jobSensorPositions, newPos, _jobSensorPositions.Length);
				NativeArray<float>.Copy(_jobSensorHeadings, newHead, _jobSensorHeadings.Length);
				NativeArray<double>.Copy(_jobSensorRanges, newRange, _jobSensorRanges.Length);
				NativeArray<float>.Copy(_jobSensorFOVs, newFOV, _jobSensorFOVs.Length);
				NativeArray<int>.Copy(_jobSensorTypes, newType, _jobSensorTypes.Length);
				NativeArray<float>.Copy(_jobSensorSensitivities, newSens, _jobSensorSensitivities.Length);

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
		
		private void ScheduleFilterJob()
		{
			int entityCount = _allEntities.Count;
			if (entityCount == 0 || SimulationRenderer.Instance == null) return;

			// Snapshot the count at schedule time
			_scheduledEntityCount = entityCount;

			// First pass: Count total active sensors on owned entities
			int totalSensors = 0;
			for (int i = 0; i < entityCount; i++)
			{
				var entity = _allEntities[i];
				if (entity.IsOwned)
				{
					foreach (var _ in entity.GetCapabilities<SensorCapability>())
					{
						totalSensors++;
					}
				}
			}

			EnsureNativeArrayCapacities(entityCount, totalSensors);

			int sensorIndex = 0;
			for (int i = 0; i < entityCount; i++)
			{
				Entity entity = _allEntities[i];
				_processingEntities[i] = entity;
				_jobPositions[i] = entity.Position;
				_jobRadii[i] = entity.Radius;
				_jobMasses[i] = entity.Mass;
				_jobIsOwned[i] = entity.IsOwned;
				_jobIsKnown[i] = entity.IsKnown;

				if (entity.IsOwned)
				{
					foreach (var sensor in entity.GetCapabilities<SensorCapability>())
					{
						_jobSensorPositions[sensorIndex] = entity.Position;
						_jobSensorHeadings[sensorIndex] = entity.Orientation + sensor.OrientationOffset;
						_jobSensorRanges[sensorIndex] = sensor.Range;
						_jobSensorFOVs[sensorIndex] = sensor.FieldOfViewDegrees;
						_jobSensorTypes[sensorIndex] = (int)sensor.Type;
						_jobSensorSensitivities[sensorIndex] = sensor.Sensitivity;
						sensorIndex++;
					}
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
				SensorPositions = _jobSensorPositions.GetSubArray(0, sensorIndex),
				SensorHeadings = _jobSensorHeadings.GetSubArray(0, sensorIndex),
				SensorRanges = _jobSensorRanges.GetSubArray(0, sensorIndex),
				SensorFOVs = _jobSensorFOVs.GetSubArray(0, sensorIndex),
				SensorTypes = _jobSensorTypes.GetSubArray(0, sensorIndex),
				SensorSensitivities = _jobSensorSensitivities.GetSubArray(0, sensorIndex),
				BufferedBounds = bufferedBounds,
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

			int count = _scheduledEntityCount;

			// Gather all entities currently involved in interactions that must remain rendered
			HashSet<Entity> protectedEntities = new HashSet<Entity>();
			var interactionController = SimulationInteractionController.Instance;
			if (interactionController != null)
			{
				if (interactionController.TargetedEntity != null) protectedEntities.Add(interactionController.TargetedEntity);
				if (interactionController.TrackedEntity != null) protectedEntities.Add(interactionController.TrackedEntity);
				
				foreach (var mapObj in interactionController.SelectedMapObjects)
				{
					if (mapObj is EntityMapObject ego && ego.Entity != null) protectedEntities.Add(ego.Entity);
				}
				foreach (var mapObj in interactionController.PinnedMapObjects)
				{
					if (mapObj is EntityMapObject ego && ego.Entity != null) protectedEntities.Add(ego.Entity);
				}
			}

			// 1. Single pass to evaluate job results and map to discrete collections
			for (int i = 0; i < count; i++)
			{
				Entity entity = _processingEntities[i];

				// Safeguard against null entries or entities unregistered during the job cycle
				if (entity == null || !_allEntities.Contains(entity)) continue;
				
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

				if (_jobIsVisibleResults[i] || protectedEntities.Contains(entity))
				{
					_visibleEntities.Add(entity);

					// Instantiate and map visual proxy directly from the rendering pool
					if (!_activeVisuals.ContainsKey(entity))
					{
						EntityMapObject newVisual = SimulationRenderer.Instance.GetEntityFromPool(entity);
						if (newVisual != null) _activeVisuals.Add(entity, newVisual);
					}
				}
			}

			// 2. Diff against active visual dictionary to target off-screen or undetected bodies
			foreach (var kvp in _activeVisuals)
			{
				if (!_visibleEntities.Contains(kvp.Key))
				{
					_entitiesToRemove.Add(kvp.Key);
				}
			}

			// 3. Deactivate flagged bodies securely
			foreach (var entity in _entitiesToRemove)
			{
				try
				{
					_activeVisuals[entity].Deactivate();
					_activeVisuals.Remove(entity);
				}
				catch (Exception ex)
				{
					Debug.LogError($"[{GetType().Name}] Error removing visual for entity: {ex.Message}");
				}
			}

			// 4. Release object references to prevent GC leaks on destroyed entities
			Array.Clear(_processingEntities, 0, count);
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
		
		[WriteOnly] public NativeArray<bool> IsObserved;
		[WriteOnly] public NativeArray<bool> IsVisible;

		public void Execute(int index)
		{
			bool owned = IsOwned[index];
			bool known = owned || IsKnown[index];
			bool observed = false;

			double2 targetPos = Positions[index];
			double targetRadius = Radii[index];
			double targetMass = Masses[index];
			
			if (!owned)
			{
				for (int i = 0; i < SensorPositions.Length; i++)
				{
					double2 sensorPos = SensorPositions[i];
					double range = SensorRanges[i];
					double2 dir = targetPos - sensorPos;
					double dist = math.length(dir);

					if (dist > range || dist <= 1e-5) continue;

					// Check Angle Swatch (Field of View)
					float fov = SensorFOVs[i];
					if (fov < 360f)
					{
						float angleToTarget = math.atan2((float)dir.y, (float)dir.x);
						float heading = SensorHeadings[i];
						float deltaAngle = angleToTarget - heading;

						while (deltaAngle > (float)math.PI) deltaAngle -= 2.0f * (float)math.PI;
						while (deltaAngle < -(float)math.PI) deltaAngle += 2.0f * (float)math.PI;

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
							detectionScore = (targetRadius * targetRadius) / (dist * dist) * 1e4 * sensitivity;
							break;
						case 1: // Infrared: Proportional to thermal signature (mass / dist^2)
							detectionScore = (targetMass / 1000.0) / (dist * dist) * 1e8 * sensitivity;
							break;
						case 2: // Radar: Active reflection fall-off (radius^2 / dist^4)
							double distSq = dist * dist;
							detectionScore = (targetRadius * targetRadius) / (distSq * distSq) * 1e15 * sensitivity;
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