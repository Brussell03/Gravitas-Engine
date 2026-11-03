using System;
using System.Collections.Generic;
using Gravitas2D;
using Unity.Mathematics;
using UnityEngine;

namespace GravitasDemo
{
	public enum EntityType : int
	{
		CelestialBody,
		Asteroid,
		Vehicle,
		Missile,
		Projectile,
		Station
	}

	public class Entity
	{
		private PhysicsBody _physicsBody;
		private bool _isOwned;
		private string _name;
		private double _dryMass;
		private double _maxFuelMass;
		private double _addedMass;
		private float _fuelPercent;
		private bool _isTracked = false;
		private bool _isKnown;
		private bool _isObserved;
		private Entity _dominantEntity;
		private EntityType _entityType;
		private EntityGroup _group;
		private int _numSensors;
		private bool _isGroupable;
		private AutonomyController _autonomyController;

		private List<Entity> _observedEntities = new List<Entity>();
		private List<IEntityCapability> _capabilities = new List<IEntityCapability>();
		private LaunchCapability _launchCapability;
		private MissileCapability _missileCapability;
		private NavigationCapability _navigationCapability;

		private double2 _lastObservedPosition;
		private double2 _lastObservedVelocity;
		private double _lastObservedTime;

		public double CargoMass { get; set; } = 0.0;
		public double MaxCargoCapacity { get; private set; } = 100000.0;

		public PhysicsBody PhysicsBody => _physicsBody;
		public bool IsOwned => _isOwned;
		public string Name => _name;
		public bool IsTracked => _isTracked;
		public bool IsKnown => _isKnown || _isOwned;
		public bool IsObserved => _isObserved || _isOwned;
		public Entity DominantEntity => _dominantEntity;
		public double2 Position => _physicsBody.Position;
		public double2 Velocity => _physicsBody.Velocity;
		public double Radius => _physicsBody.Radius;
		public double Mass => _physicsBody.Mass;
		public float Orientation => _physicsBody.Orientation;
		public float AngularVelocity => _physicsBody.AngularVelocity;
		public bool IsSpawned => !_physicsBody.IsPending;
		public bool IsDestroyed => _physicsBody.IsDestroyed;
		public bool IsCelestial => _physicsBody.IsCelestial;
		public bool IsAsteroid => _entityType == EntityType.Asteroid;
		public bool IsVehicle => _entityType == EntityType.Vehicle;
		public bool IsMissile => _entityType == EntityType.Missile;
		public bool IsProjectile => _entityType == EntityType.Projectile;
		public bool IsStation => _entityType == EntityType.Station;
		public bool IsLauncher => _launchCapability != null;
		public bool IsMissileLauncher => _missileCapability != null;
		public bool IsNavigable => _navigationCapability != null;
		public IAutonomousTask CurrentTask => _autonomyController.CurrentTask;
		public LaunchCapability LaunchCapability => _launchCapability;
		public MissileCapability MissileCapability => _missileCapability;
		public NavigationCapability NavigationCapability => _navigationCapability;
		public List<Entity> ObservedEntities => _observedEntities;
		public EntityType Type => _entityType;
		public CompositionProfile Composition => _physicsBody.Composition;
		public EntityGroup Group => _group;
		public bool InGroup => _group != null && !_group.IsDestroyed;
		public int NumSensors => _numSensors;
		public bool IsAutonomouslyRunning => _autonomyController.IsRunning;
		public bool IsGroupable => _isGroupable;

		public Entity(PhysicsBody body, string name, EntityType type, bool isOwned = false)
		{
			// Input validation: Body reference is strictly required for entity evaluation
			if (body == null) throw new ArgumentNullException(nameof(body), "Entity initialization requires a valid PhysicsBody.");
			
			_physicsBody = body;
			_isOwned = isOwned;
			_isKnown = isOwned;
			_isObserved = isOwned;
			_name = name;
			_entityType = type;

			_isGroupable = type == EntityType.Vehicle;

			_autonomyController = new AutonomyController(this);
			_autonomyController.OnRunningChanged += EntitiesManager.Instance.NotifyAutonomous;

			UpdateDominantEntity(_physicsBody.ParentSoI);
			_physicsBody.OnSoIChanged += UpdateDominantEntity;
			_physicsBody.OnDestroyed += Remove;
		}

		public void RemoveManual()
		{
			_physicsBody.Remove();
			Remove();
		}

		public void Remove()
		{
			_physicsBody.OnSoIChanged -= UpdateDominantEntity;
			_autonomyController.OnRunningChanged -= EntitiesManager.Instance.NotifyAutonomous;
			_observedEntities.Clear();
			EntitiesManager.Instance.UnregisterEntity(this);
		}

		public void MarkObserved()
		{
			_isKnown = true;
			_isObserved = true;
		}

		public void MarkUnobserved()
		{
			_isObserved = false;
			_lastObservedPosition = _physicsBody.Position;
			_lastObservedVelocity = _physicsBody.Velocity;
			_lastObservedTime = PhysicsManager2D.Instance.SimulationElapsedTime;
		}

		#region Capabilities Management

		public void AddCapability(IEntityCapability capability)
		{
			if (capability == null) throw new ArgumentNullException(nameof(capability));

			if (capability is LaunchCapability launchCap)
			{
				if (_launchCapability != null)
				{
					Debug.Log($"[{GetType().Name}, Entity {_name}] already has a launch capability, can't add new one.");
					return;
				}
				
				_launchCapability = launchCap;
			}

			if (capability is MissileCapability misCap)
			{
				if (_missileCapability != null)
				{
					Debug.Log($"[{GetType().Name}, Entity {_name}] already has a missile capability, can't add new one.");
					return;
				}
				
				_missileCapability = misCap;
			}

			// Automatically compute attitude gains using vehicle inertia tensor when adding NavigationCapability
			if (capability is NavigationCapability navCap)
			{
				if (_navigationCapability != null)
				{
					Debug.Log($"[{GetType().Name}, Entity {_name}] already has a navigation capability, can't add new one.");
					return;
				}

				_navigationCapability = navCap;
				_navigationCapability.OwnerEntity = this;
			}

			if (capability is SensorCapability senCap)
			{
				_numSensors++;
			}

			_capabilities.Add(capability);
			EntitiesManager.Instance?.NotifyCapabilityAdded(this, capability);

			Debug.Log($"[{GetType().Name}, Entity {_name}] Added capability: {capability.GetType().Name}");
		}

		public T GetCapability<T>() where T : class, IEntityCapability
		{
			foreach (var cap in _capabilities)
			{
				if (cap is T match) return match;
			}
			return null;
		}

		public IEnumerable<T> GetCapabilities<T>() where T : class, IEntityCapability
		{
			foreach (var cap in _capabilities)
			{
				if (cap is T match) yield return match;
			}
		}

		public bool HasCapability<T>() where T : class, IEntityCapability
		{
			return GetCapability<T>() != null;
		}

		#endregion

		public void SetTracked(bool isTracked)
		{
			_isTracked = isTracked;
			_physicsBody.SetTracked(isTracked);
		}

		public void Fracture() => _physicsBody.Fracture();

		private void UpdateDominantEntity(PhysicsBody parentSoI)
		{
			if (parentSoI == null)
			{
				_dominantEntity = null;
				return;
			}
			_dominantEntity = EntitiesManager.Instance.EntityLookup[parentSoI.Name];
		}

		private void UpdatePhysicsBodyMass()
		{
			double totalMass = _dryMass + _addedMass + _maxFuelMass * _fuelPercent + CargoMass;
			_physicsBody.SetMass(totalMass);
		}

		/// <summary>
		/// Queries all other entities visible to this entity's active sensors using the spatial hash map.
		/// </summary>
		public void UpdateObservedEntities()
		{
			_observedEntities.Clear();

			if (PhysicsManager2D.Instance == null || !PhysicsManager2D.Instance.IsInitialized)
				return;

			var sensors = GetCapabilities<SensorCapability>();
			var sensorList = sensors != null ? new List<SensorCapability>(sensors) : new List<SensorCapability>();

			if (sensorList.Count == 0) return;

			PhysicsManager2D.Instance.QuerySpatialHash((spatialMap, bodyStateVectors, cellSize) =>
			{
				double maxRange = 0.0;
				for (int i = 0; i < sensorList.Count; i++)
				{
					if (sensorList[i].Range > maxRange) maxRange = sensorList[i].Range;
				}

				HashSet<int> evaluatedIndices = new HashSet<int>();
				int cellRadius = math.max(1, (int)math.ceil(maxRange / cellSize));
				
				int centerCellX = (int)math.floor(Position.x / cellSize);
				int centerCellY = (int)math.floor(Position.y / cellSize);

				for (int x = -cellRadius; x <= cellRadius; x++)
				{
					for (int y = -cellRadius; y <= cellRadius; y++)
					{
						int hash = SpatialHash.GetCellHash(centerCellX + x, centerCellY + y);

						if (spatialMap.TryGetFirstValue(hash, out int bodyIndex, out var iterator))
						{
							do
							{
								if (bodyIndex == _physicsBody.Index || evaluatedIndices.Contains(bodyIndex)) continue;
								evaluatedIndices.Add(bodyIndex);

								if (bodyIndex < 0 || bodyIndex >= bodyStateVectors.Length) continue;

								StateVector targetStateVector = bodyStateVectors[bodyIndex];
								PhysicsBody targetBody = PhysicsManager2D.Instance.PhysicsBodies[bodyIndex];
								if (targetBody == null || targetBody.IsDestroyed) continue;

								double targetRadius = targetBody.Radius;
								double targetMass = targetBody.Mass;
								double2 targetPos = targetStateVector.Position;
								double2 dir = targetPos - Position;
								double dist = math.length(dir);

								for (int s = 0; s < sensorList.Count; s++)
								{
									var sensor = sensorList[s];
									if (dist > sensor.Range || dist <= 1e-5) continue;

									// Field of View Check
									if (sensor.FieldOfViewDegrees < 360f)
									{
										float angleToTarget = math.atan2((float)dir.y, (float)dir.x);
										float heading = Orientation + sensor.OrientationOffset;
										float deltaAngle = angleToTarget - heading;

										while (deltaAngle > (float)math.PI) deltaAngle -= 2.0f * (float)math.PI;
										while (deltaAngle < -(float)math.PI) deltaAngle += 2.0f * (float)math.PI;

										if (math.abs(deltaAngle) > (sensor.FieldOfViewDegrees * 0.5f * math.PI / 180.0f))
										{
											continue;
										}
									}

									// Evaluate Distance Fall-off & Detection Strength by Sensor Type (matching FilterEntitiesJob)
									double detectionScore = 0.0;
									switch (sensor.Type)
									{
										case SensorType.Optical:
											detectionScore = (targetRadius * targetRadius) / (dist * dist) * 1e4 * sensor.Sensitivity;
											break;
										case SensorType.Infrared:
											detectionScore = (targetMass / 1000.0) / (dist * dist) * 1e8 * sensor.Sensitivity;
											break;
										case SensorType.Radar:
											double distSq = dist * dist;
											detectionScore = (targetRadius * targetRadius) / (distSq * distSq) * 1e15 * sensor.Sensitivity;
											break;
									}

									// Threshold for successful contact detection matching the visualizer bounds
									if (detectionScore >= 1.0)
									{
										if (EntitiesManager.Instance != null && 
											EntitiesManager.Instance.EntityBodyLookup.TryGetValue(targetBody, out Entity matchedEntity))
										{
											if (!_observedEntities.Contains(matchedEntity))
											{
												_observedEntities.Add(matchedEntity);
											}
										}
										break; // Successfully detected by at least one sensor
									}
								}

							} while (spatialMap.TryGetNextValue(out bodyIndex, ref iterator));
						}
					}
				}
			});
		}

		#region Autonomy Controls

		public void EnqueueTask(IAutonomousTask task, bool addToLoopTemplate = false)
		{
			if (_autonomyController == null)
			{
				_autonomyController = new AutonomyController(this);
			}
			_autonomyController.EnqueueTask(task, addToLoopTemplate);
		}

		public void StartAutonomy(bool loop = false)
		{
			if (_autonomyController == null) return;
			_autonomyController.IsLooping = loop;
			_autonomyController.StartAutonomy();
		}

		public void StopAutonomy() => _autonomyController?.StopAutonomy();
		
		public void ClearTasks() => _autonomyController?.ClearTasks();

		public void EvaluateAutonomy(float dt) => _autonomyController?.EvaluateAutonomy(dt);

		public void StartMiningLoop(Entity station, Entity asteroid)
		{
			if (station == null || station.IsDestroyed || asteroid == null || asteroid.IsDestroyed) return;

			// Setup harvesting and deposit cycle
			EnqueueTask(new NavigateTask(asteroid, isRendezvous: true), addToLoopTemplate: true);
			EnqueueTask(new ExtractMassTask(_autonomyController, asteroid, extractionAmount: 100000.0, ratePerSecond: 100.0), addToLoopTemplate: true);
			EnqueueTask(new NavigateTask(station, isRendezvous: true), addToLoopTemplate: true);
			EnqueueTask(new DepositMassTask(_autonomyController, station, ratePerSecond: 200.0), addToLoopTemplate: true);

			// Start with continuous infinite looping enabled
			StartAutonomy(loop: true);
		}

		public void StartMiningLoopHard()
		{
			var station = EntitiesManager.Instance.EntityLookup["Mining Station"];
			var asteroid = EntitiesManager.Instance.EntityLookup["M-Type Asteroid"];

			StartMiningLoop(station, asteroid);
		}

		#endregion

		#region Grouping

		public void FormGroup(params Entity[] otherEntities)
		{
			new EntityGroup(this, otherEntities);
		}

		public bool JoinGroup(EntityGroup group)
		{
			if (InGroup) LeaveGroup();

			if (group.AddEntity(this))
			{
				return true;
			}

			return false;
		}

		public void LeaveGroup()
		{
			_group.RemoveEntity(this);
		}

		public void SetGroup(EntityGroup group)
		{
			_group = group;
		}

		#endregion

		#region Navigation Controls

		public void HandleNavigationTask(IAutonomousTask task, bool enqueue)
		{
			var navCap = GetCapability<NavigationCapability>();
			if (navCap == null)
			{
				Debug.LogWarning($"[{GetType().Name}] {Name} lacks a NavigationCapability.");
				return;
			}

			if (!enqueue)
			{
				ClearTasks();
			}

			EnqueueTask(task);

			if (_autonomyController != null && !_autonomyController.IsRunning)
			{
				StartAutonomy();
			}
		}

		public void InterceptEntity(Entity targetEntity, float arrivalThreshold = 10f, float timeoutSeconds = float.PositiveInfinity, bool enqueue = false)
		{
			if (targetEntity == null)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot navigate: Target entity is null.");
				return;
			}
			HandleNavigationTask(new NavigateTask(targetEntity, isRendezvous: false, arrivalThreshold, timeoutSeconds), enqueue);
		}

		public void InterceptPosition(double2 targetPosition, float arrivalThreshold = 10f, float timeoutSeconds = 300f, bool enqueue = false)
		{
			HandleNavigationTask(new NavigateTask(targetPosition, isRendezvous: false, arrivalThreshold, timeoutSeconds), enqueue);
		}

		public void RendezvousWithEntity(Entity targetEntity, float arrivalThreshold = 10f, float timeoutSeconds = float.PositiveInfinity, bool enqueue = false)
		{
			if (targetEntity == null)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot navigate: Target entity is null.");
				return;
			}
			HandleNavigationTask(new NavigateTask(targetEntity, isRendezvous: true, arrivalThreshold, timeoutSeconds), enqueue);
		}

		public void RendezvousWithPosition(double2 targetPosition, float arrivalThreshold = 10f, float timeoutSeconds = 300f, bool enqueue = false)
		{
			HandleNavigationTask(new NavigateTask(targetPosition, isRendezvous: true, arrivalThreshold, timeoutSeconds), enqueue);
		}

		public void StationKeepWithEntity(Entity targetEntity, float timeoutSeconds = float.PositiveInfinity, bool enqueue = false)
		{
			if (targetEntity == null)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot station keep: Target entity is null.");
				return;
			}
			HandleNavigationTask(new StationKeepTask(targetEntity, timeoutSeconds), enqueue);
		}

		public void StationKeepWithPosition(double2 targetPosition, float timeoutSeconds = float.PositiveInfinity, bool enqueue = false)
		{
			HandleNavigationTask(new StationKeepTask(targetPosition, timeoutSeconds), enqueue);
		}

		public void OrbitMatchWithEntity(Entity targetEntity, float timeoutSeconds = float.PositiveInfinity, bool enqueue = false)
		{
			if (targetEntity == null) return;
			HandleNavigationTask(new OrbitMatchTask(targetEntity, timeoutSeconds), enqueue);
		}

		public void OrbitMatchWithState(double2 targetPosition, double2 targetVelocity, float timeoutSeconds = float.PositiveInfinity, bool enqueue = false)
		{
			HandleNavigationTask(new OrbitMatchTask(targetPosition, targetVelocity, timeoutSeconds), enqueue);
		}

		public void SetTargetOrientation(float targetOrientation, float timeoutSeconds = float.PositiveInfinity, bool enqueue = false)
		{
			HandleNavigationTask(new OrientationTask(targetOrientation, timeoutSeconds), enqueue);
		}

		/// <summary>
		/// Disables navigation and attitude holding, then puts the entity into a constant spin 
		/// at the specified angular velocity by applying a single torque impulse.
		/// </summary>
		public void StartConstantSpin(float targetAngularVelocity)
		{
			if (_physicsBody == null || _physicsBody.IsDestroyed)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot apply spin: PhysicsBody is null or destroyed.");
				return;
			}

			// 1. Disable active navigation and attitude control so they don't counteract the spin
			if (_navigationCapability != null)
			{
				_navigationCapability.EndNavigation();
			}

			// 2. Compute the required torque impulse from body inertia and velocity delta
			double inertia = _physicsBody.Inertia;
			if (inertia <= 0.0 || double.IsNaN(inertia) || double.IsInfinity(inertia))
			{
				inertia = 1.0;
			}

			float currentOmega = AngularVelocity;
			float deltaOmega = targetAngularVelocity - currentOmega;

			// Retrieve live physics timestep for correct impulse scaling
			float dt = Time.fixedDeltaTime;
			PhysicsManager2D manager = PhysicsManager2D.Instance;
			if (manager != null && manager.IsInitialized)
			{
				float scaledStep = manager.ScaledPhysicsTimeStep;
				if (scaledStep > 0.00001f)
					dt = scaledStep;
			}

			float torqueImpulse = (float)(inertia * deltaOmega) / Mathf.Max(0.0001f, dt);

			// 3. Apply the single torque impulse to the physics body
			_physicsBody.AddTorque(torqueImpulse);

			Debug.Log($"[{GetType().Name}] Entity {_name} navigation disabled. Constant spin initiated at {targetAngularVelocity} rad/s (torque impulse: {torqueImpulse:F2}).");
		}

		#endregion
	}
}