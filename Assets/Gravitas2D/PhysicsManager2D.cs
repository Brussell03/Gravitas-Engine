using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Gravitas2D
{
	[DefaultExecutionOrder(-50)]
	public class PhysicsManager2D : MonoBehaviour
	{
		public static PhysicsManager2D Instance { get; private set; }

		public enum Integrator { RK4, VelocityVerlet }

		[SerializeField] private Integrator _integrationMethod;
		[SerializeField] private double2 _simBoundsCenter;
		[SerializeField, Min(1)] private double _simBoundsRadius;
		[SerializeField] private List<FractureProfile> _fractureProfiles;
		[SerializeField] private List<CompositionProfile> _compositionProfiles;

		[Header("Broadphase Collision")]
		[SerializeField, Min(1f)] private double _spatialCellSize = 1000000f;

		public event Action OnPhysicsUpdated;
		public event Action<bool> OnSimulationPaused;
		public event Action<PhysicsBody, PhysicsBody> OnSimulationOriginatedBody;

		private float _physicsTimeStep;
		private float _currentTimeStep;
		private float _timeStepMultiplier = 1;
		private bool _isInitialized = false;
		private bool _isPaused = true;
		private bool _isForcePaused = false;
		private float _simulationElapsedTime;
		private int _stepCount = 0;
		private float _fractureRadiusThreshold;
		private Bounds2D _simulationBounds;
		private List<PhysicsBody> _physicsBodies = new List<PhysicsBody>();
		private Dictionary<string, PhysicsBody> _physicsBodyLookup = new Dictionary<string, PhysicsBody>();
		private NativeList<PhysicsBodyData> _physicsBodiesData;
		private NativeList<float> _breakSpeeds;
		private NativeList<double> _warheadPayloadEnergies;
		private NativeList<bool> _isCelestialMap;
		private NativeList<bool> _isCollisionTriggerMap;
		private NativeList<bool> _isCollidableMap;

		// Deferral Queues
		private Queue<PendingAddition> _bodiesToAdd = new Queue<PendingAddition>();
		private List<DelayedAddition> _delayedBodies = new List<DelayedAddition>();
		private List<PhysicsBody> _bodiesToRemove = new List<PhysicsBody>();
		private List<CollisionActivation> _collisionActivations = new List<CollisionActivation>();

		// Background Bounds Check State
		private Coroutine _boundsCheckCoroutine;
		private JobHandle _boundsCheckHandle;
		private NativeList<int> _outOfBoundsIndices;
		private bool _isBoundsCheckRunning = false;

		private JobHandle _propagateBodiesJobHandle;
		private bool _physicsJobScheduled = false;

		private NativeParallelMultiHashMap<int, int> _spatialHashMap;
		private NativeQueue<CollisionEvent> _collisionQueue;
		private NativeQueue<int> _bodiesToDestroyQueue;
		private NativeQueue<int> _bodiesToFractureQueue;
		private NativeArray<PhysicsBodyData> _nextPhysicsBodiesData;

		public bool IsInitialized => _isInitialized;
		public bool IsPaused => _isPaused;
		public float PhysicsTimeStep => _physicsTimeStep;
		public bool IsPhysicsStepRunning => _physicsJobScheduled;
		public List<PhysicsBody> PhysicsBodies => _physicsBodies;
		public Dictionary<string, PhysicsBody> PhysicsBodyLookup => _physicsBodyLookup;
		public float SimulationElapsedTime => _simulationElapsedTime;
		public Bounds2D SimulationBounds => _simulationBounds;
		public float ScaledPhysicsTimeStep => _physicsTimeStep * _timeStepMultiplier;
		public bool IsAtSpeed1 => _timeStepMultiplier == 1f;
		public bool IsAtSpeed2 => _timeStepMultiplier == 10f;
		public bool IsAtSpeed3 => _timeStepMultiplier == 100f;
		public bool IsAtSpeed4 => _timeStepMultiplier == 3600f;
		public float FractureRadiusThreshold => _fractureRadiusThreshold;
		public List<FractureProfile> FractureProfiles => _fractureProfiles;
		public List<CompositionProfile> CompositionProfiles => _compositionProfiles;
		public NativeParallelMultiHashMap<int, int> SpatialHashMap => _spatialHashMap;
		public double SpatialCellSize => _spatialCellSize;

		private struct PendingAddition
		{
			public PhysicsBody Body;
			public StateVector State;
			public VehicleModel Model;
			public int CompositionIndex;
			public bool IsCelestial;
			public bool IsCollisionTrigger;
			public bool IsCollidable;
			public string Name;
			public double WarheadEnergy;
			public bool EnableCollisionsAfterDelay;
		}

		private struct DelayedAddition
		{
			public PhysicsBody Body;
			public StateVector State;
			public PhysicsBody ReferenceBody;
			public VehicleModel Model;
			public int CompositionIndex;
			public bool IsCelestial;
			public bool IsCollisionTrigger;
			public bool IsCollidable;
			public string Name;
			public double RemainingDelay;
			public double WarheadEnergy;
			public bool EnableCollisionsAfterDelay;
		}

		private struct CollisionActivation
		{
			public PhysicsBody Body;
			public double RemainingDelay;
		}

		private void Awake()
		{
			if (Instance != null && Instance != this)
			{
				Destroy(gameObject);
				return;
			}
			Instance = this;

			_simulationBounds = new Bounds2D(_simBoundsCenter, _simBoundsRadius);

			StartCoroutine(Initialize());
		}

		private IEnumerator Initialize()
		{
			if (_isInitialized) yield break;
			yield return null;

			Debug.Log($"[{GetType().Name}] Performing first-time initialization...");

			// Initialize collections
			_physicsBodies.Clear();
			_physicsBodyLookup.Clear();
			_physicsBodiesData = new NativeList<PhysicsBodyData>(Allocator.Persistent);
			_breakSpeeds = new NativeList<float>(Allocator.Persistent);
			_warheadPayloadEnergies = new NativeList<double>(Allocator.Persistent);
			_isCelestialMap = new NativeList<bool>(Allocator.Persistent);
			_isCollisionTriggerMap = new NativeList<bool>(Allocator.Persistent);
			_isCollidableMap = new NativeList<bool>(Allocator.Persistent);

			_spatialHashMap = new NativeParallelMultiHashMap<int, int>(4096, Allocator.Persistent);
			_collisionQueue = new NativeQueue<CollisionEvent>(Allocator.Persistent);
			_bodiesToDestroyQueue = new NativeQueue<int>(Allocator.Persistent);
			_bodiesToFractureQueue = new NativeQueue<int>(Allocator.Persistent);

			_physicsTimeStep = Time.fixedDeltaTime;
			
			// Check through fracture profiles to find minimum radius threshold
			_fractureRadiusThreshold = float.MaxValue;
			for (int i = 0; i < _fractureProfiles.Count; i++)
			{
				if (_fractureRadiusThreshold > _fractureProfiles[i].MinRadiusThreshold)
				{
					_fractureRadiusThreshold = _fractureProfiles[i].MinRadiusThreshold;
				}
			}

			// Initialize gravitational management class
			GravitySources.Initialize();

			// Mark core setup as complete
			_isInitialized = true;
			Debug.Log($"[{GetType().Name}] Core Initialization Complete.");
			
			// Pause and set default time scale
			SetTimeScale(1);
			SetPaused(true);

			// Start background bounds checking
			_boundsCheckCoroutine = StartCoroutine(BoundsCheckRoutine());
		}

		#region Physics Loop

		private void FixedUpdate()
		{
			// 1. Always resolve scheduled jobs and dispose memory first, regardless of simulation state
			FinishPhysicsStep();
			
			// 2. Perform early-exit check for the rest of the simulation loop
			if (!_isInitialized) return;
			
			// 3. Process remaining queue modifications if no job was scheduled
			if (!_physicsJobScheduled && (_bodiesToAdd.Count > 0 || _bodiesToRemove.Count > 0))
			{
				ProcessPendingChanges();
			}

			// 4. Start the next physics step
			PhysicsStep();
		}

		private void PhysicsStep()
		{
			if (!_isInitialized || _physicsBodies.Count == 0 || _physicsJobScheduled || _isPaused || _isForcePaused) return;

			_physicsJobScheduled = true;
			_currentTimeStep = ScaledPhysicsTimeStep;
			_stepCount++;

			// Resize physics data array if changed
			if (!_nextPhysicsBodiesData.IsCreated || _nextPhysicsBodiesData.Length != _physicsBodiesData.Length)
			{
				if (_nextPhysicsBodiesData.IsCreated) _nextPhysicsBodiesData.Dispose();
				_nextPhysicsBodiesData = new NativeArray<PhysicsBodyData>(_physicsBodiesData.Length, Allocator.Persistent);
			}

			// 1. Update states of gravitational sources
			GravitySources.ScheduleStateUpdateJob(_physicsBodiesData.AsArray(), out JobHandle dependency);

			_spatialHashMap.Clear();
			_collisionQueue.Clear();
			_bodiesToDestroyQueue.Clear();
			_bodiesToFractureQueue.Clear();

			// Ensure Spatial Hash Map Capacity
			// NativeParallelMultiHashMap parallel writers throw exceptions if capacity is exceeded during a job.
			if (_physicsBodiesData.Length > _spatialHashMap.Capacity)
			{
				_spatialHashMap.Capacity = math.max(_spatialHashMap.Capacity * 2, _physicsBodiesData.Length + 256);
			}

			// 2. Build spatial hash
			var hashJob = new SpatialHash.HashBodiesJob
			{
				Bodies = _physicsBodiesData.AsArray(),
				CellSize = _spatialCellSize,
				SpatialMapWriter = _spatialHashMap.AsParallelWriter()
			};
			_propagateBodiesJobHandle = hashJob.Schedule(_physicsBodiesData.Length, 64, dependency); // Hash executes parallel across ranks

			// 3. Schedule propagation and collision detection jobs
			if (_integrationMethod == Integrator.VelocityVerlet)
			{
				var propagatePhysicsJob = new Integrators.IntegrateJobVelocityVerlet()
				{
					Time = Time.fixedTime,
					TimeStep = ScaledPhysicsTimeStep,
					CellSize = _spatialCellSize,
					GravitationalBodies = GravitySources.GravitationalBodies,
					GravitationalBodyIndices = GravitySources.GravitationalBodyIndices,
					SpatialMap = _spatialHashMap,
					CollisionTriggerMap = _isCollisionTriggerMap.AsArray(),
					CollidableMap = _isCollidableMap.AsArray(),
					CurrentBodies = _physicsBodiesData.AsArray(),
					NextBodies = _nextPhysicsBodiesData,
					CollisionQueue = _collisionQueue.AsParallelWriter()
				};
				_propagateBodiesJobHandle = propagatePhysicsJob.Schedule(_physicsBodiesData.Length, 32, _propagateBodiesJobHandle);
			}
			else
			{
				var propagatePhysicsJob = new Integrators.IntegrateJobRK4()
				{
					Time = Time.fixedTime,
					TimeStep = ScaledPhysicsTimeStep,
					CellSize = _spatialCellSize,
					GravitationalBodies = GravitySources.GravitationalBodies,
					GravitationalBodyIndices = GravitySources.GravitationalBodyIndices,
					SpatialMap = _spatialHashMap,
					CollisionTriggerMap = _isCollisionTriggerMap.AsArray(),
					CollidableMap = _isCollidableMap.AsArray(),
					CurrentBodies = _physicsBodiesData.AsArray(),
					NextBodies = _nextPhysicsBodiesData,
					CollisionQueue = _collisionQueue.AsParallelWriter()
				};
				_propagateBodiesJobHandle = propagatePhysicsJob.Schedule(_physicsBodiesData.Length, 32, _propagateBodiesJobHandle);
			}

			// 4. Resolve collisions (Single-threaded to prevent conflicting multi-body resolution)
			var resolveCollisionsJob = new CollisionResolution.ResolveCollisionsJob
			{
				TimeStep = ScaledPhysicsTimeStep,
				BreakSpeeds = _breakSpeeds.AsArray(),
				WarheadPayloadEnergies = _warheadPayloadEnergies.AsArray(),
				FractureRadiusThreshold = _fractureRadiusThreshold,
				CurrentBodies = _physicsBodiesData.AsArray(),
				NextBodies = _nextPhysicsBodiesData,
				CollisionQueue = _collisionQueue,
				BodiesToDestroy = _bodiesToDestroyQueue,
				BodiesToFracture = _bodiesToFractureQueue
			};
			_propagateBodiesJobHandle = resolveCollisionsJob.Schedule(_propagateBodiesJobHandle);

			// 5. Swap data collections
			var finalCopyJob = new CopyStatesJob
			{
				Source = _nextPhysicsBodiesData,
				Destination = _physicsBodiesData.AsArray()
			};
			_propagateBodiesJobHandle = finalCopyJob.Schedule(_physicsBodiesData.Length, 64, _propagateBodiesJobHandle);
			
			JobHandle.ScheduleBatchedJobs();
		}

		private void FinishPhysicsStep()
		{
			if (_physicsJobScheduled)
			{
				// Complete previous physics propagation step
				_propagateBodiesJobHandle.Complete();
				_physicsJobScheduled = false;

				// Process body destructions and fractures triggered during collision resolution
				while (_bodiesToFractureQueue.TryDequeue(out int index)) FractureBody(index);
				while (_bodiesToDestroyQueue.TryDequeue(out int index)) RemoveBody(index);

				ProcessRemovals();

				for (int i = 0; i < _physicsBodies.Count; i++)
				{
					PhysicsBody body = _physicsBodies[i];

					// Synchronize state and recalculate SoI.
					body.SyncStateFromManager(_physicsBodiesData[i]);
				}

				OnPhysicsUpdated?.Invoke();

				// Process deferred instantiations by decrementing delay timers
				for (int i = _delayedBodies.Count - 1; i >= 0; i--)
				{
					DelayedAddition db = _delayedBodies[i];
					db.RemainingDelay -= _currentTimeStep;
					
					// Queue additions if passed delay
					if (db.RemainingDelay <= 0f)
					{
						double integrationTime = -db.RemainingDelay;
						var updatedState = db.State;

						if (db.ReferenceBody != null && !db.ReferenceBody.IsDestroyed)
						{
							PhysicsBody parentSoI = db.ReferenceBody.ParentSoI;
							double mu = parentSoI != null ? OrbitalMath.GetStandardGravitationalParameter(parentSoI.Mass, db.ReferenceBody.Mass) : 0.0;

							// Evaluate exact reference body state at the moment the timer lapsed.
							// (RemainingDelay is negative, so this correctly steps backward in time from the current post-step frame)
							OrbitalMath.GetLocalStateAtTime(db.ReferenceBody, parentSoI, db.RemainingDelay, mu, out double2 refPosLaunch, out double2 refVelLaunch);
							refPosLaunch += parentSoI.Position;
							refVelLaunch += parentSoI.Velocity;
							
							if (parentSoI == null && math.lengthsq(db.ReferenceBody.Velocity) > 0)
							{
								// Linear fallback if no parent exists (e.g., primary central body)
								refPosLaunch = db.ReferenceBody.Position + db.ReferenceBody.Velocity * db.RemainingDelay;
								refVelLaunch = db.ReferenceBody.Velocity;
							}

							updatedState.Position = refPosLaunch;

							// Derive minimal collision clearance time to prevent immediate re-impact
							double relativeSpeed = math.length(updatedState.Velocity - refVelLaunch);
							double minTime = 0.0;
							if (relativeSpeed > 1e-5)
							{
								// Apply 1% padding to sum of radii to safeguard against floating point collision boundary penetration
								minTime = (db.ReferenceBody.Radius + db.Model.Radius) * 1.01 / relativeSpeed;
							}

							integrationTime = math.max(-db.RemainingDelay, minTime);
						}
						
						// Integrate projectile forward into current frame time, bypassing bounding box overlap.
						// Extracted from reference body block to ensure all additions account for frame delay.
						if (integrationTime > 0)
						{
							Integrators.VelocityVerlet(
								PhysicsBodyData.Stripped(updatedState, db.Model),
								GravitySources.GravitationalBodies,
								GravitySources.GravitationalBodyIndices,
								_simulationElapsedTime,
								(float)integrationTime,
								-1,
								out db.State
							);
						}
						else
						{
							db.State = updatedState;
						}
						
						_bodiesToAdd.Enqueue(new PendingAddition
						{
							Body = db.Body,
							State = db.State,
							Model = db.Model,
							CompositionIndex = db.CompositionIndex,
							IsCelestial = db.IsCelestial,
							Name = db.Name,
							WarheadEnergy = db.WarheadEnergy,
							EnableCollisionsAfterDelay = db.EnableCollisionsAfterDelay
						});
						_delayedBodies.RemoveAt(i);
					}
					else
					{
						_delayedBodies[i] = db; // Reassign modified struct back to list
					}
				}

				// Process pending collision activations
				for (int i = _collisionActivations.Count - 1; i >= 0; i--)
				{
					var activation = _collisionActivations[i];
					activation.RemainingDelay -= _currentTimeStep;

					if (activation.RemainingDelay <= 0f)
					{
						int idx = activation.Body.Index;
						_isCollidableMap[idx] = true;
						_collisionActivations.RemoveAt(i);
					}
					else
					{
						_collisionActivations[i] = activation;
					}
				}

				ProcessAdditions();

				_simulationElapsedTime += _currentTimeStep;
			}
		}

		private IEnumerator BoundsCheckRoutine()
		{
			while (true)
			{
				// Run independently every 60 seconds
				yield return new WaitForSeconds(60f);

				// Skip if system is empty or a check is already somehow active
				if (!_isInitialized || _physicsBodies.Count == 0 || _isBoundsCheckRunning) continue;

				// Wait until the main propagation job is finished to safely read _physicsBodiesData
				yield return new WaitUntil(() => !_physicsJobScheduled);

				_isBoundsCheckRunning = true;

				// Allocate with maximum possible capacity since the parallel writer uses AddNoResize
				_outOfBoundsIndices = new NativeList<int>(_physicsBodiesData.Length, Allocator.Persistent);

				var boundsJob = new CheckWithinBounds
				{
					Bodies = _physicsBodiesData.AsArray(),
					OutOfBoundsIndices = _outOfBoundsIndices.AsParallelWriter(),
					SimulationBounds = _simulationBounds
				};

				// Schedule as an independent background job (using batch size of 64)
				_boundsCheckHandle = boundsJob.Schedule(_physicsBodiesData.Length, 64);

				// Yield execution completely out of the main thread until the job finishes
				yield return new WaitUntil(() => _boundsCheckHandle.IsCompleted);
				
				_boundsCheckHandle.Complete();

				if (_outOfBoundsIndices.Length > 0)
				{
					// Sort descending. Higher indices are removed first so index shifting
					// doesn't invalidate subsequent removals if processed sequentially.
					_outOfBoundsIndices.Sort(); 
					
					for (int i = _outOfBoundsIndices.Length - 1; i >= 0; i--)
					{
						RemoveBody(_outOfBoundsIndices[i]);
					}
					
					Debug.Log($"[{GetType().Name}] Background bounds check complete. Queued {_outOfBoundsIndices.Length} out-of-bounds bodies for removal.");
				}

				_outOfBoundsIndices.Dispose();
				_isBoundsCheckRunning = false;
			}
		}

		private void ProcessRemovals()
		{
			// Process Removals
			if (_bodiesToRemove.Count > 0)
			{
				for (int i = 0; i < _bodiesToRemove.Count; i++)
				{
					PhysicsBody target = _bodiesToRemove[i];
					int index = _physicsBodies.IndexOf(target);
					
					if (index == -1) continue;

					string bodyName = target.Name;

					// Remove from managed and parallel native lists securely
					_physicsBodies.RemoveAt(index);
					_physicsBodyLookup.Remove(target.Name);
					_physicsBodiesData.RemoveAt(index);
					_breakSpeeds.RemoveAt(index);
					_warheadPayloadEnergies.RemoveAt(index);
					_isCelestialMap.RemoveAt(index);
					_isCollisionTriggerMap.RemoveAt(index);
					_isCollidableMap.RemoveAt(index);

					if (target.IsCelestial) GravitySources.RemoveGravitationalSource(index);

					for (int j = index; j < _physicsBodies.Count; j++)
					{
						_physicsBodies[j].DecrementIndex();
					}

					target.PostRemove();

					Debug.Log($"[{GetType().Name}] Removed physics body {bodyName} at original index {index}. New count: {_physicsBodies.Count}");
				}
				_bodiesToRemove.Clear();
			}
		}

		private void ProcessAdditions()
		{
			// Process Additions
			if (_bodiesToAdd.Count > 0)
			{
				int newBatchStartIndex = _physicsBodies.Count;

				while (_bodiesToAdd.Count > 0)
				{
					PendingAddition add = _bodiesToAdd.Dequeue();
					int index = _physicsBodies.Count;

					_physicsBodiesData.Add(new PhysicsBodyData(add.State, add.Model, float2.zero, 0f));
					_breakSpeeds.Add(_compositionProfiles[add.CompositionIndex].BreakSpeedThreshold);
					_warheadPayloadEnergies.Add(add.WarheadEnergy);
					_isCelestialMap.Add(add.IsCelestial);
					_isCollisionTriggerMap.Add(add.IsCollisionTrigger);
					_isCollidableMap.Add(add.IsCollidable);

					// Transition from pending shell to active simulation body
					add.Body.SetIndex(index);
					// Forces a sync which will automatically execute any mutations queued in _pendingOperations during the delay
					add.Body.SyncStateFromManager(_physicsBodiesData[index]);
					
					_physicsBodies.Add(add.Body);
					_physicsBodyLookup.Add(add.Body.Name, add.Body);

					if (add.Body.IsCelestial) GravitySources.AddGravitationalSource(index);

					if (add.EnableCollisionsAfterDelay)
					{
						_collisionActivations.Add(new CollisionActivation
						{
							Body = add.Body,
							RemainingDelay = 3.0 // 3 seconds relative grace period from spawn time
						});
					}

					Debug.Log($"[{GetType().Name}] Added physics body {add.Body.Name}. New count: {_physicsBodies.Count}");
				}

				// Evaluate hierarchy bounds strictly after the entire batch is integrated
				// into the master lists and GravitySources array.
				for (int i = newBatchStartIndex; i < _physicsBodies.Count; i++)
				{
					_physicsBodies[i].UpdateSoI();
				}
			}
		}

		private void ProcessPendingChanges()
		{
			// 1. Process Removals
			ProcessRemovals();

			// 2. Process Additions
			ProcessAdditions();
		}

		#endregion

		#region Cleaning

		private void OnDestroy()
		{
			Cleanup();
			if (Instance == this) Instance = null;
		}

		public void Cleanup()
		{
			Debug.Log($"[{GetType().Name}] Cleanup");

			EndAllAsync();

			_physicsBodies.Clear();
			_physicsBodyLookup.Clear();
			_bodiesToAdd.Clear();
			_delayedBodies.Clear();
			_bodiesToRemove.Clear();
			_collisionActivations.Clear();

			if (_physicsBodiesData.IsCreated) _physicsBodiesData.Dispose();
			if (_breakSpeeds.IsCreated) _breakSpeeds.Dispose();
			if (_warheadPayloadEnergies.IsCreated) _warheadPayloadEnergies.Dispose();
			if (_isCelestialMap.IsCreated) _isCelestialMap.Dispose();
			if (_isCollisionTriggerMap.IsCreated) _isCollisionTriggerMap.Dispose();
			if (_isCollidableMap.IsCreated) _isCollidableMap.Dispose();
			if (_nextPhysicsBodiesData.IsCreated) _nextPhysicsBodiesData.Dispose();
			if (_spatialHashMap.IsCreated) _spatialHashMap.Dispose();
			if (_collisionQueue.IsCreated) _collisionQueue.Dispose();
			if (_bodiesToDestroyQueue.IsCreated) _bodiesToDestroyQueue.Dispose();
			if (_bodiesToFractureQueue.IsCreated) _bodiesToFractureQueue.Dispose();

			GravitySources.Cleanup();

			_isInitialized = false;
		}

		public void ClearSimulation()
		{
			Debug.Log($"[{GetType().Name}] Simulation state clear starting");
			
			EndAllAsync();

			_physicsBodies.Clear();
			_physicsBodyLookup.Clear();
			_bodiesToAdd.Clear();
			_delayedBodies.Clear();
			_bodiesToRemove.Clear();
			_collisionActivations.Clear();
			if (_physicsBodiesData.IsCreated) _physicsBodiesData.Clear();
			if (_breakSpeeds.IsCreated) _breakSpeeds.Clear();
			if (_warheadPayloadEnergies.IsCreated) _warheadPayloadEnergies.Clear();
			if (_isCelestialMap.IsCreated) _isCelestialMap.Clear();
			if (_isCollisionTriggerMap.IsCreated) _isCollisionTriggerMap.Clear();
			if (_isCollidableMap.IsCreated) _isCollidableMap.Clear();
			_timeStepMultiplier = 1;
			_isPaused = true;
			_stepCount = 0;

			GravitySources.RemoveAllSources();

			Debug.Log($"[{GetType().Name}] Simulation state clear complete.");
		}

		private void EndAllAsync()
		{
			if (_boundsCheckCoroutine != null)
			{
				StopCoroutine(_boundsCheckCoroutine);
				_boundsCheckCoroutine = null;
			}

			if (_isBoundsCheckRunning)
			{
				_boundsCheckHandle.Complete();
				if (_outOfBoundsIndices.IsCreated) _outOfBoundsIndices.Dispose();
				_isBoundsCheckRunning = false;
			}
			
			if (_physicsJobScheduled)
			{
				_propagateBodiesJobHandle.Complete();
				_physicsJobScheduled = false;
			}
		}

		#endregion

		#region Public API

		public void SetTimeScale(float multiplier)
		{
			_timeStepMultiplier = math.max(0.1f, multiplier);
		}

		public void SetPaused(bool isPaused)
		{
			_isPaused = isPaused;
			OnSimulationPaused?.Invoke(isPaused);
		}

		public void TogglePaused()
		{
			if (_isPaused) SetPaused(false);
			else SetPaused(true);
		}

		public void SetForcePaused(bool isForcedPaused)
		{
			_isForcePaused = isForcedPaused;
		}

		public PhysicsBody AddBody(StateVector startState, VehicleModel vehicleModel, int compositionIndex, bool isCelestial, string name, bool isCollisionTrigger = true, bool isCollidable = true, double warheadEnergy = 0, double delay = 0f, PhysicsBody referenceBody = null)
		{
			// Pre-allocate the body with a pending index (-2) so it exists in memory immediately
			PhysicsBody newBody = new PhysicsBody(-2, startState, vehicleModel, compositionIndex, name, isCelestial);
			
			// If a physics job is currently scheduled, immediate additions must wait in the delay queue.
			// This ensures FinishPhysicsStep integrates them forward by the missed timestep, aligning them correctly.
			if (delay > 0f || _physicsJobScheduled)
			{
				_delayedBodies.Add(new DelayedAddition
				{
					Body = newBody,
					State = startState,
					ReferenceBody = referenceBody,
					Model = vehicleModel,
					CompositionIndex = compositionIndex,
					IsCelestial = isCelestial,
					IsCollisionTrigger = isCollisionTrigger,
					IsCollidable = false,
					Name = name,
					RemainingDelay = delay,
					WarheadEnergy = warheadEnergy,
					EnableCollisionsAfterDelay = isCollidable
				});
			}
			else
			{
				_bodiesToAdd.Enqueue(new PendingAddition
				{
					Body = newBody,
					State = startState,
					Model = vehicleModel,
					CompositionIndex = compositionIndex,
					IsCelestial = isCelestial,
					IsCollisionTrigger = isCollisionTrigger,
					IsCollidable = false,
					Name = name,
					WarheadEnergy = warheadEnergy,
					EnableCollisionsAfterDelay = isCollidable
				});
			}

			return newBody;
		}

		public void RemoveBody(int bodyIndex)
		{
			if (!_isInitialized || bodyIndex < 0 || bodyIndex >= _physicsBodies.Count)
			{
				Debug.LogError($"[{GetType().Name}] Cannot queue removal: Invalid index.");
				return;
			}

			Debug.Log($"{_physicsBodies[bodyIndex].Name} collision status: {_isCollidableMap[bodyIndex]}");

			PhysicsBody targetBody = _physicsBodies[bodyIndex];
			if (!_bodiesToRemove.Contains(targetBody))
			{
				_bodiesToRemove.Add(targetBody);
			}
		}

		/// <summary>
		/// Returns the PhysicsBodyData struct for a physics body index, IF propagation is not active.
		/// </summary>
		public bool TryGetBodyData(int bodyIndex, out PhysicsBodyData bodyData)
		{
			if (_physicsJobScheduled)
			{
				bodyData = default;
				return false;
			}

			if (_physicsBodiesData.IsCreated && bodyIndex >= 0 && bodyIndex < _physicsBodiesData.Length)
			{
				bodyData = _physicsBodiesData[bodyIndex];
				return true;
			}

			bodyData = default;
			return false;
		}

		public void UpdateBodyData(int index, in PhysicsBodyData newBodyData)
		{
			_physicsBodiesData[index] = newBodyData;
		}

		public void FractureBody(int bodyIndex)
		{
			FinishPhysicsStep();

			if (_fractureProfiles == null || _fractureProfiles.Count == 0)
			{
				// Destroy or disable the original body
				RemoveBody(bodyIndex);

				Debug.LogError($"[{GetType().Name}] Fracture profiles is null or empty. Destroying body instead");
				return;
			}

			PhysicsBodyData originalBodyData = _physicsBodiesData[bodyIndex];
			double bodyRadius = originalBodyData.VehicleModel.Radius;

			FractureProfile profile = null;

			for (int i = 0; i < _fractureProfiles.Count; i++)
			{
				if (bodyRadius >= _fractureProfiles[i].MinRadiusThreshold && bodyRadius < _fractureProfiles[i].MaxRadiusThreshold)
				{
					profile = _fractureProfiles[i];
				}
			}

			if (profile == null)
			{
				// Destroy or disable the original body
				RemoveBody(bodyIndex);

				Debug.Log($"[{GetType().Name}] {_physicsBodies[bodyIndex].Name} doesn't fall into any fracture profile. Destroying body instead");
				return;
			}

			Debug.Log($"[{GetType().Name}] Fracturing {_physicsBodies[bodyIndex].Name} with profile: {profile.ProfileName}");

			List<ChunkData> fractureData = BodyFractureUtility.CalculateFracture(
				originalBodyData.StateVector.Position, originalBodyData.StateVector.Velocity, originalBodyData.StateVector.AngularVelocity,
				originalBodyData.VehicleModel.Radius, originalBodyData.VehicleModel.Mass, profile, 0);
			
			int originalCompositionIndex = _physicsBodies[bodyIndex].CompositionIndex;

			for (int i = 0; i < fractureData.Count; i++)
			{
				// Instantiate new PhysicsBody
				var chunk = fractureData[i];
				var startState = new StateVector(chunk.Position, chunk.Velocity, 0, chunk.AngularVelocity);
				var vehicleModel = new VehicleModel(chunk.Mass, chunk.Radius);
				string newName = $"{_physicsBodies[bodyIndex].Name}_chunk_{i}";
				PhysicsBody chunkBody = AddBody(startState, vehicleModel, originalCompositionIndex, false, newName, isCollisionTrigger: false);

				OnSimulationOriginatedBody?.Invoke(chunkBody, _physicsBodies[bodyIndex]);
			}

			// Destroy or disable the parent body
			RemoveBody(bodyIndex);

			Debug.Log($"[{GetType().Name}] {_physicsBodies[bodyIndex].Name} (index = {bodyIndex}) fractured.");
		}

		public bool IsBodyFracturable(PhysicsBody body)
		{
			if (_fractureProfiles == null || _fractureProfiles.Count == 0) return false;

			FractureProfile profile = null;

			for (int i = 0; i < _fractureProfiles.Count; i++)
			{
				if (body.Radius >= _fractureProfiles[i].MinRadiusThreshold && body.Radius < _fractureProfiles[i].MaxRadiusThreshold)
				{
					profile = _fractureProfiles[i];
				}
			}

			if (profile) return true;
			else return false;
		}

		/// <summary>
		/// Safely executes a query delegate against the spatial hash map, waiting for active jobs to complete if necessary 
		/// to ensure data safety when queried outside the namespace or during physics execution.
		/// </summary>
		public void QuerySpatialHash(Action<NativeParallelMultiHashMap<int, int>, NativeArray<PhysicsBodyData>, double> queryAction)
		{
			if (_physicsJobScheduled)
			{
				_propagateBodiesJobHandle.Complete();
				_physicsJobScheduled = false;
			}

			if (_spatialHashMap.IsCreated && _physicsBodiesData.IsCreated)
			{
				queryAction(_spatialHashMap, _physicsBodiesData.AsArray(), _spatialCellSize);
			}
		}

		#endregion
	}

	[BurstCompile]
	public struct CopyStatesJob : IJobParallelFor
	{
		[ReadOnly] public NativeArray<PhysicsBodyData> Source;
		public NativeArray<PhysicsBodyData> Destination;

		public void Execute(int index)
		{
			Destination[index] = Source[index];
		}
	}

	[BurstCompile]
	public struct CheckWithinBounds : IJobParallelFor
	{
		[ReadOnly] public NativeArray<PhysicsBodyData> Bodies;
		[WriteOnly] public NativeList<int>.ParallelWriter OutOfBoundsIndices;
		public Bounds2D SimulationBounds;

		public void Execute(int index)
		{
			if (!SimulationBounds.IsWithinBounds(Bodies[index].StateVector.Position))
			{
				OutOfBoundsIndices.AddNoResize(index);
			}
		}
	}
}