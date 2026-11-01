using UnityEngine;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using System;
using System.IO;
using Unity.Mathematics;
using System.Linq;

namespace Gravitas
{
	public class PhysicsManager : MonoBehaviour
	{
		public enum Integrator
		{
			RK4,
			VelocityVerlet
		}

		public Integrator integrationMethod;
		public string selectedSystemFile;
		public PhysicsBodyMono bodyPrefab;
		public bool isSimulating = true;
		public static event Action OnPhysicsUpdated;

		public static float physicsTimeStep;
		static NativeArray<PhysicsBodyData> physicsBodiesData;
		static List<PhysicsBody> physicsBodies;
		static List<PhysicsBody> kinematicPhysicsBodies;
		static List<PhysicsBodyData> kinematicBodiesData;
		public static Dictionary<Collider, PhysicsBody> colliderBodyMap;
		static NativeList<DynamicContactInfo> rawContactList;
		static NativeList<RaycastContext> raycastContexts;
		static PhysicsBody playerBody;
		public static double3 playerPos { get; private set; }

		static bool isInitialized = false;

		static JobHandle propagatePhysicsJobHandle;
		static FunctionPointer<Integrators.DerivativeFunctionPtr> derivativePtr;
		static FunctionPointer<Integrators.CalculateGravitationAcceleration> gravAccelPtr;
		static bool physicsJobScheduled = false;

		static SceneBodiesManager sceneBodiesManager;

		public void Awake()
		{
			if (!isInitialized)
			{
				Debug.Log("PhysicsManager: Performing first-time initialization...");

				physicsBodies = new List<PhysicsBody>();
				kinematicPhysicsBodies = new List<PhysicsBody>();
				kinematicBodiesData = new List<PhysicsBodyData>();

				colliderBodyMap = new Dictionary<Collider, PhysicsBody>();
				if (!rawContactList.IsCreated) rawContactList = new NativeList<DynamicContactInfo>(Allocator.Persistent);
				if (!raycastContexts.IsCreated) raycastContexts = new NativeList<RaycastContext>(Allocator.Persistent);

				sceneBodiesManager = new SceneBodiesManager(bodyPrefab);

				physicsTimeStep = Time.fixedDeltaTime;

				//GameObject.DontDestroyOnLoad(transform);

				GravitySources.Initialize();

				// Compile function pointers for Burst Jobs
				derivativePtr = BurstCompiler.CompileFunctionPointer<Integrators.DerivativeFunctionPtr>(EquationsOfMotion.CalculateDerivative);
				gravAccelPtr = BurstCompiler.CompileFunctionPointer<Integrators.CalculateGravitationAcceleration>(GravitationalAcceleration.CalculateGravitationalAcceleration);

				Application.quitting -= Cleanup;
				Application.quitting += Cleanup;

				// if (FloatingOrigin.Instance)
				// {
				// 	FloatingOrigin.Instance.PostFloatingOriginUpdate -= OffsetBodies;
				// 	FloatingOrigin.Instance.PostFloatingOriginUpdate += OffsetBodies;
				// }

				// Mark core setup as complete
				isInitialized = true;
				Debug.Log("PhysicsManager: Core Initialization Complete.");

				// Load in physics bodies
				LoadBodies();
				Debug.Log("PhysicsManager: Loading Bodies Complete.");

			}
		}

		void FixedUpdate()
		{
			// Don't update physics if system isn't initialized
			if (!isInitialized || physicsBodiesData.Length == 0) return;

			if (physicsJobScheduled)
			{
				// Complete previous physics propagation step
				propagatePhysicsJobHandle.Complete();

				rawContactList.Clear();
				raycastContexts.Clear();

				playerPos = playerBody.position;

				// Invoke post-update actions
				OnPhysicsUpdated?.Invoke();

				physicsJobScheduled = false;
			}
		}

		void Update()
		{
			// Start physics step if frame is a FixedUpdate frame
			if (isInitialized && physicsBodiesData.Length > 0 && !physicsJobScheduled && isSimulating)
			{
				physicsJobScheduled = true;
				PhysicsStep();
			}
		}

		[BurstCompile]
		void PhysicsStep()
		{
			// Update states of gravitational sources for next propagation step
			// var gravitySourcesUpdateJobHandle = GravitySources.ScheduleStateUpdateJob(physicsBodiesData);
			GravitySources.ScheduleStateUpdateJob(physicsBodiesData, out JobHandle gravitySourcesUpdateJobHandle);

			var collisionJobHandle = ScheduleProcessCollisions(gravitySourcesUpdateJobHandle);

			// Create propagation job
			if (integrationMethod == Integrator.VelocityVerlet)
			{
				var propagatePhysicsJob = new Integrators.IntegrateJobVelocityVerlet()
				{
					time = Time.fixedTime,
					timeStep = physicsTimeStep,
					derivativeFuncPtr = derivativePtr,
					gravAccelFuncPtr = gravAccelPtr,
					GravitationalBodies = GravitySources.gravitationalBodies,
					GravitationalBodiesPhysicsIndices = GravitySources.gravitationalBodiesPhysicsIndices,
					Bodies = physicsBodiesData
				};

				// Schedule propagation job
				propagatePhysicsJobHandle = propagatePhysicsJob.Schedule(physicsBodiesData.Length, 32, collisionJobHandle);
			}
			else if (integrationMethod == Integrator.RK4)
			{
				var propagatePhysicsJob = new Integrators.IntegrateJobRK4()
				{
					time = Time.fixedTime,
					timeStep = physicsTimeStep,
					derivativeFuncPtr = derivativePtr,
					gravAccelFuncPtr = gravAccelPtr,
					GravitationalBodies = GravitySources.gravitationalBodies,
					GravitationalBodiesPhysicsIndices = GravitySources.gravitationalBodiesPhysicsIndices,
					Bodies = physicsBodiesData
				};

				// Schedule propagation job
				propagatePhysicsJobHandle = propagatePhysicsJob.Schedule(physicsBodiesData.Length, 32, collisionJobHandle);
			}
			JobHandle.ScheduleBatchedJobs();
		}

		[BurstCompile]
		static JobHandle ScheduleProcessCollisions(JobHandle dependency)
		{
			// var raycastCommands = new NativeArray<RaycastCommand>(raycastContexts.Length, Allocator.TempJob);
			// var raycastResults = new NativeArray<RaycastHit>(raycastContexts.Length, Allocator.TempJob);
			// var contacts = new NativeArray<DynamicContactInfo>(raycastContexts.Length, Allocator.TempJob);

			// var raycastContextsArray = raycastContexts.AsArray();

			// var prepareRaycastsJob = new PrepareRaycastCommandJob
			// {
			// 	RaycastCommands = raycastCommands,
			// 	RaycastContexts = raycastContextsArray
			// };
			// var prepareRaycastsHandle = prepareRaycastsJob.Schedule(raycastCommands.Length, 32);

			// var raycastHandle = RaycastCommand.ScheduleBatch(raycastCommands, raycastResults, 32, prepareRaycastsHandle);

			// var processHitsJob = new ProcessRaycastHitsJob
			// {
			// 	RaycastHits = raycastResults,
			// 	RaycastContexts = raycastContextsArray,
			// 	playerPos = playerPos,
			// 	// ContactWriter = rawContactList.AsParallelWriter()
			// 	Contacts = contacts
			// };
			// var processHitsHandle = processHitsJob.Schedule(raycastResults.Length, 32, raycastHandle);
			// processHitsHandle.Complete();
			// Debug.Log(string.Join(", ", contacts.Select(r => r.penetrationDepth)));

			//JobHandle solverDependency = JobHandle.CombineDependencies(processHitsHandle, dependency);

			var dynamicSolverJob = new ProcessDynamicCollisionsJob
			{
				Bodies = physicsBodiesData,
				// Contacts = contacts,
				Contacts = rawContactList.AsArray(),
				restitution = 0.1,
				staticFriction = 0.6,
				dynamicFriction = 0.4,
				timeStep = Time.fixedDeltaTime
			};
			JobHandle dynamicSolverJobHandle = dynamicSolverJob.Schedule(dependency);
			
			// raycastCommands.Dispose(dynamicSolverJobHandle);
			// raycastResults.Dispose(dynamicSolverJobHandle);
			// contacts.Dispose(dynamicSolverJobHandle);

			return dynamicSolverJobHandle;
		}

		[BurstCompile]
		public static PhysicsBodyMono CreateBody(in StateVector startState, VehicleModel vmod, bool isKinematic, bool isCelestial, double radius = 1.0, string name = null)
		{
			return sceneBodiesManager.GetBody(startState, vmod, isKinematic, isCelestial, radius, name);
		}

		[BurstCompile]
		public static int AddBody(in PhysicsBody body, in StateVector startState, in VehicleModel vmod)
		{
			int index = physicsBodies.Count;
			physicsBodies.Add(body);
			// Add body data to array
			AppendBodyToArray(new PhysicsBodyData(startState, vmod, float3.zero, float3.zero));

			// Add gravitational source if body is a celestial
			if (body.isCelestial) GravitySources.AddGravitationalSource(index);

			Debug.Log($"PhysicsManager: Added physics body {body.name}. New count: {physicsBodies.Count}");
			return index;
		}

		[BurstCompile]
		public static int AddKinematicBody(in PhysicsBody body, in StateVector startState, in VehicleModel vmod)
		{
			int index = kinematicPhysicsBodies.Count;
			kinematicPhysicsBodies.Add(body);
			kinematicBodiesData.Add(new PhysicsBodyData(startState, vmod, float3.zero, float3.zero));

			Debug.Log($"PhysicsManager: Added kinematic body {body.name}. New count: {kinematicPhysicsBodies.Count}");
			return index;
		}

		[BurstCompile]
		public static void RemoveBody(int physicsBodyIndex)
		{
			if (!isInitialized)
			{
				Debug.LogError("Cannot remove body: Invalid index or array not initialized.");
				return;
			}

			string name = physicsBodies[physicsBodyIndex].name;
			physicsBodies.RemoveAt(physicsBodyIndex);

			// Remove body from arrays
			PopBodyAtIndex(physicsBodyIndex);

			// Remove gravitational source if body is a celestial
			if (physicsBodies[physicsBodyIndex].isCelestial) GravitySources.RemoveGravitationalSource(physicsBodyIndex);

			// Update physics bodies array reference index
			for (int i = physicsBodyIndex; i < physicsBodies.Count; i++)
			{
				physicsBodies[i].DecrementIndex();
			}

			Debug.Log($"PhysicsManager: Removed physics body {name} at index {physicsBodyIndex}. New count: {physicsBodies.Count}");
		}

		[BurstCompile]
		public static void RemoveKinematicBody(int physicsBodyIndex)
		{
			if (!isInitialized)
			{
				Debug.LogError("Cannot remove body: Invalid index or array not initialized.");
				return;
			}

			string name = kinematicPhysicsBodies[physicsBodyIndex].name;
			kinematicPhysicsBodies.RemoveAt(physicsBodyIndex);

			kinematicBodiesData.RemoveAt(physicsBodyIndex);

			// Update physics bodies array reference index
			for (int i = physicsBodyIndex; i < kinematicPhysicsBodies.Count; i++)
			{
				kinematicPhysicsBodies[i].DecrementIndex();
			}

			Debug.Log($"PhysicsManager: Removed kinematic body {name} at index {physicsBodyIndex}. New count: {kinematicPhysicsBodies.Count}");
		}
		
		[BurstCompile]
		public static int ConvertKinematicBodyToPhysicsBody(int physicsBodyIndex)
		{
			var body = kinematicPhysicsBodies[physicsBodyIndex];
			var bodyData = kinematicBodiesData[physicsBodyIndex];
			RemoveKinematicBody(physicsBodyIndex);
			return AddBody(body, bodyData.stateVector, bodyData.vmod);
		}
		
		[BurstCompile]
		public static int ConvertPhysicsBodyToKinematicBody(int physicsBodyIndex)
		{
			var body = physicsBodies[physicsBodyIndex];
			var bodyData = physicsBodiesData[physicsBodyIndex];
			RemoveBody(physicsBodyIndex);
			return AddKinematicBody(body, bodyData.stateVector, bodyData.vmod);
		}

		[BurstCompile]
		private static void AppendBodyToArray(in PhysicsBodyData newBodyData)
		{
			if (!physicsBodiesData.IsCreated || physicsBodiesData.Length == 0) // If first body or array was disposed/empty
			{
				// Dispose if somehow created but length 0
				if (physicsBodiesData.IsCreated) physicsBodiesData.Dispose();

				// Create a new array of size 1
				physicsBodiesData = new NativeArray<PhysicsBodyData>(1, Allocator.Persistent);
				physicsBodiesData[0] = newBodyData;
			}
			else // Array exists and has elements
			{
				int oldSize = physicsBodiesData.Length;

				// Create a new, larger array
				NativeArray<PhysicsBodyData> newArray = new NativeArray<PhysicsBodyData>(oldSize + 1, Allocator.Persistent);

				// Copy data from the old array to the new one
				NativeArray<PhysicsBodyData>.Copy(physicsBodiesData, newArray, oldSize);

				// Add the new element
				newArray[oldSize] = newBodyData;

				// Dispose the old array to prevent memory leaks
				physicsBodiesData.Dispose();

				// Replace the static reference with the new array
				physicsBodiesData = newArray;
			}
		}

		[BurstCompile]
		public static void PopBodyAtIndex(int index)
		{
			int oldSize = physicsBodiesData.Length;
			if (oldSize == 1) // Removing the last element
			{
				physicsBodiesData.Dispose();
				//bodiesData = new NativeArray<PhysicsBodyData>(0, Allocator.Persistent); // Create empty array
			}
			else
			{
				// Create smaller array
				NativeArray<PhysicsBodyData> newArray = new NativeArray<PhysicsBodyData>(oldSize - 1, Allocator.Persistent);

				// Copy elements before the removed index
				if (index > 0)
				{
					NativeArray<PhysicsBodyData>.Copy(physicsBodiesData, 0, newArray, 0, index);
				}
				// Copy elements after the removed index
				if (index < oldSize - 1)
				{
					NativeArray<PhysicsBodyData>.Copy(physicsBodiesData, index + 1, newArray, index, oldSize - 1 - index);
				}

				physicsBodiesData.Dispose();
				physicsBodiesData = newArray;
			}
		}

		[BurstCompile]
		public static void UpdateBodyData(int index, in PhysicsBodyData newBodyData, bool isKinematic)
		{
			if (isKinematic)
			{
				kinematicBodiesData[index] = newBodyData;
			}
			else
			{
				physicsBodiesData[index] = newBodyData;
			}
		}

		void LoadBodies()
		{
			if (string.IsNullOrEmpty(selectedSystemFile))
			{
				Debug.LogError("No Solar System file selected in PhysicsManager!");
				return;
			}

			string pathToJson = Path.Combine(Application.streamingAssetsPath, "Data", selectedSystemFile + ".json");

			if (!File.Exists(pathToJson))
			{
				Debug.LogError($"File not found at path: {pathToJson}. Check StreamingAssets/Data folder.");
				return;
			}

			PhysicsBodyLoader.LoadBodiesFromJson(pathToJson);
		}
		
		[BurstCompile]
		public static void SetPlayerBody(in PhysicsBody body)
		{
			playerBody = body;
			playerPos = body.position;
		}

		public static void RegisterCollision(int indexA, int indexB, in Collision collision, float penetration)
		{
			if (!rawContactList.IsCreated) return;

			for (int i = 0; i < collision.contactCount; i++)
			{
				ContactPoint contact = collision.GetContact(i);

				rawContactList.Add(new DynamicContactInfo
				{
					bodyIndexA = indexA,
					bodyIndexB = indexB,
					worldNormal = contact.normal,
					worldPoint = GetSimulationPosition(new double3(contact.point)),
					penetrationDepth = penetration
				});
			}
		}

		public static void RegisterContact(int indexA, int indexB, float3 contactPoint, float3 contactNormal, float penetrationDepth)
		{
			if (!rawContactList.IsCreated) return;

			rawContactList.Add(new DynamicContactInfo
			{
				bodyIndexA = indexA,
				bodyIndexB = indexB,
				worldNormal = contactNormal,
				worldPoint = GetSimulationPosition(new double3(contactPoint)),
				penetrationDepth = penetrationDepth
			});
		}

		public static void QueueRaycast(RaycastContext context)
		{
			raycastContexts.Add(context);
		}

		public static PhysicsBodyMono InstantiateBody(PhysicsBodyMono obj)
		{
			return Instantiate(obj);
		}

		public static void DestroyBody(GameObject obj)
		{
			Destroy(obj);
		}

		public static void Cleanup()
		{
			Debug.Log("PhysicsManager: Cleanup");
			propagatePhysicsJobHandle.Complete();
			Application.quitting -= Cleanup;

			physicsBodies.Clear();
			kinematicPhysicsBodies.Clear();
			kinematicBodiesData.Clear();

			if (physicsBodiesData.IsCreated) physicsBodiesData.Dispose();
			if (rawContactList.IsCreated) rawContactList.Dispose();
			if (raycastContexts.IsCreated) raycastContexts.Dispose();

			GravitySources.Cleanup();

			isInitialized = false;
		}

		[BurstCompile]
		public static StateVector GetBodyState(int physicsBodyIndex, bool isKinematic)
		{
			if (isKinematic)
			{
				return kinematicBodiesData[physicsBodyIndex].stateVector;
			}
			else
			{
				if (physicsBodiesData.IsCreated)
				{
					return physicsBodiesData[physicsBodyIndex].stateVector;
				}
			}

			return new StateVector();
		}

		[BurstCompile]
		public static PhysicsBodyData GetBodyData(int physicsBodyIndex, bool isKinematic)
		{
			if (isKinematic)
			{
				return kinematicBodiesData[physicsBodyIndex];
			}
			else
			{
				if (physicsBodiesData.IsCreated)
				{
					return physicsBodiesData[physicsBodyIndex];
				}
			}

			return new PhysicsBodyData();
		}

		[BurstCompile]
		public static PhysicsBody GetBodyFromCollider(Collider collider)
		{
			colliderBodyMap.TryGetValue(collider, out PhysicsBody body);
			return body;
		}

		[BurstCompile]
		public static double3 GetScenePosition(double3 position)
		{
			return position - playerPos;
		}

		[BurstCompile]
		public static double3 GetSimulationPosition(double3 position)
		{
			return position + playerPos;
		}
	}
}