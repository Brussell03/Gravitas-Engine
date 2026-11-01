using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;

namespace Gravitas
{
	[BurstCompile]
	public static class GravitySources
	{
		public static double gravitationalConstant { get; private set; } = 6.6743e-11; // m^3 * kg^-1 * s^-2
		public static NativeArray<PhysicsBodyData> gravitationalBodies { get; private set; }
		public static NativeList<int> gravitationalBodiesPhysicsIndices { get; private set; }
		public static int amount { get; private set; }

		public static double CalculateGravitationalParameter(double mass)
		{
			return 6.6743e-11 * mass;
		}

		public static void Initialize()
		{
			gravitationalBodiesPhysicsIndices = new NativeList<int>(Allocator.Persistent);
			UpdateGravitationalBodiesArray();
		}
		
		public static void ScheduleStateUpdateJob(in NativeArray<PhysicsBodyData> physicsBodyData, out JobHandle updateStateJobHandle)
		{
			var updateStateJob = new UpdateGravitationalBodiesStatesJob
			{
				GravitationalBodies = gravitationalBodies,
				PhysicsBodyData = physicsBodyData,
				GravitationalBodiesPhysicsIndex = gravitationalBodiesPhysicsIndices
			};
			updateStateJobHandle = updateStateJob.Schedule();

			//return updateStateJobHandle;
		}
		
		public static void ScheduleStateUpdateJob(in NativeArray<PhysicsBodyData> physicsBodyData, in JobHandle dependsOn, out JobHandle updateStateJobHandle)
		{
			var updateStateJob = new UpdateGravitationalBodiesStatesJob
			{
				GravitationalBodies = gravitationalBodies,
				PhysicsBodyData = physicsBodyData,
				GravitationalBodiesPhysicsIndex = gravitationalBodiesPhysicsIndices
			};
			updateStateJobHandle = updateStateJob.Schedule(dependsOn);

			//return updateStateJobHandle;
		}

		[BurstCompile]
		public struct UpdateGravitationalBodiesStatesJob : IJob
		{
			[ReadOnly] public NativeArray<PhysicsBodyData> PhysicsBodyData;
			[ReadOnly] public NativeList<int> GravitationalBodiesPhysicsIndex;
			public NativeArray<PhysicsBodyData> GravitationalBodies;

			public void Execute()
			{
				for (int i = 0; i < GravitationalBodies.Length; i++)
				{
					GravitationalBodies[i] = PhysicsBodyData[GravitationalBodiesPhysicsIndex[i]];
				}
			}
		}

		public static void AddGravitationalSource(int index)
		{
			gravitationalBodiesPhysicsIndices.Add(index);
			amount = gravitationalBodiesPhysicsIndices.Length;

			UpdateGravitationalBodiesArray();
		}

		public static void RemoveGravitationalSource(int index)
		{
			NativeList<int> indices = gravitationalBodiesPhysicsIndices;
			
			indices.RemoveAt(indices.IndexOf(index));
			amount = indices.Length;

			for (int i = 0; i < indices.Length; i++)
			{
				if (indices[i] > index)
				{
					indices[i] -= 1;
				}
			}

			UpdateGravitationalBodiesArray();
		}

		static void UpdateGravitationalBodiesArray()
		{
			if (gravitationalBodies.IsCreated && gravitationalBodies.Length != amount)
			{
				gravitationalBodies.Dispose();
				gravitationalBodies = new NativeArray<PhysicsBodyData>(amount, Allocator.Persistent);
			}
			else
			{
				gravitationalBodies = new NativeArray<PhysicsBodyData>(amount, Allocator.Persistent);
			}
		}
		
		public static void Cleanup()
		{
			if (gravitationalBodies.IsCreated) gravitationalBodies.Dispose();
			if (gravitationalBodiesPhysicsIndices.IsCreated) gravitationalBodiesPhysicsIndices.Dispose();
		}
	}
}