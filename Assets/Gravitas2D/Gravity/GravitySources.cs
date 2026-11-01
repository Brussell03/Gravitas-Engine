using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Gravitas2D
{
	[StructLayout(LayoutKind.Sequential)]
	public struct GravitySourceData
	{
		public double2 Position;
		public double Mu; // Precomputed G * Mass
	}
	
	[BurstCompile]
	public static class GravitySources
	{
		private static double _gravitationalConstant = 6.6743e-11; // m^3 * kg^-1 * s^-2
		private static NativeArray<GravitySourceData> _gravitationalBodies;
		private static NativeList<int> _gravitationalBodyIndices;
		private static int _numGravitationalBodies;

		public static double GravitationalConstant => _gravitationalConstant;
		public static int NumGravitationalBodies => _numGravitationalBodies;
		public static NativeArray<GravitySourceData> GravitationalBodies => _gravitationalBodies;
		public static NativeList<int> GravitationalBodyIndices => _gravitationalBodyIndices;

		public static double CalculateGravitationalParameter(double mass)
		{
			return _gravitationalConstant * mass;
		}

		public static void Initialize()
		{
			_gravitationalBodyIndices = new NativeList<int>(Allocator.Persistent);
			UpdateGravitationalBodiesArray();
		}
		
		public static void ScheduleStateUpdateJob(in NativeArray<PhysicsBodyData> physicsBodyData, out JobHandle updateStateJobHandle)
		{
			var updateStateJob = new UpdateGravitationalBodyStatesJob
			{
				GravitationalBodies = _gravitationalBodies,
				PhysicsBodyData = physicsBodyData,
				GravitationalBodyIndices = _gravitationalBodyIndices,
				GravitationalConstant = _gravitationalConstant
			};
			updateStateJobHandle = updateStateJob.Schedule();
		}
		
		public static void ScheduleStateUpdateJob(in NativeArray<PhysicsBodyData> physicsBodyData, in JobHandle dependsOn, out JobHandle updateStateJobHandle)
		{
			var updateStateJob = new UpdateGravitationalBodyStatesJob
			{
				GravitationalBodies = _gravitationalBodies,
				PhysicsBodyData = physicsBodyData,
				GravitationalBodyIndices = _gravitationalBodyIndices,
				GravitationalConstant = _gravitationalConstant
			};
			updateStateJobHandle = updateStateJob.Schedule(dependsOn);
		}

		[BurstCompile]
		public struct UpdateGravitationalBodyStatesJob : IJob
		{
			[ReadOnly] public NativeArray<PhysicsBodyData> PhysicsBodyData;
			[ReadOnly] public NativeList<int> GravitationalBodyIndices;
			public NativeArray<GravitySourceData> GravitationalBodies;
			public double GravitationalConstant;

			public void Execute()
			{
				for (int i = 0; i < GravitationalBodies.Length; i++)
				{
					var data = PhysicsBodyData[GravitationalBodyIndices[i]];
					GravitationalBodies[i] = new GravitySourceData
					{
						Position = data.StateVector.Position,
						Mu = GravitationalConstant * data.VehicleModel.Mass
					};
				}
			}
		}

		public static void AddGravitationalSource(int index)
		{
			_gravitationalBodyIndices.Add(index);
			_numGravitationalBodies = _gravitationalBodyIndices.Length;

			UpdateGravitationalBodiesArray();
		}

		public static void RemoveGravitationalSource(int index)
		{
			_gravitationalBodyIndices.RemoveAt(_gravitationalBodyIndices.IndexOf(index));
			_numGravitationalBodies = _gravitationalBodyIndices.Length;

			for (int i = 0; i < _gravitationalBodyIndices.Length; i++)
			{
				if (_gravitationalBodyIndices[i] > index)
				{
					_gravitationalBodyIndices[i] -= 1;
				}
			}

			UpdateGravitationalBodiesArray();
		}

		static void UpdateGravitationalBodiesArray()
		{
			if (_gravitationalBodies.IsCreated)
			{
				if (_gravitationalBodies.Length == _numGravitationalBodies) return;
				_gravitationalBodies.Dispose();
			}
			_gravitationalBodies = new NativeArray<GravitySourceData>(_numGravitationalBodies, Allocator.Persistent);
		}

		public static void RemoveAllSources()
		{
			_gravitationalBodyIndices.Clear();
			_numGravitationalBodies = 0;
		}
		
		public static void Cleanup()
		{
			if (_gravitationalBodies.IsCreated) _gravitationalBodies.Dispose();
			if (_gravitationalBodyIndices.IsCreated) _gravitationalBodyIndices.Dispose();
		}
	}
}