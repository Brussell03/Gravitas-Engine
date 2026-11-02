using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Gravitas2D
{
	[BurstCompile]
	public static class SpatialHash
	{
		// Large prime numbers for minimizing hash collisions in unbounded space
		private const int HashPrime1 = 73856093;
		private const int HashPrime2 = 19349663;

		/// <summary>
		/// Calculates a 1D spatial hash from a 2D world position.
		/// </summary>
		[BurstCompile]
		public static int GetCellHash(in double2 position, double cellSize)
		{
			// Floor to handle negative coordinate grids correctly
			int cellX = (int)math.floor(position.x / cellSize);
			int cellY = (int)math.floor(position.y / cellSize);

			return (cellX * HashPrime1) ^ (cellY * HashPrime2);
		}

		/// <summary>
		/// Calculates a 1D spatial hash from integer cell coordinates.
		/// </summary>
		[BurstCompile]
		public static int GetCellHash(int cellX, int cellY)
		{
			return (cellX * HashPrime1) ^ (cellY * HashPrime2);
		}

		[BurstCompile]
		public struct HashBodiesJob : IJobParallelFor
		{
			[ReadOnly] public NativeArray<StateVector> BodyStateVectors;
			[ReadOnly] public double CellSize;
			
			[WriteOnly] public NativeParallelMultiHashMap<int, int>.ParallelWriter SpatialMapWriter;

			public void Execute(int index)
			{
				int hash = GetCellHash(BodyStateVectors[index].Position, CellSize);
				SpatialMapWriter.Add(hash, index);
			}
		}
	}
}