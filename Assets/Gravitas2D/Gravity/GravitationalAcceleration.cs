using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace Gravitas2D
{
	[BurstCompile]
	public static class GravitationalAcceleration
	{
		[BurstCompile]
		public static void CalculateGravitationalAcceleration(in double2 position, in NativeList<int> gravitationalBodyIndices, in NativeArray<GravitySourceData> gravitationalBodies, int physicsBodyIndex, out double2 gravity_w)
		{
			gravity_w = double2.zero;

			for (int i = 0; i < gravitationalBodyIndices.Length; i++)
			{
				if (gravitationalBodyIndices[i] == physicsBodyIndex) continue;

				var source = gravitationalBodies[i];

				double2 relativePosition_w = position - source.Position;
				double dist_from_source_sqr = math.lengthsq(relativePosition_w);

				// equivalent to dist > 1e-6
				if (dist_from_source_sqr > 1e-12)
				{
					double dist_from_source = math.sqrt(dist_from_source_sqr);
					
					double inv_r3 = 1.0 / (dist_from_source_sqr * dist_from_source);
					gravity_w += -relativePosition_w * (source.Mu * inv_r3);
				}
			}
		}

		[BurstCompile]
		public static void NoGravitationalAcceleration(in double2 position, in NativeList<int> gravitationalBodyIndices, in NativeArray<GravitySourceData> gravitationalBodies, int bodyIndex, out double2 gravity_w)
		{
			gravity_w = double2.zero;
		}
	}
}