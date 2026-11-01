using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace Gravitas
{
	[BurstCompile]
	public static class GravitationalAcceleration
	{
		[BurstCompile]
		public static void CalculateGravitationalAcceleration(in double3 position, in NativeList<int> gravitationalBodiesPhysicsIndices, in NativeArray<PhysicsBodyData> gravitationalBodies, int physicsBodyIndex, out double3 gravity_w)
		{
			// Gravity (Round Earth) - acting towards planet center in world frame
			gravity_w = double3.zero;

			for (int i = 0; i < gravitationalBodiesPhysicsIndices.Length; i++)
			{
				if (gravitationalBodiesPhysicsIndices[i] == physicsBodyIndex) continue;

				var source = gravitationalBodies[i];

				double3 relativePosition_w = position - source.stateVector.position;

				double dist_from_source_sqr = math.lengthsq(relativePosition_w);
				double dist_from_source = math.sqrt(dist_from_source_sqr);

				double grav_mag = GravitySources.CalculateGravitationalParameter(source.vmod.mass) / dist_from_source_sqr;
				gravity_w += (dist_from_source > 1e-6) ? -math.normalize(relativePosition_w) * grav_mag : double3.zero;
			}
		}

		[BurstCompile]
		public static void NoGravitationalAcceleration(in double3 position, in NativeList<int> gravitationalBodiesPhysicsIndices, in NativeArray<PhysicsBodyData> gravitationalBodies, int physicsBodyIndex, out double3 gravity_w)
		{
			gravity_w = double3.zero;
		}
	}
}