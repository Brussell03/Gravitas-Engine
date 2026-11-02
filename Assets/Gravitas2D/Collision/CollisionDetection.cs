using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

namespace Gravitas2D
{
	public struct CollisionEvent
	{
		public int BodyAIndex;
		public int BodyBIndex;
		public float TimeOfImpact;
		public double2 ContactPoint;
		public double2 Normal;
		public double2 RelativeVelocity;
	}

	[BurstCompile]
	public static class CollisionDetection
	{
		[BurstCompile]
		public static void EvaluateCollisions(int index, in StateVector stateVector, in VehicleModel vehicleModel, in StateVector nextState, in NativeArray<bool> collisionTriggerMap, in NativeArray<bool> collidableMap, float timeStep, double cellSize, in NativeArray<StateVector> bodyStateVectors, in NativeArray<VehicleModel> bodyVehicleModels, in NativeParallelMultiHashMap<int, int> spatialMap, ref NativeQueue<CollisionEvent>.ParallelWriter collisionQueue)
		{
			// Optimization: Only collidable objects initiate spatial grid queries.
			// This prevents non-collidable fragments from doing AABB math.
			if (!collisionTriggerMap[index] || !collidableMap[index]) return;

			double radius = vehicleModel.Radius;

			// Calculate swept AABB
			double minX = math.min(stateVector.Position.x, nextState.Position.x) - radius;
			double maxX = math.max(stateVector.Position.x, nextState.Position.x) + radius;
			double minY = math.min(stateVector.Position.y, nextState.Position.y) - radius;
			double maxY = math.max(stateVector.Position.y, nextState.Position.y) + radius;

			int minCellX = (int)math.floor(minX / cellSize);
			int maxCellX = (int)math.floor(maxX / cellSize);
			int minCellY = (int)math.floor(minY / cellSize);
			int maxCellY = (int)math.floor(maxY / cellSize);

			for (int cx = minCellX; cx <= maxCellX; cx++)
			{
				for (int cy = minCellY; cy <= maxCellY; cy++)
				{
					int hash = SpatialHash.GetCellHash(cx, cy);
					
					if (spatialMap.TryGetFirstValue(hash, out int otherIndex, out var iterator))
					{
						do
						{
							// Prevent self-collision
							if (otherIndex == index) continue;

							// Ensure the other body is also a valid collidable
							if (!collidableMap[otherIndex]) continue;

							// Deduplication: Only apply index comparison if BOTH objects are collidable.
							if (otherIndex < index) continue;

							StateVector otherStateVector = bodyStateVectors[otherIndex];

							if (SweptCircleIntersection(
								stateVector.Position, stateVector.Velocity, radius,
								otherStateVector.Position, otherStateVector.Velocity, bodyVehicleModels[otherIndex].Radius,
								timeStep, out float toi))
							{
								// Calculate exact contact parameters
								double2 posA_AtImpact = stateVector.Position + (stateVector.Velocity * toi);
								double2 posB_AtImpact = otherStateVector.Position + (otherStateVector.Velocity * toi);
								
								double2 relativePos = posB_AtImpact - posA_AtImpact;

								// Error handling: Fallback normal to prevent NaN when positions perfectly overlap
								double2 normal = math.lengthsq(relativePos) > 1e-12
									? math.normalize(relativePos) 
									: new double2(1, 0);
								
								double2 contactPoint = posA_AtImpact + normal * radius;
								double2 relativeVelocity = otherStateVector.Velocity - stateVector.Velocity;

								collisionQueue.Enqueue(new CollisionEvent
								{
									BodyAIndex = index,
									BodyBIndex = otherIndex,
									TimeOfImpact = toi,
									ContactPoint = contactPoint,
									Normal = normal,
									RelativeVelocity = relativeVelocity
								});
							}
						} while (spatialMap.TryGetNextValue(out otherIndex, ref iterator));
					}
				}
			}
		}
		
		/// <summary>
		/// Calculates exact time of impact for two moving circles using continuous collision detection.
		/// </summary>
		[BurstCompile]
		public static bool SweptCircleIntersection(
			in double2 posA, in double2 velA, double radiusA,
			in double2 posB, in double2 velB, double radiusB,
			float dt,
			out float timeOfImpact)
		{
			timeOfImpact = 0f;
			double2 pRel = posB - posA;
			double2 vRel = velB - velA;
			double rTotal = radiusA + radiusB;

			double a = math.lengthsq(vRel);
			double b = 2.0 * math.dot(pRel, vRel);
			double c = math.lengthsq(pRel) - (rTotal * rTotal);

			// Bodies are already overlapping at t=0
			if (c <= 0.0)
			{
				timeOfImpact = 0f;
				return true;
			}

			// Bodies are moving apart
			if (b > 0.0) return false;

			// Relative velocity is effectively zero, and they aren't overlapping
			if (a < 1e-8) return false;

			double discriminant = b * b - 4.0 * a * c;
			
			// No real roots, paths do not cross within radii
			if (discriminant < 0.0) return false;

			// Calculate the smallest positive root
			double t = (-b - math.sqrt(discriminant)) / (2.0 * a);

			// Valid collision only if it occurs within the current time step
			if (t >= 0.0 && t <= dt)
			{
				timeOfImpact = (float)t;
				return true;
			}

			return false;
		}
	}
}