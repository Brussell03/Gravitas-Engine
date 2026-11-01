using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace Gravitas2D
{
	public struct ChunkData
	{
		public double2 Position;
		public double2 Velocity;
		public float AngularVelocity;
		public double Mass;
		public double Radius;
	}

	public static class BodyFractureUtility
	{
		/// <summary>
		/// Shatters a body into mass-conserved, momentum-conserved chunks using a power-law size distribution.
		/// </summary>
		/// <param name="originalPosition">2D position of the original body.</param>
		/// <param name="originalVelocity">2D velocity of the original body.</param>
		/// <param name="originalAngularVelocity">Rotational velocity of the original body in radians per second.</param>
		/// <param name="originalRadius">Radius of the original body.</param>
		/// <param name="originalMass">Total mass to be distributed.</param>
		/// <param name="profile">Profile of fracture settings.</param>
		/// <param name="seed">Seed value for random generation.</param>
		/// <returns>List of calculated chunk data.</returns>
		public static List<ChunkData> CalculateFracture(
			double2 originalPosition,
			double2 originalVelocity,
			float originalAngularVelocity,
			double originalRadius,
			double originalMass,
			FractureProfile profile,
			uint seed)
		{
			if (profile.MinChunkRadiusPerc <= 0 || profile.MaxChunkRadiusPerc <= profile.MinChunkRadiusPerc || profile.MaxChunkRadiusPerc >= 1)
			{
				Debug.LogError($"[{nameof(PhysicsManager2D)}] Invalid FractureProfile: MinRadius must be > 0 and < MaxRadius.");
				return new List<ChunkData>();
			}

			Unity.Mathematics.Random rng = new Unity.Mathematics.Random(seed == 0 ? 1 : seed);
			List<ChunkData> chunks = new List<ChunkData>();

			// Volumetric Distribution (Spheres)
			double originalVolume = (4.0 / 3.0) * math.PI * math.pow(originalRadius, 3.0);
			double remainingVolume = originalVolume;
			double originalDensity = originalMass / originalVolume;

			double2 totalDebrisMomentum = double2.zero;
			double totalDebrisMass = 0.0;

			double minChunkRadius = profile.MinChunkRadiusPerc * originalRadius;
			double maxChunkRadius = profile.MaxChunkRadiusPerc * originalRadius;

			// 1. Generation Pass: Extract chunks until mass/volume is depleted
			while (remainingVolume > 0.0)
			{
				double u = rng.NextDouble();
				double skewedValue = math.pow(u, profile.DistributionExponent);
				
				double chunkRadius = minChunkRadius + (maxChunkRadius - minChunkRadius) * skewedValue;
				double chunkVolume = (4.0 / 3.0) * math.PI * math.pow(chunkRadius, 3.0);

				// Clamp final chunk volume to guarantee mass conservation
				if (chunkVolume > remainingVolume)
				{
					chunkVolume = remainingVolume;
					chunkRadius = math.pow((chunkVolume * 0.75) / math.PI, 1.0 / 3.0);
				}

				remainingVolume -= chunkVolume;
				double chunkMass = chunkVolume * originalDensity;

				// 2. Spatial Distribution: Place within original radius
				double angle = rng.NextDouble() * math.PI * 2.0;
				double distFromCenter = rng.NextDouble() * math.max(0, originalRadius - chunkRadius);
				
				double2 localOffset = new double2(math.cos(angle), math.sin(angle)) * distFromCenter;
				double2 chunkPosition = originalPosition + localOffset;

				// 3. Kinetic Energy Partitioning: Smaller chunks get ejected faster
				double2 radialDir = math.lengthsq(localOffset) > 0.0001 
					? math.normalize(localOffset) 
					: new double2(rng.NextDouble() - 0.5, rng.NextDouble() - 0.5);

				// Apply 2D rotation perturbation to radial vector
				double scatterAngle = (rng.NextDouble() * 2.0 - 1.0) * profile.DirectionScatter;
				double cosA = math.cos(scatterAngle);
				double sinA = math.sin(scatterAngle);
				double2 ejectionDir = math.normalize(new double2(
					radialDir.x * cosA - radialDir.y * sinA,
					radialDir.x * sinA + radialDir.y * cosA
				));

				// Tangential velocity from original body's rotation (v = w * r)
				double2 tangentialVelocity = new double2(
					-originalAngularVelocity * localOffset.y, 
					originalAngularVelocity * localOffset.x
				);

				// v = sqrt(2E/m). Simplified inversely proportional velocity scaling.
				double nominalSpeed = profile.EjectionEnergy / math.max(chunkRadius, 0.1);
				double speedVariationFactor = 1.0 + ((rng.NextDouble() * 2.0 - 1.0) * profile.EjectionEnergyVariation);
				double velocityMagnitude = math.max(0.0, nominalSpeed * speedVariationFactor);

				double2 chunkVelocity = originalVelocity + tangentialVelocity + (ejectionDir * velocityMagnitude);

				// Spin is highly volatile for small masses, stable for large cores
				double spinDirection = rng.NextBool() ? 1.0 : -1.0;
				float angularVelocity = (float)(spinDirection * (profile.SpinEnergy / math.max(chunkRadius, 0.1)) * rng.NextDouble());

				chunks.Add(new ChunkData
				{
					Position = chunkPosition,
					Velocity = chunkVelocity,
					AngularVelocity = angularVelocity,
					Radius = chunkRadius,
					Mass = chunkMass
				});

				totalDebrisMomentum += chunkVelocity * chunkMass;
				totalDebrisMass += chunkMass;

				// Failsafe for infinitesimal volume lock
				if (remainingVolume < (4.0 / 3.0) * math.PI * math.pow(minChunkRadius, 3.0))
				{
					break;
				}
			}

			// 4. Systemic Momentum Correction Pass
			// The barycenter of the debris must perfectly follow the original body's trajectory.
			double2 expectedOriginalMomentum = originalVelocity * originalMass;
			double2 momentumError = expectedOriginalMomentum - totalDebrisMomentum;
			
			// Distribute the correction velocity evenly across the system to nullify drift
			double2 systemicVelocityCorrection = momentumError / totalDebrisMass;

			for (int i = 0; i < chunks.Count; i++)
			{
				var chunk = chunks[i];
				chunk.Velocity += systemicVelocityCorrection;
				chunks[i] = chunk;
			}

			return chunks;
		}
	}
}