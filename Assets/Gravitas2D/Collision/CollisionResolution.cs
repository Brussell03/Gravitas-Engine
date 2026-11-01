using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace Gravitas2D
{
	[StructLayout(LayoutKind.Sequential)]
	public struct CollisionOutcome
	{
		public double DeltaV;
		[MarshalAs(UnmanagedType.U1)] public bool IsDestroyed;
		[MarshalAs(UnmanagedType.U1)] public bool IsFractured;
	}

	[BurstCompile]
	public static class CollisionResolution
	{
		/// <summary>
		/// Computes the collision outcome and target response using first-principles impact mechanics.
		/// Shared between the real collision resolution job and the firing preview controller.
		/// </summary>
		[BurstCompile]
		public static void ComputeCollisionOutcome(
			double massA, double massB, in double2 velA, in double2 velB,
			double radiusB, double breakSpeedB,
			double warheadEnergyA, double warheadEnergyB,
			in double2 normal, float fractureRadiusThreshold, out CollisionOutcome outcome, float restitution = 0.1f)
		{
			outcome = new CollisionOutcome();
			double GravitationalConstant = 6.67430e-11;

			double invMassA = math.rcp(massA);
			double invMassB = math.rcp(massB);

			double2 relativeVelocity = velB - velA;

			double velAlongNormal = math.dot(relativeVelocity, normal);
			if (velAlongNormal > 0)
			{
				outcome.DeltaV = 0.0;
				outcome.IsDestroyed = false;
				outcome.IsFractured = false;
				return;
			}

			// Kinetic Enhancement via Vaporization / Beta-Factor
			double relSpeed = math.length(relativeVelocity);
			double beta = 1.0;
			double2 effectiveNormal = normal;
			if (relSpeed > 5000.0)
			{
				// Hypervelocity plasma/vapor plume momentum multiplier scaling with impact speed
				beta = 1.0 + math.min(5.0, (relSpeed - 5000.0) / 40000.0);

				// Excavated crater ejecta streams backward opposite to the impact vector,
				// skewing the impulse vector for realistic momentum enhancement.
				double2 impactDir = relSpeed > 1e-3 ? relativeVelocity / relSpeed : normal;
				double2 ejectaDir = math.normalize(-impactDir + normal);
				double blendFactor = math.min(0.6, (relSpeed - 5000.0) / 100000.0);
				effectiveNormal = math.normalize(math.lerp(normal, ejectaDir, blendFactor));
			}

			double baseJ = -(1.0 + restitution) * math.dot(relativeVelocity, effectiveNormal) * beta;

			double explosiveBonus = 0.0;
			if (warheadEnergyA > 0f)
			{
				explosiveBonus += math.sqrt(2.0 * massA * warheadEnergyA);
			}
			if (warheadEnergyB > 0f)
			{
				explosiveBonus += math.sqrt(2.0 * massB * warheadEnergyB);
			}

			double j = (baseJ + explosiveBonus) / (invMassA + invMassB);
			double2 impulse = j * normal;

			// Target (Body B) outcome evaluation
			double energyDeliveredToB = 0.5 * massA * relSpeed * relSpeed + warheadEnergyA;
			double specificEnergyB = energyDeliveredToB / math.max(1.0, massB);
			
			double deltaVB = math.length(impulse * invMassB);
			bool disintegrateB = false;

			// Gravitational self-binding energy disruption only applies to large self-gravitating bodies.
			// Smaller vehicles and objects rely strictly on structural break speed and material limits.
			if (radiusB >= fractureRadiusThreshold)
			{
				double bindingEnergyThresholdB = (GravitationalConstant * 3.0 * massB * massB) / (5.0 * math.max(1.0, radiusB));
				double specificBindingThresholdB = bindingEnergyThresholdB / math.max(1.0, massB);
				disintegrateB = specificEnergyB >= specificBindingThresholdB || deltaVB > breakSpeedB;
			}
			else
			{
				disintegrateB = deltaVB > breakSpeedB || warheadEnergyA > 0.0 || warheadEnergyB > 0.0;
			}

			outcome.DeltaV = deltaVB;
			if (disintegrateB)
			{
				if (radiusB >= fractureRadiusThreshold && deltaVB < breakSpeedB * 10)
				{
					outcome.IsFractured = true;
					outcome.IsDestroyed = false;
				}
				else
				{
					outcome.IsFractured = false;
					outcome.IsDestroyed = true;
				}
			}
			else
			{
				outcome.IsFractured = false;
				outcome.IsDestroyed = false;
			}

			return;
		}
		
		[BurstCompile]
		public struct ResolveCollisionsJob : IJob
		{
			[ReadOnly] public float TimeStep;
			[ReadOnly] public float FractureRadiusThreshold;
			
			[ReadOnly] public NativeArray<PhysicsBodyData> CurrentBodies;
			[ReadOnly] public NativeArray<float> BreakSpeeds;
			[ReadOnly] public NativeArray<double> WarheadPayloadEnergies;

			public NativeArray<PhysicsBodyData> NextBodies;
			
			[ReadOnly] public NativeQueue<CollisionEvent> CollisionQueue;
			public NativeQueue<int> BodiesToDestroy;
			public NativeQueue<int> BodiesToFracture;

			public void Execute()
			{
				if (CollisionQueue.IsEmpty()) return;

				NativeArray<CollisionEvent> collisions = CollisionQueue.ToArray(Allocator.Temp);
				collisions.Sort(new CollisionEventComparer());

				NativeArray<bool> resolvedBodies = new NativeArray<bool>(CurrentBodies.Length, Allocator.Temp);
				float restitution = 0.1f;
				const double GravitationalConstant = 6.67430e-11;

				for (int i = 0; i < collisions.Length; i++)
				{
					CollisionEvent col = collisions[i];
					int indexA = col.BodyAIndex;
					int indexB = col.BodyBIndex;

					if (resolvedBodies[indexA] || resolvedBodies[indexB]) continue;

					PhysicsBodyData bodyA = CurrentBodies[indexA];
					PhysicsBodyData bodyB = CurrentBodies[indexB];

					float breakSpeedA = BreakSpeeds[indexA];
					float breakSpeedB = BreakSpeeds[indexB];

					double massA = bodyA.VehicleModel.Mass;
					double massB = bodyB.VehicleModel.Mass;
					double invMassA = bodyA.VehicleModel.InvMass;
					double invMassB = bodyB.VehicleModel.InvMass;

					double2 vA = bodyA.StateVector.Velocity;
					double2 vB = bodyB.StateVector.Velocity;
					
					double2 normal = col.Normal;
					double2 relativeVelocity = vB - vA;

					double velAlongNormal = math.dot(relativeVelocity, normal);
					if (velAlongNormal > 0) continue;

					// Kinetic Enhancement via Vaporization / Beta-Factor
					double relSpeed = math.length(relativeVelocity);
					double beta = 1.0;
					double2 effectiveNormal = normal;
					if (relSpeed > 5000.0)
					{
						// Hypervelocity plasma/vapor plume momentum multiplier scaling with impact speed
						beta = 1.0 + math.min(5.0, (relSpeed - 5000.0) / 40000.0);

						// Excavated crater ejecta streams backward opposite to the impact vector,
						// skewing the impulse vector for realistic momentum enhancement.
						double2 impactDir = relSpeed > 1e-3 ? relativeVelocity / relSpeed : normal;
						double2 ejectaDir = math.normalize(-impactDir + normal);
						double blendFactor = math.min(0.6, (relSpeed - 5000.0) / 100000.0);
						effectiveNormal = math.normalize(math.lerp(normal, ejectaDir, blendFactor));
					}

					double baseJ = -(1.0 + restitution) * math.dot(relativeVelocity, effectiveNormal) * beta;

					// Explosive Warhead Payloads
					double payloadA = WarheadPayloadEnergies[indexA];
					double payloadB = WarheadPayloadEnergies[indexB];
					double explosiveBonus = 0.0;

					if (payloadA > 0f)
					{
						explosiveBonus += math.sqrt(2.0 * bodyA.VehicleModel.Mass * payloadA);
						BodiesToDestroy.Enqueue(indexA);
					}
					if (payloadB > 0f)
					{
						explosiveBonus += math.sqrt(2.0 * bodyB.VehicleModel.Mass * payloadB);
						BodiesToDestroy.Enqueue(indexB);
					}

					double j = (baseJ + explosiveBonus) / (invMassA + invMassB);

					double2 impulse = j * normal;
					double2 newVelA = vA - (impulse * invMassA);
					double2 newVelB = vB + (impulse * invMassB);

					// Gravitational Binding Energy & Specific Impact Energy Disruption
					double energyDeliveredToA = 0.5 * massB * relSpeed * relSpeed + payloadB;
					double energyDeliveredToB = 0.5 * massA * relSpeed * relSpeed + payloadA;

					double specificEnergyA = energyDeliveredToA / math.max(1.0, massA);
					double specificEnergyB = energyDeliveredToB / math.max(1.0, massB);

					double deltaVA = math.length(impulse * invMassA);
					double deltaVB = math.length(impulse * invMassB);

					bool disintegrateA = false;
					bool disintegrateB = false;

					if (bodyA.VehicleModel.Radius >= FractureRadiusThreshold)
					{
						double bindingEnergyThresholdA = (GravitationalConstant * 3.0 * massA * massA) / (5.0 * math.max(1.0, bodyA.VehicleModel.Radius));
						double specificBindingThresholdA = bindingEnergyThresholdA / math.max(1.0, massA);
						disintegrateA = specificEnergyA >= specificBindingThresholdA || deltaVA > breakSpeedA;
					}
					else
					{
						disintegrateA = deltaVA > breakSpeedA || payloadB > 0.0;
					}

					if (bodyB.VehicleModel.Radius >= FractureRadiusThreshold)
					{
						double bindingEnergyThresholdB = (GravitationalConstant * 3.0 * massB * massB) / (5.0 * math.max(1.0, bodyB.VehicleModel.Radius));
						double specificBindingThresholdB = bindingEnergyThresholdB / math.max(1.0, massB);
						disintegrateB = specificEnergyB >= specificBindingThresholdB || deltaVB > breakSpeedB;
					}
					else
					{
						disintegrateB = deltaVB > breakSpeedB || payloadA > 0.0;
					}

					if (disintegrateA)
					{
						if (bodyA.VehicleModel.Radius >= FractureRadiusThreshold && deltaVA < breakSpeedA * 10) BodiesToFracture.Enqueue(indexA);
						else BodiesToDestroy.Enqueue(indexA);
					}

					if (disintegrateB)
					{
						if (bodyB.VehicleModel.Radius >= FractureRadiusThreshold && deltaVB < breakSpeedB * 10) BodiesToFracture.Enqueue(indexB);
						else BodiesToDestroy.Enqueue(indexB);
					}

					float toi = col.TimeOfImpact;
					float remainingTime = TimeStep - toi;
					double remainingTimeD = (double)remainingTime;

					double2 posImpactA = bodyA.StateVector.Position + (vA * toi);
					double2 posImpactB = bodyB.StateVector.Position + (vB * toi);

					double2 finalPosA = posImpactA + (newVelA * remainingTimeD);
					double2 finalPosB = posImpactB + (newVelB * remainingTimeD);

					StateVector nextStateA = NextBodies[indexA].StateVector;
					StateVector nextStateB = NextBodies[indexB].StateVector;

					nextStateA.Velocity = newVelA;
					nextStateA.Position = finalPosA;
					nextStateB.Velocity = newVelB;
					nextStateB.Position = finalPosB;

					NextBodies[indexA] = new PhysicsBodyData(nextStateA, bodyA.VehicleModel, float2.zero, 0f);
					NextBodies[indexB] = new PhysicsBodyData(nextStateB, bodyB.VehicleModel, float2.zero, 0f);

					resolvedBodies[indexA] = true;
					resolvedBodies[indexB] = true;
				}

				resolvedBodies.Dispose();
				collisions.Dispose();
			}
		}

		private struct CollisionEventComparer : IComparer<CollisionEvent>
		{
			public int Compare(CollisionEvent x, CollisionEvent y)
			{
				return x.TimeOfImpact.CompareTo(y.TimeOfImpact);
			}
		}
	}
}