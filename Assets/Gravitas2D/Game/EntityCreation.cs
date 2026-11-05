using UnityEngine;
using Gravitas2D;
using Unity.Mathematics;
using System.Collections.Generic;

namespace GravitasDemo
{
	public static class EntityCreation
	{
		public static bool Create(string name, VehicleModel vehicleModel, double2 spawnPos, double2 spawnVel, float orientation, float angVel, int compositionIndex, bool isOwned, EntityType type, BodySpawnOptions options, List<IEntityCapability> capabilities = null, IAutonomousTask initialTask = null, bool isKnown = false)
		{
			if (isOwned) isKnown = true;

			double warheadEnergy = 0;
			if (capabilities != null)
			{
				foreach (var capability in capabilities)
				{
					if (capability is WarheadCapability warheadCap)
					{
						warheadEnergy = warheadCap.WarheadEnergy;
						break;
					}
				}
			}

			if (PhysicsBodyCreation.CreateBody(name, vehicleModel, spawnPos, spawnVel, orientation, angVel, compositionIndex, warheadEnergy, out PhysicsBody physicsBody, options))
			{
				Entity entity;
				if (type == EntityType.Station) entity = new StationEntity(physicsBody, name, isOwned);
				else entity = new Entity(physicsBody, name, type, isOwned: isOwned, isKnown: isKnown);
				
				if (EntitiesManager.Instance.RegisterEntity(entity))
				{
					if (capabilities != null)
					{
						foreach (var capability in capabilities)
						{
							entity.AddCapability(capability);
						}
					}

					if (initialTask != null) entity.HandleNavigationTask(initialTask, false);

					Debug.Log($"[{nameof(EntityCreation)}] Successfully created entity {entity.Name}");
					return true;
				}
			}

			return false;
		}

		public static void CreateWithExistingBody(PhysicsBody physicsBody, PhysicsBody originalBody, bool isKnown = false)
		{
			Entity originatingEntity = EntitiesManager.Instance.EntityBodyLookup[originalBody];
			Entity entity = new Entity(physicsBody, physicsBody.Name, originatingEntity.Type, isOwned: false, isKnown: isKnown);
			if (EntitiesManager.Instance.RegisterEntity(entity))
			{
				Debug.Log($"[{nameof(EntityCreation)}] Successfully created entity {entity.Name}");
			}
		}
	}
}