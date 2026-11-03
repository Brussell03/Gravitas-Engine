using System;
using System.Collections.Generic;
using Gravitas2D;
using Unity.Mathematics;
using UnityEngine;
using static GravitasDemo.Entity;

namespace GravitasDemo
{
	[Serializable]
	public class EntitySnapshot
	{
		public string bodyIdentifier;
		public string entityType;
		public bool owned;
		public VehicleModelSnapshot vehicleModel;
		public InitialStateSnapshot initialState;
		public List<CapabilitySnapshot> capabilities;
	}

	[Serializable]
	public class VehicleModelSnapshot
	{
		public double mass;
		public int compositionIndex;
		public double radius;
	}

	[Serializable]
	public class InitialStateSnapshot
	{
		public double[] position;
		public double[] velocity;
		public float orientation;
		public float angularVelocity;
	}

	[Serializable]
	public class CapabilitySnapshot
	{
		public string type;
		public double range;
		public float fieldOfViewDegrees;
		public float orientationOffset;
		public float sensitivity;
		public float projectileRadius;
		public int projectileCompositionIndex;
		public float fireSpeed;
		public float reloadTime;
		public float maxTorque;
		public float targetOrientation;
		public float maxForce;
		public float arrivalThreshold;

		// Navigation active state & references
		public bool isNavigating;
		public bool isRendezvousMode;
		public bool isStationKeepingMode;
		public bool isOrbitMatchMode;
		public string targetEntityName;
		public double[] targetPosition;
		public string targetOrbitEntityName;
		public double[] targetOrbitPosition;
		public double[] targetOrbitVelocity;
	}

	public static class EntitySnapshotManager
	{
		private static string ConvertEntityTypeToString(EntityType type)
		{
			switch (type)
			{
				case EntityType.CelestialBody: return "celestial";
				case EntityType.Asteroid: return "asteroid";
				case EntityType.Vehicle: return "vehicle";
				case EntityType.Station: return "station";
				case EntityType.Missile: return "missile";
				case EntityType.Projectile: return "projectile";
				default: return type.ToString().ToLowerInvariant();
			}
		}

		/// <summary>
		/// Captures an in-memory snapshot of all active entities, their exact EntityTypes, physics states, and navigation configurations.
		/// </summary>
		public static List<EntitySnapshot> CaptureSnapshot()
		{
			var snapshots = new List<EntitySnapshot>();

			if (EntitiesManager.Instance == null || EntitiesManager.Instance.KnownEntities == null)
			{
				Debug.LogWarning($"[{nameof(EntitySnapshotManager)}] Cannot capture snapshot: EntitiesManager or KnownEntities is null.");
				return snapshots;
			}

			foreach (var entity in EntitiesManager.Instance.KnownEntities)
			{
				if (entity == null || entity.IsDestroyed) continue;

				var snapshot = new EntitySnapshot
				{
					bodyIdentifier = entity.Name,
					entityType = ConvertEntityTypeToString(entity.Type),
					owned = entity.IsOwned,
					vehicleModel = new VehicleModelSnapshot
					{
						mass = entity.Mass,
						radius = entity.Radius,
						compositionIndex = 0
					},
					initialState = new InitialStateSnapshot
					{
						position = new double[] { entity.Position.x, entity.Position.y },
						velocity = new double[] { entity.Velocity.x, entity.Velocity.y },
						orientation = entity.Orientation,
						angularVelocity = entity.AngularVelocity
					},
					capabilities = new List<CapabilitySnapshot>()
				};

				foreach (var cap in entity.GetCapabilities<IEntityCapability>())
				{
					if (cap is SensorCapability sensor)
					{
						snapshot.capabilities.Add(new CapabilitySnapshot
						{
							type = sensor.Type.ToString().ToLowerInvariant(),
							range = sensor.Range,
							fieldOfViewDegrees = sensor.FieldOfViewDegrees,
							orientationOffset = sensor.OrientationOffset,
							sensitivity = sensor.Sensitivity
						});
					}
					else if (cap is LaunchCapability launcher)
					{
						snapshot.capabilities.Add(new CapabilitySnapshot
						{
							type = "launch",
							projectileCompositionIndex = launcher.ProjectileCompositionIndex,
							projectileRadius = launcher.ProjectileRadius,
							fireSpeed = launcher.FireSpeed,
							reloadTime = launcher.ReloadTime
						});
					}
					else if (cap is MissileCapability missile)
					{
						snapshot.capabilities.Add(new CapabilitySnapshot
						{
							type = "missile",
							projectileCompositionIndex = missile.ProjectileCompositionIndex,
							projectileRadius = missile.ProjectileRadius,
							fireSpeed = missile.FireSpeed,
							reloadTime = missile.ReloadTime
						});
					}
					else if (cap is NavigationCapability nav)
					{
						snapshot.capabilities.Add(new CapabilitySnapshot
						{
							type = "navigation",
							maxForce = nav.MaxForce,
							arrivalThreshold = nav.ArrivalThreshold,
							maxTorque = nav.MaxTorque,
							targetOrientation = Mathf.Rad2Deg * nav.TargetOrientation,
							isNavigating = nav.IsNavigating,
							isRendezvousMode = nav.IsRendezvousMode,
							isStationKeepingMode = nav.IsStationKeepingMode,
							isOrbitMatchMode = nav.IsOrbitMatchMode,
							targetEntityName = nav.TargetEntity?.Name,
							targetPosition = new double[] { nav.TargetPosition.x, nav.TargetPosition.y },
							targetOrbitEntityName = nav.TargetOrbitEntity?.Name,
							targetOrbitPosition = new double[] { nav.TargetOrbitPositionOffset.x, nav.TargetOrbitPositionOffset.y },
							targetOrbitVelocity = new double[] { nav.TargetOrbitVelocity.x, nav.TargetOrbitVelocity.y }
						});
					}
				}

				snapshots.Add(snapshot);
			}

			Debug.Log($"[{nameof(EntitySnapshotManager)}] Captured snapshot containing {snapshots.Count} entities.");
			return snapshots;
		}

		/// <summary>
		/// Restores simulation state from snapshots using a two-pass approach to correctly resolve entity-to-entity navigation references.
		/// </summary>
		public static void RestoreSnapshot(List<EntitySnapshot> snapshots)
		{
			if (snapshots == null || snapshots.Count == 0)
			{
				Debug.LogWarning($"[{nameof(EntitySnapshotManager)}] Provided snapshot list is null or empty.");
				return;
			}

			if (EntitiesManager.Instance == null || PhysicsManager2D.Instance == null)
			{
				Debug.LogError($"[{nameof(EntitySnapshotManager)}] Cannot restore snapshot: EntitiesManager or PhysicsManager2D is missing.");
				return;
			}

			EntitiesManager.Instance.ClearEntities();
			PhysicsManager2D.Instance.ClearSimulation();

			// Store temporary tuples for Pass 2 entity reference linking
			var navigationLinkingData = new List<(NavigationCapability NavCap, string TargetName, string TargetOrbitName)>();

			// --- PASS 1: Instantiate all entities and capabilities ---
			foreach (var snap in snapshots)
			{
				double mass = snap.vehicleModel.mass;
				int compositionIndex = snap.vehicleModel.compositionIndex;
				double radius = snap.vehicleModel.radius;
				bool owned = snap.owned;

				VehicleModel vehicleModel = mass == 0 
					? VehicleModel.CreateFromComposition(radius, compositionIndex) 
					: VehicleModel.CreateFromMass(radius, mass);

				double2 spawnPos = new double2(snap.initialState.position[0], snap.initialState.position[1]);
				double2 spawnVel = new double2(snap.initialState.velocity[0], snap.initialState.velocity[1]);
				float orientation = snap.initialState.orientation;
				float angVel = snap.initialState.angularVelocity;

				List<IEntityCapability> capabilities = new List<IEntityCapability>();
				if (snap.capabilities != null)
				{
					foreach (var capData in snap.capabilities)
					{
						string type = capData.type?.ToLowerInvariant();
						switch (type)
						{
							case "optical":
							case "infrared":
							case "ir":
							case "radar":
								SensorType sType = SensorType.Optical;
								if (type == "infrared" || type == "ir") sType = SensorType.Infrared;
								else if (type == "radar") sType = SensorType.Radar;

								capabilities.Add(new SensorCapability
								{
									Type = sType,
									Range = capData.range,
									FieldOfViewDegrees = capData.fieldOfViewDegrees > 0 ? capData.fieldOfViewDegrees : 360f,
									OrientationOffset = capData.orientationOffset,
									Sensitivity = capData.sensitivity > 0 ? capData.sensitivity : 1.0f
								});
								break;

							case "launch":
								capabilities.Add(new LaunchCapability
								{
									ProjectileCompositionIndex = capData.projectileCompositionIndex,
									ProjectileRadius = capData.projectileRadius,
									FireSpeed = capData.fireSpeed,
									ReloadTime = capData.reloadTime
								});
								break;

							case "missile":
								capabilities.Add(new MissileCapability
								{
									ProjectileCompositionIndex = capData.projectileCompositionIndex,
									ProjectileRadius = capData.projectileRadius,
									FireSpeed = capData.fireSpeed,
									ReloadTime = capData.reloadTime
								});
								break;
							
							case "navigation":
								var navCap = new NavigationCapability
								{
									MaxForce = capData.maxForce > 0 ? capData.maxForce : 10000f,
									ArrivalThreshold = capData.arrivalThreshold > 0 ? capData.arrivalThreshold : 50f,
									MaxTorque = capData.maxTorque > 0 ? capData.maxTorque : 1000f,
									TargetOrientation = Mathf.Deg2Rad * capData.targetOrientation,
									IsNavigating = capData.isNavigating,
									IsRendezvousMode = capData.isRendezvousMode,
									IsStationKeepingMode = capData.isStationKeepingMode,
									IsOrbitMatchMode = capData.isOrbitMatchMode,
									TargetPosition = capData.targetPosition != null && capData.targetPosition.Length >= 2 
										? new double2(capData.targetPosition[0], capData.targetPosition[1]) 
										: double2.zero,
									TargetOrbitPositionOffset = capData.targetOrbitPosition != null && capData.targetOrbitPosition.Length >= 2 
										? new double2(capData.targetOrbitPosition[0], capData.targetOrbitPosition[1]) 
										: double2.zero,
									TargetOrbitVelocity = capData.targetOrbitVelocity != null && capData.targetOrbitVelocity.Length >= 2 
										? new double2(capData.targetOrbitVelocity[0], capData.targetOrbitVelocity[1]) 
										: double2.zero,
									IsEnabled = true
								};

								capabilities.Add(navCap);
								navigationLinkingData.Add((navCap, capData.targetEntityName, capData.targetOrbitEntityName));
								break;

							default:
								Debug.LogWarning($"[{nameof(EntitySnapshotManager)}] Unknown capability type during restore: {capData.type}");
								break;
						}
					}
				}

				// Parse exact EntityType properly
				EntityType entityType = EntityType.Asteroid;
				if (!string.IsNullOrEmpty(snap.entityType))
				{
					string typeStr = snap.entityType.ToLowerInvariant();
					switch (typeStr)
					{
						case "celestial":
						case "celestialbody":
							entityType = EntityType.CelestialBody;
							break;
						case "asteroid":
							entityType = EntityType.Asteroid;
							break;
						case "vehicle":
							entityType = EntityType.Vehicle;
							break;
						case "station":
							entityType = EntityType.Station;
							break;
						case "missile":
							entityType = EntityType.Missile;
							break;
						case "projectile":
							entityType = EntityType.Projectile;
							break;
						default:
							if (Enum.TryParse(snap.entityType, true, out EntityType parsedEnum))
							{
								entityType = parsedEnum;
							}
							else
							{
								if (capabilities.Count != 0) entityType = EntityType.Vehicle;
							}
							break;
					}
				}
				else
				{
					if (capabilities.Count != 0) entityType = EntityType.Vehicle;
				}

				var spawnOptions = entityType == EntityType.CelestialBody ? BodySpawnOptions.Celestial : BodySpawnOptions.Default;
				EntityCreation.Create(
					snap.bodyIdentifier,
					vehicleModel,
					spawnPos,
					spawnVel,
					orientation,
					angVel,
					compositionIndex,
					owned,
					entityType,
					spawnOptions,
					capabilities
				);
			}

			// --- PASS 2: Resolve entity references for NavigationCapabilities ---
			foreach (var link in navigationLinkingData)
			{
				if (!string.IsNullOrEmpty(link.TargetName) && EntitiesManager.Instance.EntityLookup.TryGetValue(link.TargetName, out var targetEnt))
				{
					link.NavCap.TargetEntity = targetEnt;
				}
				if (!string.IsNullOrEmpty(link.TargetOrbitName) && EntitiesManager.Instance.EntityLookup.TryGetValue(link.TargetOrbitName, out var targetOrbitEnt))
				{
					link.NavCap.TargetOrbitEntity = targetOrbitEnt;
				}
			}

			Debug.Log($"[{nameof(EntitySnapshotManager)}] Successfully restored {snapshots.Count} entities from snapshot.");
		}
	}
}