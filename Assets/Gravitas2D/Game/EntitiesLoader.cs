using System;
using System.Collections.Generic;
using System.IO;
using Gravitas2D;
using Newtonsoft.Json;
using Unity.Mathematics;
using UnityEngine;
using static GravitasDemo.Entity;

namespace GravitasDemo
{
	public static class EntitiesLoader
	{
		public static void LoadEntitiesFromJson(string filePath)
		{
			try
			{
				// --- Read JSON File ---
				string jsonString = File.ReadAllText(filePath);

				// --- Deserialize JSON Array ---
				List<EntityJsonData> jsonDataList = JsonConvert.DeserializeObject<List<EntityJsonData>>(jsonString);

				if (jsonDataList == null)
				{
					Debug.LogError($"Failed to deserialize JSON from {filePath}.");
				}

				// --- Convert Deserialized Data to Simulation Structs ---
				foreach (var jsonData in jsonDataList)
				{
					double mass = jsonData.vehicleModel.mass;
					int compositionIndex = jsonData.vehicleModel.compositionIndex;
					double radius = jsonData.vehicleModel.radius;
					bool owned = jsonData.owned;
					
					// Convert VehicleModel
					VehicleModel vehicleModel = mass == 0 ? VehicleModel.CreateFromComposition(radius, compositionIndex) : VehicleModel.CreateFromMass(radius, mass);

					double2 spawnPos = new double2(jsonData.initialState.position[0], jsonData.initialState.position[1]);
					double2 spawnVel = new double2(jsonData.initialState.velocity[0], jsonData.initialState.velocity[1]);
					float orientation = jsonData.initialState.orientation;
					float angularVelocity = jsonData.initialState.angularVelocity;

					// Parse heterogeneous capability array
					List<IEntityCapability> capabilities = new List<IEntityCapability>();
					if (jsonData.capabilities != null)
					{
						foreach (var capData in jsonData.capabilities)
						{
							string type = capData.type?.ToLowerInvariant();
							switch (type)
							{
								case "optical":
									capabilities.Add(new SensorCapability
									{
										Type = SensorType.Optical,
										Range = capData.range,
										FieldOfViewDegrees = capData.fieldOfViewDegrees > 0 ? capData.fieldOfViewDegrees : 360f,
										OrientationOffset = capData.orientationOffset,
										Sensitivity = capData.sensitivity > 0 ? capData.sensitivity : 1.0f
									});
									break;

								case "infrared":
								case "ir":
									capabilities.Add(new SensorCapability
									{
										Type = SensorType.Infrared,
										Range = capData.range,
										FieldOfViewDegrees = capData.fieldOfViewDegrees > 0 ? capData.fieldOfViewDegrees : 360f,
										OrientationOffset = capData.orientationOffset,
										Sensitivity = capData.sensitivity > 0 ? capData.sensitivity : 1.0f
									});
									break;

								case "radar":
									capabilities.Add(new SensorCapability
									{
										Type = SensorType.Radar,
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
										ReloadTime = capData.reloadTime,
										Ammo = capData.ammo
									});
									break;

								case "missile":
									capabilities.Add(new MissileCapability
									{
										ProjectileCompositionIndex = capData.projectileCompositionIndex,
										ProjectileRadius = capData.projectileRadius,
										FireSpeed = capData.fireSpeed,
										ReloadTime = capData.reloadTime,
										Ammo = capData.ammo
									});
									break;
								
								case "navigation":
									capabilities.Add(new NavigationCapability
									{
										MaxForce = capData.maxForce > 0 ? capData.maxForce : 10000f,
										ArrivalThreshold = capData.arrivalThreshold > 0 ? capData.arrivalThreshold : 10f,
										MaxTorque = capData.maxTorque > 0 ? capData.maxTorque : 1000f,
										TargetOrientation = Mathf.Deg2Rad * capData.targetOrientation
									});
									break;
								
								case "warhead":
									capabilities.Add(new WarheadCapability
									{
										WarheadEnergy = capData.warheadEnergy > 0 ? capData.warheadEnergy : 0.0
									});
									break;

								default:
									Debug.LogWarning($"[{nameof(EntitiesLoader)}] Unknown capability type: {capData.type}");
									break;
							}
						}
					}

					// Parse and manage EntityType from JSON configuration
					EntityType entityType = EntityType.Asteroid;
					if (!string.IsNullOrEmpty(jsonData.type))
					{
						string typeStr = jsonData.type.ToLowerInvariant();
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
								if (Enum.TryParse(jsonData.type, true, out EntityType parsedEnum))
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
					EntityCreation.Create(jsonData.bodyIdentifier, vehicleModel, spawnPos, spawnVel, orientation, angularVelocity, compositionIndex, owned, entityType, spawnOptions, capabilities);
				}
			}
			catch (FileNotFoundException)
			{
				Debug.LogError($"Error: JSON file not found at {filePath}");
			}
			catch (JsonException ex)
			{
				Debug.LogError($"Error parsing JSON file {filePath}: {ex.Message}");
			}
			catch (Exception ex)
			{
				Debug.LogError($"An unexpected error occurred while loading {filePath}: {ex.Message}");
			}
		}

		// Match the "initialState" object in JSON
		private class InitialStateData
		{
			public List<double> position { get; set; }
			public List<double> velocity { get; set; }
			public float orientation { get; set; }
			public float angularVelocity { get; set; }
		}

		// Match the "vehicleModel" object in JSON
		private class VehicleModelData
		{
			public double mass { get; set; }
			public int compositionIndex { get; set; }
			public double radius { get; set; }
		}

		public class CapabilityJsonData
		{
			public string type { get; set; }
			public double range { get; set; } = 1000000.0;
			public float fieldOfViewDegrees { get; set; } = 360f;
			public float orientationOffset { get; set; } = 0f;
			public float sensitivity { get; set; } = 1.0f;
			public float projectileRadius { get; set; }
			public int projectileCompositionIndex { get; set; }
			public float fireSpeed { get; set; }
			public float reloadTime { get; set; }
			public float maxTorque { get; set; }
			public float targetOrientation { get; set; }
			public float maxForce { get; set; }
			public float arrivalThreshold { get; set; }
			public double warheadEnergy { get; set; }
			public int ammo { get; set; }
		}

		// Match the top-level object for each body in the JSON array
		private class EntityJsonData
		{
			public string bodyIdentifier { get; set; }
			public bool owned { get; set; }
			public string type { get; set; }
			public VehicleModelData vehicleModel { get; set; }
			public InitialStateData initialState { get; set; }
			public List<CapabilityJsonData> capabilities { get; set; }
		}
	}
}