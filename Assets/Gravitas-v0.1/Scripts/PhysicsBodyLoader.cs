using System;
using System.Collections.Generic;
using System.IO;
using Newtonsoft.Json;
using Unity.Mathematics;
using UnityEngine;

namespace Gravitas
{
	public static class PhysicsBodyLoader
	{
		public static void LoadBodiesFromJson(string filePath)
		{
			try
			{
				// --- Read JSON File ---
				string jsonString = File.ReadAllText(filePath);

				// --- Deserialize JSON Array ---
				//var options = new JsonSerializerOptions { PropertyNameCaseInsensitive = true };
				List<PhysicsBodyJsonData> jsonDataList = JsonConvert.DeserializeObject<List<PhysicsBodyJsonData>>(jsonString);

				if (jsonDataList == null)
				{
					Debug.LogError($"Failed to deserialize JSON from {filePath}.");
				}

				// --- Convert Deserialized Data to Simulation Structs ---
				foreach (var jsonData in jsonDataList)
				{
					// Convert VehicleModel
					VehicleModel vmod = new VehicleModel
					{
						mass = jsonData.vehicleModel.mass,
						// Convert list of lists to float3x3
						InertiaTensor = new float3x3(
							jsonData.vehicleModel.inertiaTensor[0][0], jsonData.vehicleModel.inertiaTensor[0][1], jsonData.vehicleModel.inertiaTensor[0][2],
							jsonData.vehicleModel.inertiaTensor[1][0], jsonData.vehicleModel.inertiaTensor[1][1], jsonData.vehicleModel.inertiaTensor[1][2],
							jsonData.vehicleModel.inertiaTensor[2][0], jsonData.vehicleModel.inertiaTensor[2][1], jsonData.vehicleModel.inertiaTensor[2][2]
						),
						linearDamping = new float3(jsonData.vehicleModel.linearDamping[0], jsonData.vehicleModel.linearDamping[1], jsonData.vehicleModel.linearDamping[2]),
						angularDamping = new float3(jsonData.vehicleModel.angularDamping[0], jsonData.vehicleModel.angularDamping[1], jsonData.vehicleModel.angularDamping[2])
					};
					vmod.Initialize(); // Calculate inverse inertia tensor

					// Convert StateVector
					StateVector state = new StateVector
					{
						position = new double3(jsonData.initialState.position[0], jsonData.initialState.position[1], jsonData.initialState.position[2]),
						velocity = new double3(jsonData.initialState.velocity[0], jsonData.initialState.velocity[1], jsonData.initialState.velocity[2]),
						orientation = new quaternion(jsonData.initialState.orientation[0], jsonData.initialState.orientation[1], jsonData.initialState.orientation[2], jsonData.initialState.orientation[3]),
						angularVelocity = new float3(jsonData.initialState.angularVelocity[0], jsonData.initialState.angularVelocity[1], jsonData.initialState.angularVelocity[2])
					};

					// Create the final PhysicsBodyData struct (assuming AtmosphericModel is handled elsewhere or default)
					// You might need to add 'amod' loading if it's also in the JSON
					var body = PhysicsManager.CreateBody(in state, vmod, jsonData.kinematic, jsonData.celestial, math.max(0.5, jsonData.radius), name: jsonData.bodyIdentifier);
					// new PhysicsBody(in state, vmod, jsonData.kinematic, jsonData.celestial, jsonData.radius, name: jsonData.bodyIdentifier);

					// Optionally store the identifier somewhere if needed
					// bodyData.Identifier = jsonData.bodyIdentifier;

				}

				for (int j = 0; j < 9; j++)
				{
					for (int i = 2; i < jsonDataList.Count; i++)
					{
						var jsonData = jsonDataList[i];
						// Convert VehicleModel
						VehicleModel vmod = new VehicleModel
						{
							mass = jsonData.vehicleModel.mass,
							// Convert list of lists to float3x3
							InertiaTensor = new float3x3(
								jsonData.vehicleModel.inertiaTensor[0][0], jsonData.vehicleModel.inertiaTensor[0][1], jsonData.vehicleModel.inertiaTensor[0][2],
								jsonData.vehicleModel.inertiaTensor[1][0], jsonData.vehicleModel.inertiaTensor[1][1], jsonData.vehicleModel.inertiaTensor[1][2],
								jsonData.vehicleModel.inertiaTensor[2][0], jsonData.vehicleModel.inertiaTensor[2][1], jsonData.vehicleModel.inertiaTensor[2][2]
							),
							linearDamping = new float3(jsonData.vehicleModel.linearDamping[0], jsonData.vehicleModel.linearDamping[1], jsonData.vehicleModel.linearDamping[2]),
							angularDamping = new float3(jsonData.vehicleModel.angularDamping[0], jsonData.vehicleModel.angularDamping[1], jsonData.vehicleModel.angularDamping[2])
						};
						vmod.Initialize(); // Calculate inverse inertia tensor

						// Convert StateVector
						StateVector state = new StateVector
						{
							position = new double3(jsonData.initialState.position[0], jsonData.initialState.position[1] + (j + 1) * 2, jsonData.initialState.position[2]),
							velocity = new double3(jsonData.initialState.velocity[0], jsonData.initialState.velocity[1], jsonData.initialState.velocity[2]),
							orientation = new quaternion(jsonData.initialState.orientation[0], jsonData.initialState.orientation[1], jsonData.initialState.orientation[2], jsonData.initialState.orientation[3]),
							angularVelocity = new float3(jsonData.initialState.angularVelocity[0], jsonData.initialState.angularVelocity[1], jsonData.initialState.angularVelocity[2])
						};

						// Create the final PhysicsBodyData struct (assuming AtmosphericModel is handled elsewhere or default)
						// You might need to add 'amod' loading if it's also in the JSON
						var body = PhysicsManager.CreateBody(in state, vmod, jsonData.kinematic, jsonData.celestial, math.max(0.5, jsonData.radius), name: jsonData.bodyIdentifier);
						//new PhysicsBody(in state, vmod, jsonData.kinematic, jsonData.celestial, jsonData.radius, name: jsonData.bodyIdentifier);

						// Optionally store the identifier somewhere if needed
						// bodyData.Identifier = jsonData.bodyIdentifier;

					}
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
			public List<float> orientation { get; set; } // Quaternion (x, y, z, w)
			public List<float> angularVelocity { get; set; }
		}

		// Match the "vehicleModel" object in JSON
		private class VehicleModelData
		{
			public float mass { get; set; }
			public List<List<float>> inertiaTensor { get; set; } // List of rows
			public List<float> linearDamping { get; set; }
			public List<float> angularDamping { get; set; }
		}

		// Match the top-level object for each body in the JSON array
		private class PhysicsBodyJsonData
		{
			public string bodyIdentifier { get; set; }
			public bool kinematic { get; set; }
			public bool celestial { get; set; }
			public double radius { get; set; }
			public VehicleModelData vehicleModel { get; set; }
			public InitialStateData initialState { get; set; }
		}
	}
}