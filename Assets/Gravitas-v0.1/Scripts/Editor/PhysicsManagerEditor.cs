using UnityEngine;
using UnityEditor;
using System.IO;
using System.Linq;
using System;

namespace Gravitas
{
	[CustomEditor(typeof(PhysicsManager))]
	public class PhysicsManagerEditor : Editor {

		private string[] systemFileNames;
		private string dataPath;

		void OnEnable()
		{
			// Define the path to scan
			dataPath = Path.Combine(Application.dataPath, "StreamingAssets", "Data");
			// Get the file names when the inspector is enabled
			systemFileNames = GetJsonFileNames();
		}
		
		public override void OnInspectorGUI()
		{
			// Get the PhysicsManager instance we are inspecting
			PhysicsManager manager = (PhysicsManager)target;

			// Draw the 'Integration Method' enum field
			manager.integrationMethod = (PhysicsManager.Integrator)EditorGUILayout.EnumPopup("Integration Method", manager.integrationMethod);

			// --- Draw the Dynamic Dropdown ---
			
			if (systemFileNames == null || systemFileNames.Length == 0)
			{
				EditorGUILayout.HelpBox("No .json files found in Assets/StreamingAssets/Data", MessageType.Warning);
				// Optionally, refresh in case files were added
				if (GUILayout.Button("Rescan Files"))
				{
					systemFileNames = GetJsonFileNames();
				}
				return;
			}

			// Find the index of the currently selected file
			int currentIndex = Array.IndexOf(systemFileNames, manager.selectedSystemFile);

			// If not found (e.g., file deleted or first run), default to 0
			if (currentIndex < 0)
			{
				currentIndex = 0;
				manager.selectedSystemFile = systemFileNames[0]; // Set default
			}

			// Draw the popup (dropdown)
			int newIndex = EditorGUILayout.Popup("Solar System", currentIndex, systemFileNames);

			// If the user selected a different option, update the string
			if (newIndex != currentIndex)
			{
				Undo.RecordObject(manager, "Change Solar System");
				manager.selectedSystemFile = systemFileNames[newIndex];
				EditorUtility.SetDirty(manager); // Mark as changed
			}

			manager.bodyPrefab = (PhysicsBodyMono)EditorGUILayout.ObjectField("Body Prefab", manager.bodyPrefab, typeof(PhysicsBodyMono), false);
			manager.isSimulating = EditorGUILayout.Toggle("Simulate", manager.isSimulating);
			
			// Draw other default fields if you had any
			// DrawDefaultInspector(); 
		}

		/// <summary>
		/// Scans the StreamingAssets/Data folder for .json files.
		/// </summary>
		private string[] GetJsonFileNames()
		{
			if (!Directory.Exists(dataPath))
			{
				Debug.LogWarning($"BLine: Directory not found, creating: {dataPath}");
				Directory.CreateDirectory(dataPath);
				return new string[0];
			}

			// Get all files ending in .json, but not .meta files
			string[] filePaths = Directory.GetFiles(dataPath, "*.json");
			
			return filePaths
				.Select(path => Path.GetFileNameWithoutExtension(path))
				.ToArray();
		}
	}
}
