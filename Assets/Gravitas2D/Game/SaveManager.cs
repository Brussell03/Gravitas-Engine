using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using Gravitas2D;
using UnityEngine;

namespace GravitasDemo
{
	public class SaveManager : MonoBehaviour
	{
		public static SaveManager Instance { get; private set; }

		[SerializeField] private string _systemFileName;

		public event Action<bool> OnLoad;
		public event Action OnReset;

		private List<EntitySnapshot> _entitySnapshots;

		private void Awake()
		{
			if (Instance != null && Instance != this)
			{
				Destroy(gameObject);
				return;
			}
			Instance = this;

			StartCoroutine(LoadingRoutine());
		}

		private IEnumerator LoadingRoutine()
		{
			// Infinite loop to execute spawning indefinitely
			while (true)
			{
				// Validate PhysicsManager existence before accessing properties
				if (PhysicsManager2D.Instance == null || PhysicsManager2D.Instance.IsInitialized || EntitiesManager.Instance == null || EntitiesManager.Instance.IsInitialized) yield return null;

				Debug.Log($"[{GetType().Name}] Loading save...");

				// Load in entities
				LoadSystemFile(_systemFileName);
				
				yield break;
			}
		}

		private void OnDestroy()
		{
			if (Instance == this) Instance = null;
		}

		public void ResetToFile()
		{
			LoadSystemFile(_systemFileName);
		}

		public void LoadSystemFile(string systemFileName)
		{
			EntitiesManager.Instance.ClearEntities();
			PhysicsManager2D.Instance.ClearSimulation();

			OnLoad?.Invoke(false);
			OnReset?.Invoke();

			if (string.IsNullOrEmpty(systemFileName))
			{
				Debug.LogError($"[{GetType().Name}] No Solar System file selected in PhysicsManager!");
				return;
			}

			string pathToJson = Path.Combine(Application.streamingAssetsPath, "Data", systemFileName + ".json");

			if (!File.Exists(pathToJson))
			{
				Debug.LogError($"[{GetType().Name}] File not found at path: {pathToJson}. Check StreamingAssets/Data folder.");
				return;
			}

			EntitiesLoader.LoadEntitiesFromJson(pathToJson);

			Debug.Log($"[{GetType().Name}] Loading {systemFileName} complete.");

			OnLoad?.Invoke(true);
		}

		public void Save()
		{
			_entitySnapshots = EntitySnapshotManager.CaptureSnapshot();
		}

		public void Load()
		{
			EntitySnapshotManager.RestoreSnapshot(_entitySnapshots);
			OnLoad?.Invoke(true);
		}
	}
}