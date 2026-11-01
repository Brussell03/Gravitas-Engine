using System.Collections;
using System.Collections.Generic;
using Gravitas2D;
using Unity.Mathematics;
using UnityEngine;

namespace GravitasDemo
{
	public class GameManager : MonoBehaviour
	{
		public static GameManager Instance { get; private set; }

		[Header("Pause Menu")]
		[SerializeField] private GameObject _pauseMenu;

		[Header("Targeting & Spawning")]
		[SerializeField] private string _targetName = "Earth";
		[SerializeField] private float _minSpawnInterval = 10f;
		[SerializeField] private float _maxSpawnInterval = 20f;
		[SerializeField] private float _minRadius = 100f;
		[SerializeField] private float _maxRadius = 10000f;
		[SerializeField] private List<int> _spawnCompositionIndices;
		
		[Header("Impactor Parameters")]
		[SerializeField] private float _minTimeToTarget = 3600f;
		[SerializeField] private float _maxTimeToTarget = 7200f;
		[SerializeField] private float _minImpactVelocity = 1000f;
		[SerializeField] private float _maxImpactVelocity = 5000f;

		[Header("Spawn Constraints")]
		[Tooltip("Minimum angle below the horizon (0 is tangent, 90 is straight down).")]
		[SerializeField, Range(0f, 89f)] private float _minImpactAngleDegrees = 20f;
		[Tooltip("Minimum absolute distance from the target center for the spawn position.")]
		[SerializeField] private float _minSpawnDistance = 50000000f;

		private Coroutine _spawnerCoroutine;
		private bool _isPaused = false;

		public bool IsPaused => _isPaused;

		private static int _numSpawned = 0;

		private void Start()
		{
			if (Instance != null && Instance != this)
			{
				Destroy(gameObject);
				return;
			}
			Instance = this;

			_pauseMenu.SetActive(false);
		}

		private void OnDestroy()
		{
			StopSpawningImpactors();

			if (Instance == this) Instance = null;
		}

		void Update()
		{
			if (Input.GetKeyDown(KeyCode.Escape))
			{
				if (_isPaused) ResumeGame();
				else PauseGame();
			}
		}

		/// <summary>
		/// Initiates the infinite spawning loop.
		/// </summary>
		public void StartSpawningImpactors()
		{
			StartSpawningImpactors(PhysicsManager2D.Instance.PhysicsBodyLookup[_targetName]);
		}

		/// <summary>
		/// Initiates the infinite spawning loop.
		/// </summary>
		private void StartSpawningImpactors(PhysicsBody target)
		{
			if (_spawnerCoroutine != null) StopSpawningImpactors();

			// Input validation for target existence
			if (target == null)
			{
				Debug.LogError($"[{GetType().Name}] Cannot start spawning: target PhysicsBody is null.");
				return;
			}

			// Input validation for intervals
			if (_minSpawnInterval < 0f || _maxSpawnInterval < _minSpawnInterval)
			{
				Debug.LogError($"[{GetType().Name}] Cannot start spawning: Invalid spawn intervals provided.");
				return;
			}

			// Terminate existing routine to prevent duplicate overlapping spawners
			StopSpawningImpactors();
			
			_spawnerCoroutine = StartCoroutine(SpawnImpactorRoutine(target));
		}

		/// <summary>
		/// Halts the infinite spawning loop.
		/// </summary>
		public void StopSpawningImpactors()
		{
			if (_spawnerCoroutine != null)
			{
				StopCoroutine(_spawnerCoroutine);
				_spawnerCoroutine = null;
			}
		}

		public void ToggleSpawningImpactors(bool enable)
		{
			if (enable) StartSpawningImpactors();
			else StopSpawningImpactors();
		}

		private IEnumerator SpawnImpactorRoutine(PhysicsBody target)
		{
			// Infinite loop to execute spawning indefinitely
			while (true)
			{
				float waitTime = UnityEngine.Random.Range(_minSpawnInterval, _maxSpawnInterval);
				
				// Validate PhysicsManager existence before accessing properties
				if (PhysicsManager2D.Instance == null) yield break;
				
				// Calculate the absolute simulation time threshold for the next spawn
				float targetSimTime = PhysicsManager2D.Instance.SimulationElapsedTime + waitTime;

				// Yield execution until the simulation clock surpasses the calculated target time
				yield return new WaitUntil(() =>
				{
					// Halt wait loop immediately if the physics manager is destroyed
					if (PhysicsManager2D.Instance == null) return true;
					
					return PhysicsManager2D.Instance.SimulationElapsedTime >= targetSimTime;
				});

				// Terminate the coroutine safely if the physics manager was destroyed during the wait
				if (PhysicsManager2D.Instance == null) yield break;

				// Re-validate target state after the wait time has passed
				if (target == null)
				{
					Debug.LogWarning($"[{GetType().Name}] Target destroyed during wait. Stopping spawner.");
					StopSpawningImpactors();
					yield break;
				}

				SpawnImpactor(target);
			}
		}

		public void SpawnImpactor(PhysicsBody target)
		{
			// Runtime validation to prevent execution on invalid targets
			if (target == null)
			{
				Debug.LogError($"[{GetType().Name}] SpawnImpactor failed: target is null.");
				return;
			}
			
			float radius = UnityEngine.Random.Range(_minRadius, _maxRadius);
			int spawnCompIndex = _spawnCompositionIndices[UnityEngine.Random.Range(0, _spawnCompositionIndices.Count)];
			VehicleModel asteroidModel = VehicleModel.CreateFromComposition(radius, spawnCompIndex);

			int maxAttempts = 10;
			bool validSpawnFound = false;
			
			double2 finalSpawnPos = double2.zero;
			double2 finalSpawnVel = double2.zero;
			double finalTimeToTarget = 0;

			// Bounded loop to enforce minimum spawn distance without blocking the main thread
			for (int i = 0; i < maxAttempts; i++)
			{
				// 1. Determine exact impact coordinate on the target's surface
				Vector2 rawDir = UnityEngine.Random.insideUnitCircle.normalized;
				if (rawDir.sqrMagnitude < 0.01f) rawDir = Vector2.up; // Failsafe against exactly (0,0)

				double2 surfaceNormal = new double2(rawDir.x, rawDir.y);
				double2 relativeSurfacePos = surfaceNormal * target.Radius;
				double2 absoluteSurfacePos = target.Position + relativeSurfacePos;
				double2 inwardNormal = -surfaceNormal;

				// 2. Enforce Horizon Angle Constraint using a 2D rotation matrix
				// A 90-degree angle to the inward normal is the horizon.
				// We restrict the random angle to a cone that excludes the horizon limits.
				float maxAngleFromNormal = 90f - _minImpactAngleDegrees;
				float randomAngleRad = UnityEngine.Random.Range(-maxAngleFromNormal, maxAngleFromNormal) * Mathf.Deg2Rad;

				double cosA = math.cos(randomAngleRad);
				double sinA = math.sin(randomAngleRad);

				double2 velocityDir = new double2(
					inwardNormal.x * cosA - inwardNormal.y * sinA,
					inwardNormal.x * sinA + inwardNormal.y * cosA
				);

				double velocityMagnitude = UnityEngine.Random.Range(_minImpactVelocity, _maxImpactVelocity);
				double2 relativeImpactVel = velocityDir * velocityMagnitude;
				double2 absoluteImpactVel = target.Velocity + relativeImpactVel;

				// 3. Define time of flight and temporal reversal
				double timeToTarget = UnityEngine.Random.Range(_minTimeToTarget, _maxTimeToTarget);
				double timeAgo = -timeToTarget; // Must be negative to propagate backwards

				// 4. Derive physical invariants and execute Universal Keplerian backward propagation
				double mu = OrbitalMath.GetStandardGravitationalParameter(target.Mass, asteroidModel.Mass);

				OrbitalMath.GetLocalStateAtTime(
					absoluteSurfacePos,
					absoluteImpactVel,
					target,
					timeAgo,
					mu,
					out double2 spawnPos,
					out double2 spawnVel
				);

				// 5. Validate Distance Constraint
				if (math.length(spawnPos) >= _minSpawnDistance)
				{
					validSpawnFound = true;
					finalSpawnPos = spawnPos + target.Position;
					finalSpawnVel = spawnVel + target.Velocity;
					finalTimeToTarget = timeToTarget;
					break;
				}
			}

			if (!validSpawnFound)
			{
				Debug.LogWarning($"[{GetType().Name}] Failed to find valid spawn location exceeding {_minSpawnDistance}m after {maxAttempts} attempts. Aborting this spawn cycle.");
				return;
			}

			// Instantiate body directly at derived historical coordinates
			EntityCreation.Create($"Impactor {++_numSpawned}", asteroidModel, finalSpawnPos, finalSpawnVel, 0, 0, spawnCompIndex, false, EntityType.Asteroid, BodySpawnOptions.Default);

			Debug.Log($"[{GetType().Name}] Spawned impactor at {finalSpawnPos} targeting {target.Name} with ToF {finalTimeToTarget}s.");
		}

		public void PauseGame()
		{
			_isPaused = true;

			PhysicsManager2D.Instance?.SetForcePaused(true);
			_pauseMenu.SetActive(true);
		}

		public void ResumeGame()
		{
			if (!_isPaused) return;

			_isPaused = false;
			PhysicsManager2D.Instance?.SetForcePaused(false);
			_pauseMenu.SetActive(false);
		}

		public void QuitGame()
		{
			// Actively exits the actual built application
			#if UNITY_STANDALONE
			Application.Quit();
			#endif

			// Exits play mode inside the Unity Editor safely
			#if UNITY_EDITOR
			UnityEditor.EditorApplication.isPlaying = false;
			#endif
		}

		public void SpeedUpGame()
		{
			var physMan = PhysicsManager2D.Instance;
			if (physMan == null) return;

			if (physMan.IsAtSpeed1) physMan.SetTimeScale(10);
			else if (physMan.IsAtSpeed2) physMan.SetTimeScale(100);
			else if (physMan.IsAtSpeed3) physMan.SetTimeScale(3600);
		}

		public void SpeedDownGame()
		{
			var physMan = PhysicsManager2D.Instance;
			if (physMan == null) return;

			if (physMan.IsAtSpeed2) physMan.SetTimeScale(1);
			else if (physMan.IsAtSpeed3) physMan.SetTimeScale(10);
			else if (physMan.IsAtSpeed4) physMan.SetTimeScale(100);
		}
	}
}