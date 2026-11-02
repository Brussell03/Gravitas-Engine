using Gravitas2D;
using Unity.Mathematics;
using UnityEngine;
using System.Collections.Generic;
using UnityEngine.UI;
using TMPro;
using GravitasDemo.Rendering;

namespace GravitasDemo.Interaction
{
	public class SimulationFiringController : MonoBehaviour
	{
		public static SimulationFiringController Instance { get; private set; }

		[SerializeField] private float _lineWidth = 1.0f;
		[SerializeField] private Color _validColor = new Color(0.2f, 1f, 0.4f, 0.8f); // Green when hit is possible
		[SerializeField] private Color _invalidColor = new Color(1f, 0.3f, 0.2f, 0.8f); // Red/Orange when no hit solution
		[SerializeField] private Color _orbitMatchColor = new Color(0.8f, 0.2f, 1f, 0.6f); // Magenta for preview orbit
		[SerializeField] private Material _lineMaterial;
		[SerializeField] private RectTransform _lineContainer;

		private LineGraphic _fireLine;
		private LineGraphic _orbitPreviewLine;
		private Entity _firingEntity;
		private bool _wasPausedAtStart;
		private bool _isActive = false;
		private bool _isMissile;
		private Vector2[] _pathBuffer = new Vector2[180];
		private double _interceptTime;
		private bool _hasValidLock;

		private double _expectedDeltaV;
		private bool _targetDestroyed;
		private bool _targetFractured;

		private static int _numShotSpawned = 0;
		private static int _numMissileSpawned = 0;

		public bool IsActive => _isActive;
		public bool IsProjectileActive => _isActive && !_isMissile;
		public bool IsMissileActive => _isActive && _isMissile;

		void Start()
		{
			if (Instance != null && Instance != this)
			{
				Destroy(gameObject);
				return;
			}
			Instance = this;
		}

		void OnDestroy()
		{
			if (_fireLine != null) Destroy(_fireLine.gameObject);
			if (_orbitPreviewLine != null) Destroy(_orbitPreviewLine.gameObject);
			if (Instance == this) Instance = null;
		}

		public void StartProjectileFiringSequenceOfSelected()
		{
			StartFiringSequence(SimulationInteractionController.Instance.SelectedEntity, false);
		}

		public void StartMissileFiringSequenceOfSelected()
		{
			StartFiringSequence(SimulationInteractionController.Instance.SelectedEntity, true);
		}

		public void StartFiringSequence(Entity firingEntity, bool isMissile)
		{
			if (firingEntity == null) return;
			if (firingEntity.IsCelestial || (!firingEntity.IsLauncher && !firingEntity.IsMissileLauncher)) return;

			_isMissile = isMissile;

			// _wasPausedAtStart = PhysicsManager2D.Instance.IsPaused;
			// PhysicsManager2D.Instance.SetPaused(true);

			SimulationInteractionController.Instance.SetFiringMode();

			_firingEntity = firingEntity;
			_isActive = true;

			InitializeFireLine();
		}

		private void InitializeFireLine()
		{
			if (_fireLine == null)
			{
				_fireLine = LineGraphic.CreateLine("FiringVectorLine", _lineWidth, _invalidColor, _lineMaterial, LineType.Basic, _lineContainer);
			}
			else
			{
				_fireLine.gameObject.SetActive(true);
			}

			if (_orbitPreviewLine == null)
			{
				_orbitPreviewLine = LineGraphic.CreateLine("OrbitPreviewLine", _lineWidth, _orbitMatchColor, _lineMaterial, LineType.Trajectory, _lineContainer);
			}
			else
			{
				_orbitPreviewLine.gameObject.SetActive(true);
			}
		}

		public void AppendHoverIndicatorInfo(Entity hoveredEntity, ref string text, ref Color color)
		{
			color = _hasValidLock ? _validColor : _invalidColor;

			string outcomeStr = "Intact";
			if (_targetDestroyed) outcomeStr = "Destroyed";
			else if (_targetFractured) outcomeStr = "Fractured";

			text += $"\nFire Solution: {(_hasValidLock ? "Valid" : "Invalid")}" +
					(_hasValidLock ? $"\nFlight Time: {_interceptTime:F0} s" +
					$"\nExpected DeltaV: {_expectedDeltaV:F0} m/s" +
					$"\nTarget Outcome: {outcomeStr}" : "");
		}

		void LateUpdate()
		{
			if (!_isActive) return;
			if (_firingEntity == null || _firingEntity.IsDestroyed)
			{
				EndFiringSequence();
				return;
			}
			if (SimulationRenderer.Instance == null || SimulationRenderer.Instance.VisualizationArea == null) return;

			// Right-click to cancel/exit firing mode
			if (Input.GetMouseButtonDown(1))
			{
				EndFiringSequence();
				return;
			}

			if (!SimulationRenderer.Instance.TryGetMouseViewPosition(out Vector2 localUIPos)) return;

			// Hover lock: Check if cursor is over a valid target entity and snap endpoint if applicable
			// Entity targetEntity = SimulationInteractionController.Instance.FindClosestEntityAtUIPosition(localUIPos);
			Entity targetEntity = SimulationInteractionController.Instance.HoveredEntity;
			Vector2 targetUIPos = localUIPos;
			Vector2 entityUIPos = SimulationRenderer.Instance.SimPositionToViewUI(_firingEntity.Position);

			_hasValidLock = false;
			bool showOrbitPreview = false;
			_interceptTime = 0;
			_expectedDeltaV = 0;
			_targetDestroyed = false;
			_targetFractured = false;

			if (targetEntity != null && targetEntity != _firingEntity)
			{
				targetUIPos = SimulationRenderer.Instance.SimPositionToViewUI(targetEntity.Position);

				if (_isMissile)
				{
					// Guided missiles can lock onto any target entity
					if (_firingEntity.IsMissileLauncher && _firingEntity.MissileCapability.Ammo > 0)
					{
						PhysicsBody parent = _firingEntity.PhysicsBody != null ? _firingEntity.PhysicsBody.ParentSoI : null;
						MissileCapability launcher = _firingEntity.MissileCapability;
						VehicleModel model = VehicleModel.CreateFromComposition(launcher.ProjectileRadius, launcher.ProjectileCompositionIndex);
						double muMissile = OrbitalMath.GetStandardGravitationalParameter(parent.Mass, model.Mass);

						_interceptTime = NavigationCapability.ComputeInterceptTimeOfFlight(_firingEntity.Position, _firingEntity.Velocity, targetEntity.Position, targetEntity.Velocity, model.Mass, NavigationCapability.MaxMissileForce, muMissile, parent);
						_hasValidLock = true;

						OrbitalMath.GetLocalStateAtTime(targetEntity.Position, targetEntity.Velocity, parent, _interceptTime, targetEntity.PhysicsBody.GravParam, out _, out double2 targetVelAtIntercept);

						List<double2> path = NavigationCapability.PredictInterceptPath(_firingEntity.Position, _firingEntity.Velocity, targetEntity.Position, targetEntity.Velocity, model.Mass, NavigationCapability.MaxMissileForce, muMissile, _interceptTime, _firingEntity.DominantEntity.PhysicsBody, out double2 missileVelAtIntercept, steps: 200);

						double2 normalAtIntercept = -math.normalize(targetVelAtIntercept - missileVelAtIntercept);

						CollisionResolution.ComputeCollisionOutcome(
							model.Mass, targetEntity.Mass, missileVelAtIntercept, targetVelAtIntercept,
							targetEntity.Radius, targetEntity.Composition.BreakSpeedThreshold, WarheadCapability.MissileEnergy, 0.0, normalAtIntercept, PhysicsManager2D.Instance.FractureRadiusThreshold, out CollisionOutcome outcome
						);

						_expectedDeltaV = outcome.DeltaV;
						_targetDestroyed = outcome.IsDestroyed;
						_targetFractured = outcome.IsFractured;

						if (path != null && path.Count > 0 && _orbitPreviewLine != null)
						{
							showOrbitPreview = true;
							if (_pathBuffer == null || _pathBuffer.Length < path.Count)
							{
								_pathBuffer = new Vector2[path.Count];
							}

							Bounds2D effectiveBounds = SimulationRenderer.Instance.ViewBounds;
							float rectWidth = SimulationRenderer.Instance.VisualizationArea.rect.width;
							float rectHeight = SimulationRenderer.Instance.VisualizationArea.rect.height;

							_orbitPreviewLine.gameObject.SetActive(true);
							_orbitPreviewLine.RenderPathToGraphic(path, _orbitPreviewLine, _pathBuffer, effectiveBounds, rectWidth, rectHeight);
						}
					}
				}
				else
				{
					// Projectile interceptors require a solved analytical interception trajectory
					if (_firingEntity.IsLauncher && _firingEntity.LaunchCapability.Ammo > 0)
					{
						LaunchCapability launcher = _firingEntity.LaunchCapability;
						VehicleModel model = VehicleModel.CreateFromComposition(launcher.ProjectileRadius, launcher.ProjectileCompositionIndex);
						PhysicsBody parent = _firingEntity.PhysicsBody != null ? _firingEntity.PhysicsBody.ParentSoI : null;

						if (parent != null && OrbitalMath.SolveInterceptionVelocity(_firingEntity.PhysicsBody, targetEntity.PhysicsBody, launcher.FireSpeed, out double2 interceptVelocity, out _interceptTime, projectileMass: model.Mass, maxTime: 3600 * 24f, coarseSteps: 100))
						{
							_hasValidLock = true;

							// Derive exact state of target and projectile at the intercept time using OrbitalMath.GetLocalStateAtTime
							double muProject = OrbitalMath.GetStandardGravitationalParameter(parent.Mass, model.Mass);
							OrbitalMath.GetLocalStateAtTime(targetEntity.Position, targetEntity.Velocity, parent, _interceptTime, targetEntity.PhysicsBody.GravParam, out _, out double2 targetVelAtIntercept);
							OrbitalMath.GetLocalStateAtTime(_firingEntity.Position, interceptVelocity, parent, _interceptTime, muProject, out _, out double2 projVelAtIntercept);

							double2 normalAtIntercept = -math.normalize(targetVelAtIntercept - projVelAtIntercept);

							CollisionResolution.ComputeCollisionOutcome(
								model.Mass, targetEntity.Mass, projVelAtIntercept, targetVelAtIntercept,
								targetEntity.Radius, targetEntity.Composition.BreakSpeedThreshold, 0.0, 0.0, normalAtIntercept, PhysicsManager2D.Instance.FractureRadiusThreshold, out CollisionOutcome outcome
							);

							_expectedDeltaV = outcome.DeltaV;
							_targetDestroyed = outcome.IsDestroyed;
							_targetFractured = outcome.IsFractured;

							// Compute projectile's orbit path up to the interception point
							double2 relPos = _firingEntity.Position - parent.Position;
							double2 relVel = interceptVelocity - parent.Velocity;

							int resolution = 180;
							double2[] orbitPoints = OrbitalMath.CalculateTwoBodyOrbit(relPos, relVel, parent.Position, muProject, _interceptTime, resolution);

							if (orbitPoints != null && orbitPoints.Length > 0 && _orbitPreviewLine != null)
							{
								showOrbitPreview = true;
								if (_pathBuffer == null || _pathBuffer.Length < resolution)
								{
									_pathBuffer = new Vector2[resolution];
								}

								Bounds2D effectiveBounds = SimulationRenderer.Instance.ViewBounds;
								float rectWidth = SimulationRenderer.Instance.VisualizationArea.rect.width;
								float rectHeight = SimulationRenderer.Instance.VisualizationArea.rect.height;

								_orbitPreviewLine.gameObject.SetActive(true);
								_orbitPreviewLine.RenderPathToGraphic(orbitPoints, _orbitPreviewLine, _pathBuffer, effectiveBounds, rectWidth, rectHeight);
							}
						}
					}
				}
			}

			// Clear orbit preview if not showing an active intercept preview
			if (!showOrbitPreview && _orbitPreviewLine != null)
			{
				_orbitPreviewLine.SetPoints(null);
				_orbitPreviewLine.gameObject.SetActive(false);
			}

			// Update direct fire line vector and lock indicator and color based on lock/hit feasibility
			Color markerColor = _hasValidLock ? _validColor : _invalidColor;
			if (_fireLine != null)
			{
				if (_fireLine.color != markerColor) _fireLine.color = markerColor;
				_fireLine.SetPoints(new Vector2[] { entityUIPos, targetUIPos });
			}

			// Left-click to fire at the targeted entity
			if (Input.GetMouseButtonDown(0))
			{
				if (targetEntity != null && targetEntity != _firingEntity)
				{
					if (_isMissile)
					{
						LaunchMissile(targetEntity);
					}
					else
					{
						LaunchInterceptor(targetEntity);
					}
				}
			}
		}

		public void LaunchInterceptor(Entity targetEntity)
		{
			if (!_firingEntity.IsLauncher || targetEntity == null) return;
			if (_firingEntity.LaunchCapability.Ammo <= 0) return;

			LaunchCapability launcher = _firingEntity.LaunchCapability;

			VehicleModel model = VehicleModel.CreateFromComposition(launcher.ProjectileRadius, launcher.ProjectileCompositionIndex);

			if (OrbitalMath.SolveInterceptionVelocity(_firingEntity.PhysicsBody, targetEntity.PhysicsBody, launcher.FireSpeed, out double2 interceptVelocity, projectileMass: model.Mass, maxTime: 3600 * 24f, coarseSteps: 100))
			{
				Debug.Log($"[{GetType().Name}] Successful interception fired at {targetEntity.Name}.");
				EntityCreation.Create($"Projectile {++_numShotSpawned}", model, _firingEntity.Position, interceptVelocity, 0, 0, launcher.ProjectileCompositionIndex, true, EntityType.Projectile, BodySpawnOptions.Default);
				launcher.Ammo--;
			}
			else
			{
				Debug.Log($"[{GetType().Name}] No possible interception launch solution found to hit {targetEntity.Name}.");
			}
		}

		public void LaunchMissile(Entity targetEntity)
		{
			if (!_firingEntity.IsMissileLauncher || targetEntity == null) return;
			if (_firingEntity.MissileCapability.Ammo <= 0) return;

			MissileCapability launcher = _firingEntity.MissileCapability;

			VehicleModel model = VehicleModel.CreateFromComposition(launcher.ProjectileRadius, launcher.ProjectileCompositionIndex);

			double2 forward = new double2(math.cos(_firingEntity.Orientation), math.sin(_firingEntity.Orientation));
			double2 right = new double2(-forward.y, forward.x);
			
			double clearanceDistance = _firingEntity.Radius + launcher.ProjectileRadius + 5.0;
			double2 spawnPos = _firingEntity.Position + (forward * clearanceDistance);
			double2 initialVelocity = _firingEntity.Velocity + (forward * 20.0) + (right * 5.0);

			List<IEntityCapability> capabilities = new List<IEntityCapability>()
			{
				NavigationCapability.Missile,
				WarheadCapability.Missile
			};

			var navTask = new NavigateTask(targetEntity);

			EntityCreation.Create($"Missile {++_numMissileSpawned}", model, spawnPos, initialVelocity, _firingEntity.Orientation, 0, launcher.ProjectileCompositionIndex, true, EntityType.Missile, BodySpawnOptions.Default, capabilities, navTask);
			launcher.Ammo--;
		}

		public void EndFiringSequence()
		{
			if (_fireLine != null)
			{
				_fireLine.SetPoints(null);
				_fireLine.gameObject.SetActive(false);
			}

			if (_orbitPreviewLine != null)
			{
				_orbitPreviewLine.SetPoints(null);
				_orbitPreviewLine.gameObject.SetActive(false);
			}

			_isActive = false;
			_firingEntity = null;

			SimulationInteractionController.Instance.SetSelectionMode();
			// PhysicsManager2D.Instance.SetPaused(_wasPausedAtStart);
		}
	}
}