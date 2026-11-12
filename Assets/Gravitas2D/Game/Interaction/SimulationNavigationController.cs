using System.Collections.Generic;
using Gravitas2D;
using GravitasDemo.Rendering;
using Unity.Mathematics;
using UnityEngine;

namespace GravitasDemo.Interaction
{
	public class SimulationNavigationController : MonoBehaviour
	{
		public static SimulationNavigationController Instance { get; private set; }

		[SerializeField] private RectTransform _navigationCursorPrefab;
		[SerializeField] private float _lineWidth = 1.0f;
		[SerializeField] private Color _navLineColor = new Color(0.2f, 0.8f, 1f, 0.8f); // Cyan
		[SerializeField] private Material _lineMaterial;
		[SerializeField] private RectTransform _lineContainer;

		private LineGraphic _navLine;
		private Entity _navigatingEntity;
		private EntityGroup _navigatingGroup;
		private bool _wasPausedAtStart;
		private bool _isIntercept;
		private bool _isActive = false;

		public bool IsActive => _isActive;
		public bool IsRendezvousActive => _isActive && !_isIntercept;
		public bool IsInterceptActive => _isActive && _isIntercept;

		void Start()
		{
			if (Instance != null && Instance != this)
			{
				Destroy(gameObject);
				return;
			}
			Instance = this;

			if (_navigationCursorPrefab != null) _navigationCursorPrefab.gameObject.SetActive(false);
		}

		void OnDestroy()
		{
			if (_navLine != null) Destroy(_navLine.gameObject);
			if (Instance == this) Instance = null;
		}

		public void StartInterceptSequenceOfSelected()
		{
			MapObject selected = SimulationInteractionController.Instance.SelectedMapObject;
			if (selected is EntityMapObject ego)
			{
				StartNavigationSequence(ego.Entity, true);
			}
			else if (selected is EntityGroupMapObject ggo)
			{
				StartNavigationSequence(ggo.Group, true);
			}
		}

		public void StartRendezvousSequenceOfSelected()
		{
			MapObject selected = SimulationInteractionController.Instance.SelectedMapObject;
			if (selected is EntityMapObject ego)
			{
				StartNavigationSequence(ego.Entity, false);
			}
			else if (selected is EntityGroupMapObject ggo)
			{
				StartNavigationSequence(ggo.Group, false);
			}
		}

		public void StartNavigationSequence(Entity navigatingEntity, bool isIntercept)
		{
			if (navigatingEntity == null) return;
			if (navigatingEntity.IsCelestial || !navigatingEntity.IsNavigable) return;

			_navigatingGroup = null;
			_navigatingEntity = navigatingEntity;
			StartNavigationCommon(isIntercept);
		}

		public void StartNavigationSequence(EntityGroup navigatingGroup, bool isIntercept)
		{
			if (navigatingGroup == null || navigatingGroup.Leader == null) return;
			if (navigatingGroup.Leader.IsCelestial || !navigatingGroup.Leader.IsNavigable) return;

			_navigatingGroup = navigatingGroup;
			_navigatingEntity = navigatingGroup.Leader; // Use group leader as reference for navline and hover indicator
			StartNavigationCommon(isIntercept);
		}

		private void StartNavigationCommon(bool isIntercept)
		{
			_isIntercept = isIntercept;

			_wasPausedAtStart = PhysicsManager2D.Instance.IsPaused;
			PhysicsManager2D.Instance.SetPaused(true);

			SimulationInteractionController.Instance.SetNavigationMode();

			_isActive = true;

			if (_navigationCursorPrefab != null) _navigationCursorPrefab.gameObject.SetActive(true);
			InitializeNavLine();
		}

		private void InitializeNavLine()
		{
			if (_navLine != null)
			{
				_navLine.gameObject.SetActive(true);
				return;
			}

			_navLine = LineGraphic.CreateLine("NavigationVectorLine", _lineWidth, _navLineColor, _lineMaterial, LineType.Basic, _lineContainer);
		}

		public void AppendHoverIndicatorInfo(Entity hoveredEntity, ref string text, ref Color color)
		{
			if (_navigatingEntity == null || hoveredEntity == _navigatingEntity) return;

			double mu = _navigatingEntity.PhysicsBody != null ? _navigatingEntity.PhysicsBody.GravParam : 0;
			PhysicsBody parent = _navigatingEntity.PhysicsBody != null ? _navigatingEntity.PhysicsBody.ParentSoI : null;
			float maxForce = _navigatingEntity.NavigationCapability != null ? _navigatingEntity.NavigationCapability.MaxForce : 10000f;
			float tof = 0;

			if (_isIntercept)
			{
				tof = InterceptController.ComputeInterceptTimeOfFlight(_navigatingEntity.Position, _navigatingEntity.Velocity, hoveredEntity.Position, hoveredEntity.Velocity, _navigatingEntity.Mass, maxForce, mu, parent);
			}
			else
			{
				// tof = NavigationCapability.ComputeRendezvousTimeOfFlight(_navigatingEntity.Position, _navigatingEntity.Velocity, hoveredEntity.Position, hoveredEntity.Velocity, _navigatingEntity.Mass, maxForce, mu, parent);
				var vehicleInput = new RendezvousGuidanceSolver.VehicleInput(
					_navigatingEntity.Position,
					_navigatingEntity.Velocity,
					_navigatingEntity.Mass,
					_navigatingEntity.PhysicsBody.Index);

				var targetInput = new RendezvousGuidanceSolver.TargetInput(
					hoveredEntity.Position,
					hoveredEntity.Velocity,
					RendezvousController.GetRendezvousOffset(_navigatingEntity.Position, _navigatingEntity.Radius, hoveredEntity, _navigatingEntity.InGroup, _navigatingEntity.Group, _navigatingEntity.Group?.IndexOfMember(_navigatingEntity) ?? -1),
					hoveredEntity != null,
					hoveredEntity?.IsCelestial ?? false,
					hoveredEntity?.PhysicsBody.Index ?? -1);

				// List<double2> predictedPath = new List<double2>();
				// Vector2[] pathBuffer = new Vector2[1];
				if (!RendezvousGuidanceSolver.TrySolve(
					vehicleInput,
					targetInput,
					maxForce,
					100,
					1,
					PhysicsManager2D.Instance.PhysicsTimeStep,
					0,
					out _,
					out tof/*,
					predictedPath*/))
				{
					tof = 0;
				}

				// LineGraphic.RenderPathToGraphic(predictedPath, _navLine, pathBuffer);
			}

			color = _navLineColor;
			text += $"\nMode: {(_isIntercept ? "Intercept" : "Rendezvous")}" +
					$"\nEst. Time: {tof:F0} s";
		}

		void LateUpdate()
		{
			if (!_isActive) return;
			if (_navigatingEntity == null || _navigatingEntity.IsDestroyed)
			{
				EndNavigationSequence();
				return;
			}
			if (SimulationRenderer.Instance == null || SimulationRenderer.Instance.VisualizationArea == null) return;

			// Right-click to cancel/exit navigation mode
			if (Input.GetMouseButtonDown(1))
			{
				EndNavigationSequence();
				return;
			}

			if (!SimulationRenderer.Instance.TryGetMouseViewPosition(out Vector2 localUIPos)) return;

			// Check if hovering over a valid target entity and snap endpoint if applicable
			Entity targetEntity = SimulationInteractionController.Instance.HoveredEntity;
			Vector2 targetUIPos = localUIPos;

			if (targetEntity != null && targetEntity != _navigatingEntity)
			{
				targetUIPos = SimulationRenderer.Instance.SimPositionToViewUI(targetEntity.Position);
			}

			if (_navigationCursorPrefab != null)
			{
				_navigationCursorPrefab.localPosition = new Vector3(targetUIPos.x, targetUIPos.y, _navigationCursorPrefab.localPosition.z);
			}

			// Update vector line from navigating entity position to the target UI position (mouse or snapped entity)
			if (_navLine != null)
			{
				Vector2 entityUIPos = SimulationRenderer.Instance.SimPositionToViewUI(_navigatingEntity.Position);
				_navLine.SetPoints(new Vector2[] { entityUIPos, targetUIPos });
			}

			// Left-click to confirm navigation towards target entity or empty space position
			if (Input.GetMouseButtonDown(0))
			{
				if (_navigatingGroup != null)
				{
					foreach (var entity in _navigatingGroup.Entities)
					{
						if (entity == null || entity.IsDestroyed || entity.IsCelestial || !entity.IsNavigable) continue;

						if (targetEntity != null && targetEntity != entity)
						{
							if (_isIntercept) entity.InterceptEntity(targetEntity);
							else entity.RendezvousWithEntity(targetEntity);
						}
						else
						{
							double2 targetSimPos = SimulationRenderer.Instance.ViewUIToSimPosition(localUIPos);
							if (_isIntercept) entity.InterceptPosition(targetSimPos);
							else entity.RendezvousWithPosition(targetSimPos);
						}
					}
				}
				else if (_navigatingEntity != null)
				{
					if (targetEntity != null && targetEntity != _navigatingEntity)
					{
						if (_isIntercept) _navigatingEntity.InterceptEntity(targetEntity);
						else _navigatingEntity.RendezvousWithEntity(targetEntity);
					}
					else
					{
						double2 targetSimPos = SimulationRenderer.Instance.ViewUIToSimPosition(localUIPos);
						if (_isIntercept) _navigatingEntity.InterceptPosition(targetSimPos);
						else _navigatingEntity.RendezvousWithPosition(targetSimPos);
					}
				}

				EndNavigationSequence();
			}
		}

		public void EndNavigationSequence()
		{
			if (_navigationCursorPrefab != null) _navigationCursorPrefab.gameObject.SetActive(false);

			if (_navLine != null)
			{
				_navLine.SetPoints(null);
				_navLine.gameObject.SetActive(false);
			}

			_isActive = false;
			_navigatingEntity = null;
			_navigatingGroup = null;

			SimulationInteractionController.Instance.SetSelectionMode();
			PhysicsManager2D.Instance.SetPaused(_wasPausedAtStart);
		}
	}
}