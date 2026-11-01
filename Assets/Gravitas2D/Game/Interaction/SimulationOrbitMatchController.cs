using System.Collections.Generic;
using Gravitas2D;
using TMPro;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.UI;

namespace GravitasDemo
{
	public class SimulationOrbitMatchController : MonoBehaviour
	{
		public static SimulationOrbitMatchController Instance { get; private set; }

		[SerializeField] private RectTransform _placementPrefab;
		[SerializeField] private float _velocityScaleMultiplier = 10.0f;
		[SerializeField] private float _lineWidth = 1.0f;
		[SerializeField] private Color _velocityLineColor = Color.cyan;
		[SerializeField] private Color _previewLineColor = new Color(0.25f, 1f, 0.4f, 0.8f); // Green
		[SerializeField] private int _previewOrbitResolution = 360;
		[SerializeField] private Material _lineMaterial;
		[SerializeField] private RectTransform _lineContainer;

		private double2 _orbitTargetPositionOffset; // Position relative to dominant body
		private Entity _navigatingEntity;

		private LineGraphic _velocityLine;
		private LineGraphic _previewTrajectory;
		private Vector2[] _pathBuffer;
		private bool _wasPausedAtStart;

		private enum OrbitMatchState { Inactive, SelectingPosition, SettingVelocity }
		private OrbitMatchState _state = OrbitMatchState.Inactive;

		public bool IsActive => _state != OrbitMatchState.Inactive;
		public bool IsOrbitMatchActive => _navigatingEntity != null;
		public bool IsPreviewingOrbitMatch => _state == OrbitMatchState.SettingVelocity;
		public double2 PreviewPosition => GetCurrentAbsolutePosition();
		public double2 PreviewVelocity { get; private set; }

		void Start()
		{
			if (Instance != null && Instance != this)
			{
				Destroy(gameObject);
				return;
			}
			Instance = this;

			if (_placementPrefab != null) _placementPrefab.gameObject.SetActive(false);

			_pathBuffer = new Vector2[_previewOrbitResolution];
		}

		void OnDestroy()
		{
			if (Instance == this) Instance = null;
		}

		public void StartOrbitMatchSequenceOfSelected()
		{
			StartOrbitMatchSequence(SimulationInteractionController.Instance.SelectedEntity);
		}

		public void StartOrbitMatchSequence(Entity navigatingEntity)
		{
			if (navigatingEntity == null) return;
			if (navigatingEntity.IsCelestial || !navigatingEntity.IsNavigable) return;

			_wasPausedAtStart = PhysicsManager2D.Instance.IsPaused;
			PhysicsManager2D.Instance.SetPaused(true);

			SimulationInteractionController.Instance.SetOrbitMatchMode();
			
			_navigatingEntity = navigatingEntity;
			_state = OrbitMatchState.SelectingPosition;

			if (_placementPrefab != null) _placementPrefab.gameObject.SetActive(true);
		}

		private PhysicsBody GetParentSoI()
		{
			return _navigatingEntity != null && _navigatingEntity.DominantEntity != null 
				? _navigatingEntity.DominantEntity.PhysicsBody 
				: null;
		}

		private double2 GetCurrentAbsolutePosition()
		{
			var parent = GetParentSoI();
			return parent != null ? parent.Position + _orbitTargetPositionOffset : _orbitTargetPositionOffset;
		}

		public void AppendHoverIndicatorInfo(Entity hoveredEntity, ref string text, ref Color color)
		{
			if (_state == OrbitMatchState.SettingVelocity)
			{
				color = _previewLineColor;
				text += $"\nSet Orbit to Match" +
						$"\nVel: {math.length(PreviewVelocity):F1} m/s";
			}
			else if (_state == OrbitMatchState.SelectingPosition)
			{
				color = _velocityLineColor;
				text += $"\nSelect Orbit Position";
			}
		}

		public void UpdateHoverIndicatorForNonEntity(RectTransform lockIndicator, Image image, TextMeshProUGUI textMesh)
		{
			if (lockIndicator == null) return;

			lockIndicator.gameObject.SetActive(true);
			Vector2 uiPos = SimulationRenderer.Instance.SimPositionToViewUI(PreviewPosition);
			lockIndicator.localPosition = new Vector3(uiPos.x, uiPos.y, lockIndicator.localPosition.z);

			if (textMesh != null)
			{
				textMesh.text = _state == OrbitMatchState.SettingVelocity
					? $"Set Orbit to Match\nVel: {math.length(PreviewVelocity):F1} m/s"
					: $"Select Orbit Position";
			}

			if (image != null)
			{
				image.color = _state == OrbitMatchState.SettingVelocity ? _previewLineColor : _velocityLineColor;
			}
		}

		void LateUpdate()
		{
			if (_state == OrbitMatchState.Inactive) return;
			if (_navigatingEntity == null || _navigatingEntity.IsDestroyed)
			{
				EndOrbitMatchSequence();
				return;
			}
			if (SimulationRenderer.Instance == null || SimulationRenderer.Instance.VisualizationArea == null) return;

			// Right-click to cancel or step back
			if (Input.GetMouseButtonDown(1))
			{
				if (_state == OrbitMatchState.SettingVelocity)
				{
					_state = OrbitMatchState.SelectingPosition;
					ClearVelocityPhaseVisuals();
				}
				else if (_state == OrbitMatchState.SelectingPosition)
				{
					EndOrbitMatchSequence();
				}
				return;
			}

			if (!SimulationRenderer.Instance.TryGetMouseSimPosition(out double2 mouseSimPos, out Vector2 localUIPos)) return;

			var parent = GetParentSoI();

			if (_state == OrbitMatchState.SelectingPosition)
			{
				UpdatePrefabPosition(mouseSimPos);

				// Store position relative to the dominant body so it moves with it
				_orbitTargetPositionOffset = parent != null ? mouseSimPos - parent.Position : mouseSimPos;

				if (Input.GetMouseButtonDown(0))
				{
					_state = OrbitMatchState.SettingVelocity;
					
					InitializeVelocityLine();
				}
			}
			else if (_state == OrbitMatchState.SettingVelocity)
			{
				double2 absTargetPos = GetCurrentAbsolutePosition();
				UpdatePrefabPosition(absTargetPos);

				// Define target velocity relative to parent body's motion frame
				double2 parentVel = parent != null ? parent.Velocity : double2.zero;
				double2 mouseOffset = mouseSimPos - absTargetPos;
				double2 targetVelocity = parentVel + (mouseOffset * _velocityScaleMultiplier);

				PreviewVelocity = targetVelocity;

				if (_velocityLine != null)
				{
					Vector2 posUI = SimulationRenderer.Instance.SimPositionToViewUI(absTargetPos);
					_velocityLine.SetPoints(new Vector2[] { posUI, localUIPos });
				}

				UpdatePreviewTrajectory(absTargetPos, targetVelocity);

				if (Input.GetMouseButtonDown(0))
				{
					ExecuteOrbitMatch(absTargetPos, targetVelocity);
				}
			}
		}

		private void ExecuteOrbitMatch(double2 targetPos, double2 targetVel)
		{
			_navigatingEntity.OrbitMatchWithState(targetPos, targetVel);
			EndOrbitMatchSequence();
		}

		private void UpdatePrefabPosition(double2 globalPos)
		{
			if (_placementPrefab == null) return;
			SimulationRenderer.Instance.SetUIPositionFromSimulationPosition(_placementPrefab, globalPos);
		}

		private void InitializeVelocityLine()
		{
			if (_velocityLine != null)
			{
				_velocityLine.gameObject.SetActive(true);
				return;
			}

			_velocityLine = LineGraphic.CreateLine("OrbitMatchVelocityLine", _lineWidth, _velocityLineColor, _lineMaterial, LineType.Basic, _lineContainer);
		}

		private void UpdatePreviewTrajectory(double2 targetPos, double2 velocity)
		{
			if (_previewTrajectory == null)
			{
				_previewTrajectory = LineGraphic.CreateLine("OrbitMatchPreview", _lineWidth, _previewLineColor, _lineMaterial, LineType.Basic, _lineContainer);
			}

			var parent = GetParentSoI();
			if (parent == null)
			{
				_previewTrajectory.SetPoints(null);
				return;
			}

			double mass = _navigatingEntity.Mass;
			double mu = OrbitalMath.GetStandardGravitationalParameter(parent.Mass, mass);
			double2[] path = OrbitalMath.CalculateTwoBodyOrbit(targetPos - parent.Position, velocity - parent.Velocity, parent.Position, mu, _previewOrbitResolution);
			
			Bounds2D effectiveBounds = SimulationRenderer.Instance.ViewBounds;
			float rectWidth = SimulationRenderer.Instance.VisualizationArea.rect.width;
			float rectHeight = SimulationRenderer.Instance.VisualizationArea.rect.height;

			_previewTrajectory.RenderPathToGraphic(path, _previewTrajectory, _pathBuffer, effectiveBounds, rectWidth, rectHeight);
			_previewTrajectory.gameObject.SetActive(true);
		}

		private void ClearVelocityPhaseVisuals()
		{
			if (_velocityLine != null)
			{
				_velocityLine.SetPoints(null);
				_velocityLine.gameObject.SetActive(false);
			}

			if (_previewTrajectory != null)
			{
				_previewTrajectory.SetPoints(null);
				_previewTrajectory.gameObject.SetActive(false);
			}
		}

		public void EndOrbitMatchSequence()
		{
			ClearVelocityPhaseVisuals();
			
			if (_placementPrefab != null) _placementPrefab.gameObject.SetActive(false);
			
			_state = OrbitMatchState.Inactive;
			_navigatingEntity = null;

			SimulationInteractionController.Instance.SetSelectionMode();

			PhysicsManager2D.Instance.SetPaused(_wasPausedAtStart);
		}
	}
}