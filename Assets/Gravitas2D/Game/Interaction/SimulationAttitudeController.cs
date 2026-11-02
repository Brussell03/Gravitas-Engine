using Gravitas2D;
using GravitasDemo.Rendering;
using Unity.Mathematics;
using UnityEngine;

namespace GravitasDemo.Interaction
{
	public class SimulationAttitudeController : MonoBehaviour
	{
		public static SimulationAttitudeController Instance { get; private set; }

		[SerializeField] private RectTransform _attitudeCursorPrefab;
		[SerializeField] private float _lineWidth = 1.0f;
		[SerializeField] private Color _attitudeLineColor = new Color(1f, 0.92f, 0.015f, 1f); // Yellow
		[SerializeField] private Material _lineMaterial;
		[SerializeField] private RectTransform _lineContainer;

		private LineGraphic _attitudeLine;
		private Entity _attitudingEntity;
		private bool _wasPausedAtStart;
		private bool _isActive = false;

		public bool IsAttitudeActive => _isActive;

		void Start()
		{
			if (Instance != null && Instance != this)
			{
				Destroy(gameObject);
				return;
			}
			Instance = this;

			if (_attitudeCursorPrefab != null) _attitudeCursorPrefab.gameObject.SetActive(false);
		}

		void OnDestroy()
		{
			if (_attitudeLine != null) Destroy(_attitudeLine.gameObject);
			if (Instance == this) Instance = null;
		}

		public void StartAttitudeSequenceOfSelected()
		{
			StartAttitudeSequence(SimulationInteractionController.Instance.SelectedEntity);
		}

		public void StartAttitudeSequence(Entity attitudeEntity)
		{
			if (attitudeEntity == null) return;
			if (attitudeEntity.IsCelestial || !attitudeEntity.IsNavigable) return;

			// _wasPausedAtStart = PhysicsManager2D.Instance.IsPaused;
			// PhysicsManager2D.Instance.SetPaused(true);

			SimulationInteractionController.Instance.SetAttitudeMode();

			_attitudingEntity = attitudeEntity;
			_isActive = true;

			if (_attitudeCursorPrefab != null) _attitudeCursorPrefab.gameObject.SetActive(true);
			InitializeAttitudeLine();
		}

		private void InitializeAttitudeLine()
		{
			if (_attitudeLine != null)
			{
				_attitudeLine.gameObject.SetActive(true);
				return;
			}

			_attitudeLine = LineGraphic.CreateLine("AttitudeVectorLine", _lineWidth, _attitudeLineColor, _lineMaterial, LineType.Basic, _lineContainer);
		}

		void LateUpdate()
		{
			if (!_isActive) return;
			if (_attitudingEntity == null || _attitudingEntity.IsDestroyed)
			{
				EndAttitudeSequence();
				return;
			}
			if (SimulationRenderer.Instance == null || SimulationRenderer.Instance.VisualizationArea == null) return;

			// Right-click to cancel/exit attitude mode
			if (Input.GetMouseButtonDown(1))
			{
				EndAttitudeSequence();
				return;
			}

			if (!SimulationRenderer.Instance.TryGetMouseViewPosition(out Vector2 localUIPos)) return;
			if (!SimulationRenderer.Instance.TryGetMouseSimPosition(out double2 mouseSimPos, out _)) return;

			if (_attitudeCursorPrefab != null)
			{
				_attitudeCursorPrefab.localPosition = new Vector3(localUIPos.x, localUIPos.y, _attitudeCursorPrefab.localPosition.z);
			}

			// Update vector line from entity position to the mouse position
			if (_attitudeLine != null)
			{
				Vector2 entityUIPos = SimulationRenderer.Instance.SimPositionToViewUI(_attitudingEntity.Position);
				_attitudeLine.SetPoints(new Vector2[] { entityUIPos, localUIPos });
			}

			// Continuously update target orientation towards mouse position relative to the entity while dragging/hovering
			double2 deltaPos = mouseSimPos - _attitudingEntity.Position;
			float targetOrientation = math.atan2((float)deltaPos.y, (float)deltaPos.x);
			SimulationInteractionController.Instance.SetSelectedTargetOrientation(targetOrientation);

			// Left-click to confirm and exit attitude mode
			if (Input.GetMouseButtonDown(0))
			{
				Debug.Log($"[{GetType().Name}] Confirmed target orientation for {_attitudingEntity.Name}.");
				EndAttitudeSequence();
			}
		}

		public void EndAttitudeSequence()
		{
			if (_attitudeCursorPrefab != null) _attitudeCursorPrefab.gameObject.SetActive(false);

			if (_attitudeLine != null)
			{
				_attitudeLine.SetPoints(null);
				_attitudeLine.gameObject.SetActive(false);
			}

			_isActive = false;
			_attitudingEntity = null;

			SimulationInteractionController.Instance.SetSelectionMode();
			// PhysicsManager2D.Instance.SetPaused(_wasPausedAtStart);
		}
	}
}