using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;
using Gravitas2D;
using TMPro;

namespace GravitasDemo
{
	[DefaultExecutionOrder(100)]
	public class GridVisualizer : MonoBehaviour
	{
		[Header("Grid Configuration")]
		[Tooltip("Multiplier applied to the current order of magnitude to define grid density. (e.g., 0.1 provides ~10 lines per axis).")]
		[SerializeField] private double _scaleDensity = 0.1;
		[SerializeField] private int magnitudeBase = 5;
		[SerializeField] private float _gridThickness = 1.0f;
		[SerializeField] private Color _gridColor = new Color(1f, 1f, 1f, 0.1f);
		[SerializeField] private Material _gridMaterial;
		[SerializeField] private RectTransform _gridContainer;

		[Header("Minor Grid Configuration")]
		[Tooltip("Number of subdivisions between major grid lines.")]
		[SerializeField, Min(1)] private int _minorSubdivisions = 5;
		[SerializeField] private float _minorGridThickness = 0.5f;
		[SerializeField] private Color _minorGridColor = new Color(1f, 1f, 1f, 0.05f);

		[Tooltip("Failsafe to prevent Unity freezes if zoomed out too far with an invalid grid scale.")]
		[SerializeField] private int _maxLinesPerAxis = 500;
		[SerializeField] private TextMeshProUGUI _scaleText;

		private List<LineGraphic> _majorGridLines = new List<LineGraphic>();
		private List<LineGraphic> _minorGridLines = new List<LineGraphic>();
		private int _activeMajorLineCount = 0;
		private int _activeMinorLineCount = 0;
		private double _dynamicGridScale = 1000000;
		private double2 _gridReferenceCenter;
		private double2 _previousViewCenter;
		private double2 _lastEntityPos;
		private bool _isInitialized = false;
		private Entity _lastTrackedEntity = null;
		private bool _wasTrackingLastFrame = false;

		public double CurrentGridScale => _dynamicGridScale;

		void LateUpdate()
		{
			if (SimulationRenderer.Instance == null || SimulationRenderer.Instance.VisualizationArea == null) 
				return;
				
			DrawGrid();
		}

		private bool IsUserPanning()
		{
			if (SimulationRenderer.Instance == null) return false;

			// Check Middle-Mouse Drag
			bool isMousePanning = SimulationRenderer.Instance.IsMouseInVisualizationArea() &&
								Input.GetMouseButton(2) &&
								(Mathf.Abs(Input.GetAxis("Mouse X")) > 0.001f || Mathf.Abs(Input.GetAxis("Mouse Y")) > 0.001f);

			// Check WASD / Arrow Key Panning
			bool isKeyboardPanning = Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.A) ||
									Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.D) ||
									Input.GetKey(KeyCode.UpArrow) || Input.GetKey(KeyCode.DownArrow) ||
									Input.GetKey(KeyCode.LeftArrow) || Input.GetKey(KeyCode.RightArrow);

			return isMousePanning || isKeyboardPanning;
		}

		private void DrawGrid()
		{
			_activeMajorLineCount = 0;
			_activeMinorLineCount = 0;
			var bounds = SimulationRenderer.Instance.ViewBounds;
			
			// Input validation: Prevent log10(0) or looping indefinitely
			if (bounds.Size.x <= 0 || bounds.Size.y <= 0) 
				return;

			double2 currentViewCenter = SimulationRenderer.Instance.ViewCenter;

			// Check which entity is currently tracked in the simulation
			Entity currentTrackedEntity = null;
			if (EntitiesManager.Instance != null)
			{
				foreach (var entity in EntitiesManager.Instance.KnownEntities)
				{
					if (entity.IsTracked)
					{
						currentTrackedEntity = entity;
						break;
					}
				}
			}

			bool isTracking = currentTrackedEntity != null;

			if (!_isInitialized)
			{
				_gridReferenceCenter = currentViewCenter;
				_previousViewCenter = currentViewCenter;
				_isInitialized = true;
			}
			else if (isTracking)
			{
				var entityVisual = EntitiesManager.Instance != null ? EntitiesManager.Instance.GetVisualForEntity(currentTrackedEntity) : null;

				if (entityVisual != null)
				{
					double2 currentEntityPos = entityVisual.PhysicsBody.Position;

					if (!_wasTrackingLastFrame || currentTrackedEntity != _lastTrackedEntity)
					{
						_gridReferenceCenter = currentViewCenter;
						_lastEntityPos = currentEntityPos;
					}
					else
					{
						// Move grid reference center by the entity's exact movement delta.
						// This keeps the grid locked to the fast-moving entity while allowing manual pan offsets.
						double2 entityDelta = currentEntityPos - _lastEntityPos;
						_gridReferenceCenter += entityDelta;
						_lastEntityPos = currentEntityPos;
					}
				}
				else
				{
					_gridReferenceCenter = currentViewCenter;
				}
			}
			else
			{
				// Free-roam mode
				double2 viewCenterDelta = currentViewCenter - _previousViewCenter;
				bool isZooming = Mathf.Abs(Input.mouseScrollDelta.y) > 0.01f;
				
				// Do not adjust reference center if the user is panning or actively zooming off-center
				if (!IsUserPanning() && !isZooming && math.lengthsq(viewCenterDelta) > 0)
				{
					_gridReferenceCenter += viewCenterDelta;
				}
			}

			_previousViewCenter = currentViewCenter;
			_lastTrackedEntity = currentTrackedEntity;
			_wasTrackingLastFrame = isTracking;

			// Ensure we don't divide by zero or take log of a negative number by forcing a minimum base of 2
			int safeBase = math.max(2, magnitudeBase);
			
			// Calculate the base order of magnitude of the current view window and apply the density target
			double viewMagnitude = math.pow(safeBase, math.round(math.log(bounds.Size.x) / math.log(safeBase)));
			_dynamicGridScale = viewMagnitude * _scaleDensity;

			if (_dynamicGridScale <= 0) return;

			int subdivisions = math.max(1, _minorSubdivisions);
			double minorStep = _dynamicGridScale / subdivisions;

			double endX = bounds.MinX + bounds.Size.x;
			double endY = bounds.MinY + bounds.Size.y;

			// --- Draw Vertical Lines (Index-Based) ---
			double relMinX = bounds.MinX - _gridReferenceCenter.x;
			double relMaxX = endX - _gridReferenceCenter.x;

			int iStart = (int)math.floor(relMinX / minorStep);
			int iEnd = (int)math.ceil(relMaxX / minorStep);

			int linesDrawn = 0;
			for (int i = iStart; i <= iEnd; i++)
			{
				if (linesDrawn++ > _maxLinesPerAxis * subdivisions) break;

				double x = _gridReferenceCenter.x + i * minorStep;
				Vector2 p1 = SimulationRenderer.Instance.SimPositionToViewUI(new double2(x, bounds.MinY));
				Vector2 p2 = SimulationRenderer.Instance.SimPositionToViewUI(new double2(x, endY));

				if (i % subdivisions == 0)
				{
					UpdateMajorLine(p1, p2);
				}
				else
				{
					UpdateMinorLine(p1, p2);
				}
			}

			// --- Draw Horizontal Lines (Index-Based) ---
			double relMinY = bounds.MinY - _gridReferenceCenter.y;
			double relMaxY = endY - _gridReferenceCenter.y;

			int jStart = (int)math.floor(relMinY / minorStep);
			int jEnd = (int)math.ceil(relMaxY / minorStep);

			linesDrawn = 0;
			for (int j = jStart; j <= jEnd; j++)
			{
				if (linesDrawn++ > _maxLinesPerAxis * subdivisions) break;

				double y = _gridReferenceCenter.y + j * minorStep;
				Vector2 p1 = SimulationRenderer.Instance.SimPositionToViewUI(new double2(bounds.MinX, y));
				Vector2 p2 = SimulationRenderer.Instance.SimPositionToViewUI(new double2(endX, y));

				if (j % subdivisions == 0)
				{
					UpdateMajorLine(p1, p2);
				}
				else
				{
					UpdateMinorLine(p1, p2);
				}
			}

			// Deactivate unused pooled major lines
			for (int i = _activeMajorLineCount; i < _majorGridLines.Count; i++)
			{
				if (_majorGridLines[i].gameObject.activeSelf)
				{
					_majorGridLines[i].gameObject.SetActive(false);
				}
			}

			// Deactivate unused pooled minor lines
			for (int i = _activeMinorLineCount; i < _minorGridLines.Count; i++)
			{
				if (_minorGridLines[i].gameObject.activeSelf)
				{
					_minorGridLines[i].gameObject.SetActive(false);
				}
			}

			if (_scaleText) _scaleText.text = FormatDistance(_dynamicGridScale);
		}

		private string FormatDistance(double meters)
		{
			double absMeters = math.abs(meters);
			if (absMeters < 1000.0) return $"{meters:0} m";
			if (absMeters < 1_000_000.0) return $"{meters / 1000.0:0.0} km";
			if (absMeters < 1_000_000_000.0) return $"{meters / 1_000_000.0:0.0} Mm";
			return $"{meters / 1_000_000_000.0:0.0} Gm";
		}

		private void UpdateMajorLine(Vector2 p1, Vector2 p2)
		{
			LineGraphic graphic;

			if (_activeMajorLineCount < _majorGridLines.Count)
			{
				graphic = _majorGridLines[_activeMajorLineCount];
				if (!graphic.gameObject.activeSelf) 
				{
					graphic.gameObject.SetActive(true);
				}
			}
			else
			{
				graphic = LineGraphic.CreateLine($"MajorGridLine_{_activeMajorLineCount}", _gridThickness, _gridColor, _gridMaterial, LineType.Grid, _gridContainer);
				_majorGridLines.Add(graphic);
			}

			graphic.SetPoints(new Vector2[] { p1, p2 });
			_activeMajorLineCount++;
		}

		private void UpdateMinorLine(Vector2 p1, Vector2 p2)
		{
			LineGraphic graphic;

			if (_activeMinorLineCount < _minorGridLines.Count)
			{
				graphic = _minorGridLines[_activeMinorLineCount];
				if (!graphic.gameObject.activeSelf) 
				{
					graphic.gameObject.SetActive(true);
				}
			}
			else
			{
				graphic = LineGraphic.CreateLine($"MinorGridLine_{_activeMinorLineCount}", _minorGridThickness, _minorGridColor, _gridMaterial, LineType.Grid, _gridContainer);
				_minorGridLines.Add(graphic);
			}

			graphic.SetPoints(new Vector2[] { p1, p2 });
			_activeMinorLineCount++;
		}
	}
}