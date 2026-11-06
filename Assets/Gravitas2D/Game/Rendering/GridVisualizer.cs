using Gravitas2D;
using GravitasDemo.Interaction;
using Unity.Mathematics;
using UnityEngine;
using TMPro;

namespace GravitasDemo.Rendering
{
	/// <summary>
	/// Procedural simulation grid. The CPU computes only the grid's scale and phase;
	/// a single UI quad and its fragment shader draw every major/minor line.
	///
	/// The reference-center rules intentionally match the former LineGraphic-based grid.
	/// </summary>
	[DefaultExecutionOrder(100)]
	public sealed class GridVisualizer : MonoBehaviour
	{
		[Header("Grid Configuration")]
		[Tooltip("Multiplier applied to the current order of magnitude to define grid density. (e.g., 0.1 provides ~10 major lines per axis).")]
		[SerializeField] private double _scaleDensity = 0.1;
		[SerializeField] private int magnitudeBase = 5;
		[SerializeField] private float _gridThickness = 2f;
		[SerializeField] private Color _gridColor = new Color(1f, 1f, 1f, 0.1f);
		[Tooltip("Use a material created from GravitasDemo/ProceduralGrid. If left empty, the shader is found at runtime.")]
		[SerializeField] private Material _gridMaterial;
		[SerializeField] private RectTransform _gridContainer;

		[Header("Minor Grid Configuration")]
		[Tooltip("Number of subdivisions between major grid lines.")]
		[SerializeField, Min(1)] private int _minorSubdivisions = 5;
		[SerializeField] private float _minorGridThickness = 1f;
		[SerializeField] private Color _minorGridColor = new Color(1f, 1f, 1f, 0.05f);

		[Header("Safety")]
		[Tooltip("Preserves the former line-count failsafe. Each axis renders at most this many major intervals multiplied by the subdivision count, plus the starting line.")]
		[SerializeField] private int _maxLinesPerAxis = 500;
		[SerializeField] private TextMeshProUGUI _scaleText;

		private GridShaderGraphic _gridGraphic;
		private double _dynamicGridScale = 1000000.0;
		private double2 _gridReferenceCenter;
		private double2 _previousViewCenter;
		private double2 _lastEntityPos;
		private bool _isInitialized;
		private Entity _lastTrackedEntity;
		private bool _wasTrackingLastFrame;
		private double _lastScaleTextValue = double.NaN;
		private double _lastScaleBoundsSizeX = double.NaN;
		private double _lastScaleDensity = double.NaN;
		private int _lastMagnitudeBase = int.MinValue;
		private bool _warnedMissingShader;

		public double CurrentGridScale => _dynamicGridScale;

		private void LateUpdate()
		{
			SimulationRenderer renderer = SimulationRenderer.Instance;
			if (renderer == null || renderer.VisualizationArea == null)
				return;

			if (!EnsureGridGraphic(renderer))
				return;

			UpdateGrid(renderer);
		}

		private bool EnsureGridGraphic(SimulationRenderer renderer)
		{
			if (_gridGraphic != null)
				return _gridGraphic.IsInitialized;

			Material sourceMaterial = _gridMaterial;
			Material temporaryMaterial = null;

			if (sourceMaterial == null ||
				!sourceMaterial.HasProperty("_GridMajorColor") ||
				!sourceMaterial.HasProperty("_GridMinorColor") ||
				!sourceMaterial.HasProperty("_GridMajorThickness") ||
				!sourceMaterial.HasProperty("_GridMinorThickness") ||
				!sourceMaterial.HasProperty("_GridPhase") ||
				!sourceMaterial.HasProperty("_GridCells") ||
				!sourceMaterial.HasProperty("_GridEnabled"))
			{
				Shader shader = Shader.Find("GravitasDemo/ProceduralGrid");
				if (shader == null)
				{
					if (!_warnedMissingShader)
					{
						Debug.LogError($"[{GetType().Name}] Could not find shader 'GravitasDemo/ProceduralGrid'. Assign a material using that shader to GridVisualizer.", this);
						_warnedMissingShader = true;
					}
					return false;
				}

				temporaryMaterial = new Material(shader)
				{
					name = "Procedural Grid Runtime Source",
					hideFlags = HideFlags.DontSave
				};
				sourceMaterial = temporaryMaterial;
			}

			Transform parent = _gridContainer != null ? _gridContainer : renderer.VisualizationArea;
			GameObject gridObject = new GameObject(
				"Procedural Simulation Grid",
				typeof(RectTransform),
				typeof(CanvasRenderer),
				typeof(GridShaderGraphic));

			RectTransform gridRect = gridObject.GetComponent<RectTransform>();
			gridRect.SetParent(parent, false);
			gridRect.anchorMin = Vector2.zero;
			gridRect.anchorMax = Vector2.one;
			gridRect.pivot = new Vector2(0.5f, 0.5f);
			gridRect.offsetMin = Vector2.zero;
			gridRect.offsetMax = Vector2.zero;
			gridRect.localPosition = Vector3.zero;
			gridRect.localRotation = Quaternion.identity;
			gridRect.localScale = Vector3.one;

			_gridGraphic = gridObject.GetComponent<GridShaderGraphic>();
			_gridGraphic.Initialize(sourceMaterial);
			_gridGraphic.raycastTarget = false;
			_gridGraphic.maskable = true;
			_gridGraphic.color = Color.white;

			if (temporaryMaterial != null)
				Destroy(temporaryMaterial);

			return _gridGraphic.IsInitialized;
		}

		private bool IsUserPanning(SimulationRenderer renderer)
		{
			// Do not perform a UI raycast every frame. First test whether middle-mouse
			// input is actually active; the old expression raycasted even when it was not.
			bool isMousePanning = Input.GetMouseButton(2) &&
				(Mathf.Abs(Input.GetAxis("Mouse X")) > 0.001f ||
				 Mathf.Abs(Input.GetAxis("Mouse Y")) > 0.001f) &&
				renderer.IsMouseInVisualizationArea();

			bool isKeyboardPanning = Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.A) ||
				Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.D) ||
				Input.GetKey(KeyCode.UpArrow) || Input.GetKey(KeyCode.DownArrow) ||
				Input.GetKey(KeyCode.LeftArrow) || Input.GetKey(KeyCode.RightArrow);

			return isMousePanning || isKeyboardPanning;
		}

		private void UpdateGrid(SimulationRenderer renderer)
		{
			Bounds2D bounds = renderer.ViewBounds;
			if (bounds.Size.x <= 0.0 || bounds.Size.y <= 0.0)
			{
				_gridGraphic.SetGridEnabled(false);
				return;
			}

			double2 currentViewCenter = renderer.ViewCenter;

			// The interaction controller already owns the tracked map object. Reading it
			// directly avoids scanning every known entity on every grid update.
			MapObject trackedMapObject = SimulationInteractionController.Instance != null
				? SimulationInteractionController.Instance.TrackedMapObject
				: null;
			Entity currentTrackedEntity = trackedMapObject != null ? trackedMapObject.Entity : null;
			if (currentTrackedEntity != null && currentTrackedEntity.IsDestroyed)
				currentTrackedEntity = null;

			bool isTracking = currentTrackedEntity != null;

			if (!_isInitialized)
			{
				_gridReferenceCenter = currentViewCenter;
				_previousViewCenter = currentViewCenter;
				_isInitialized = true;
			}
			else if (isTracking)
			{
				double2 currentEntityPos = currentTrackedEntity.Position;

				if (!_wasTrackingLastFrame || currentTrackedEntity != _lastTrackedEntity)
				{
					_gridReferenceCenter = currentViewCenter;
					_lastEntityPos = currentEntityPos;
				}
				else
				{
					// Move the grid reference by the tracked body's exact world-space delta.
					// Manual pan offsets therefore remain unchanged, as in the previous grid.
					double2 entityDelta = currentEntityPos - _lastEntityPos;
					_gridReferenceCenter += entityDelta;
					_lastEntityPos = currentEntityPos;
				}
			}
			else
			{
				double2 viewCenterDelta = currentViewCenter - _previousViewCenter;
				bool isZooming = Mathf.Abs(Input.mouseScrollDelta.y) > 0.01f;

				// Free-roam mode: preserve the previous rule that the grid's world anchor
				// follows view-center changes only when they are not user pan/zoom input.
				if (!IsUserPanning(renderer) && !isZooming && math.lengthsq(viewCenterDelta) > 0.0)
					_gridReferenceCenter += viewCenterDelta;
			}

			_previousViewCenter = currentViewCenter;
			_lastTrackedEntity = currentTrackedEntity;
			_wasTrackingLastFrame = isTracking;

			UpdateDynamicScale(bounds.Size.x);
			if (_dynamicGridScale <= 0.0 || double.IsNaN(_dynamicGridScale) || double.IsInfinity(_dynamicGridScale))
			{
				_gridGraphic.SetGridEnabled(false);
				return;
			}

			int subdivisions = math.max(1, _minorSubdivisions);
			double minorStep = _dynamicGridScale / subdivisions;
			if (minorStep <= 0.0 || double.IsNaN(minorStep) || double.IsInfinity(minorStep))
			{
				_gridGraphic.SetGridEnabled(false);
				return;
			}

			// Keep the large world coordinates in double precision on the CPU. Only a
			// fractional phase and a small modulo are sent to the GPU, avoiding float
			// precision loss at AU-scale coordinates and beyond.
			double relativeMinX = (bounds.MinX - _gridReferenceCenter.x) / minorStep;
			double relativeMinY = (bounds.MinY - _gridReferenceCenter.y) / minorStep;
			double startIndexX = math.floor(relativeMinX);
			double startIndexY = math.floor(relativeMinY);
			float phaseX = (float)(relativeMinX - startIndexX);
			float phaseY = (float)(relativeMinY - startIndexY);

			float startModuloX = PositiveModulo(startIndexX, subdivisions);
			float startModuloY = PositiveModulo(startIndexY, subdivisions);
			float cellsAcrossX = (float)(bounds.Size.x / minorStep);
			float cellsAcrossY = (float)(bounds.Size.y / minorStep);
			float maxLineIndex = (float)((double)_maxLinesPerAxis * subdivisions);

			_gridGraphic.SetGridParameters(
				new Vector4(phaseX, phaseY, startModuloX, startModuloY),
				new Vector4(cellsAcrossX, cellsAcrossY, subdivisions, maxLineIndex),
				_gridThickness,
				_minorGridThickness,
				_gridColor,
				_minorGridColor);
			_gridGraphic.SetGridEnabled(true);

			UpdateScaleText();
		}

		private void UpdateDynamicScale(double viewSizeX)
		{
			// The scale depends only on horizontal world span and serialized grid
			// configuration. Avoid pow/log work when none of those inputs changed.
			if (_lastScaleBoundsSizeX == viewSizeX &&
				_lastScaleDensity == _scaleDensity &&
				_lastMagnitudeBase == magnitudeBase)
				return;

			int safeBase = math.max(2, magnitudeBase);
			double viewMagnitude = math.pow(
				safeBase,
				math.round(math.log(viewSizeX) / math.log(safeBase)));
			_dynamicGridScale = viewMagnitude * _scaleDensity;

			_lastScaleBoundsSizeX = viewSizeX;
			_lastScaleDensity = _scaleDensity;
			_lastMagnitudeBase = magnitudeBase;
		}

		private static float PositiveModulo(double value, int divisor)
		{
			double remainder = value % divisor;
			if (remainder < 0.0)
				remainder += divisor;
			return (float)remainder;
		}

		private void UpdateScaleText()
		{
			if (_scaleText == null || _lastScaleTextValue == _dynamicGridScale)
				return;

			_scaleText.text = FormatDistance(_dynamicGridScale);
			_lastScaleTextValue = _dynamicGridScale;
		}

		private static string FormatDistance(double meters)
		{
			double absMeters = math.abs(meters);
			if (absMeters < 1000.0) return $"{meters:0} m";
			if (absMeters < 1_000_000.0) return $"{meters / 1000.0:0.0} km";
			if (absMeters < 1_000_000_000.0) return $"{meters / 1_000_000.0:0.0} Mm";
			return $"{meters / 1_000_000_000.0:0.0} Gm";
		}

		private void OnValidate()
		{
			_lastScaleBoundsSizeX = double.NaN;
			_lastScaleDensity = double.NaN;
			_lastMagnitudeBase = int.MinValue;
			_lastScaleTextValue = double.NaN;
		}

		private void OnDestroy()
		{
			if (_gridGraphic != null)
			{
				if (Application.isPlaying)
					Destroy(_gridGraphic.gameObject);
				else
					DestroyImmediate(_gridGraphic.gameObject);
				_gridGraphic = null;
			}
		}
	}
}