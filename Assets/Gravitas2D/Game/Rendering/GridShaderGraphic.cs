using UnityEngine;
using UnityEngine.UI;

namespace GravitasDemo.Rendering
{
	/// <summary>
	/// A single transparent UI quad that supplies procedural grid parameters to the
	/// fragment shader. Geometry is created once and never dirtied during normal updates.
	/// </summary>
	[RequireComponent(typeof(RectTransform), typeof(CanvasRenderer))]
	public sealed class GridShaderGraphic : MaskableGraphic
	{
		private static readonly int GridPhaseID = Shader.PropertyToID("_GridPhase");
		private static readonly int GridCellsID = Shader.PropertyToID("_GridCells");
		private static readonly int MajorThicknessID = Shader.PropertyToID("_GridMajorThickness");
		private static readonly int MinorThicknessID = Shader.PropertyToID("_GridMinorThickness");
		private static readonly int MajorColorID = Shader.PropertyToID("_GridMajorColor");
		private static readonly int MinorColorID = Shader.PropertyToID("_GridMinorColor");
		private static readonly int GridEnabledID = Shader.PropertyToID("_GridEnabled");

		private Material _runtimeMaterial;
		private Material _lastRenderingMaterial;
		private bool _isInitialized;
		private bool _gridEnabled = true;
		private Vector4 _currentPhase;
		private Vector4 _currentCells;
		private float _currentMajorThickness;
		private float _currentMinorThickness;
		private Color _currentMajorColor;
		private Color _currentMinorColor;
		private bool _hasGridParameters;

		public bool IsInitialized => _isInitialized;

		protected override void Awake()
		{
			base.Awake();
			raycastTarget = false;
			maskable = true;
		}

		public void Initialize(Material sourceMaterial)
		{
			if (sourceMaterial == null)
			{
				Debug.LogError($"[{GetType().Name}] A procedural grid material is required.", this);
				return;
			}

			if (!sourceMaterial.HasProperty(GridPhaseID) ||
				!sourceMaterial.HasProperty(GridCellsID) ||
				!sourceMaterial.HasProperty(MajorThicknessID) ||
				!sourceMaterial.HasProperty(MinorThicknessID) ||
				!sourceMaterial.HasProperty(MajorColorID) ||
				!sourceMaterial.HasProperty(MinorColorID) ||
				!sourceMaterial.HasProperty(GridEnabledID))
			{
				Debug.LogError($"[{GetType().Name}] The assigned material does not expose the procedural grid shader properties. Use 'GravitasDemo/ProceduralGrid'.", this);
				return;
			}

			_runtimeMaterial = new Material(sourceMaterial)
			{
				name = "Procedural Grid (Runtime Instance)",
				hideFlags = HideFlags.DontSave
			};
			material = _runtimeMaterial;
			_isInitialized = true;
			SetVerticesDirty();
		}

		public void SetGridParameters(
			Vector4 phase,
			Vector4 cells,
			float majorThickness,
			float minorThickness,
			Color majorColor,
			Color minorColor)
		{
			if (!_isInitialized || _runtimeMaterial == null)
				return;

			bool changed = !_hasGridParameters ||
				HasChanged(_currentPhase, phase) ||
				HasChanged(_currentCells, cells) ||
				!Mathf.Approximately(_currentMajorThickness, majorThickness) ||
				!Mathf.Approximately(_currentMinorThickness, minorThickness) ||
				_currentMajorColor != majorColor ||
				_currentMinorColor != minorColor;

			if (changed)
			{
				_currentPhase = phase;
				_currentCells = cells;
				_currentMajorThickness = Mathf.Max(0f, majorThickness);
				_currentMinorThickness = Mathf.Max(0f, minorThickness);
				_currentMajorColor = majorColor;
				_currentMinorColor = minorColor;
				_hasGridParameters = true;
			}

			// This also repairs a newly-created stencil material if the Mask hierarchy
			// changed since the prior update. Usually this is a cheap reference check.
			SyncMaterialState(changed);
		}

		private static bool HasChanged(Vector4 a, Vector4 b)
		{
			// Unity's Vector4 equality operator is approximate. Preserve small phase
			// changes while panning so the grid never visibly sticks and then jumps.
			return a.x != b.x || a.y != b.y || a.z != b.z || a.w != b.w;
		}

		public void SetGridEnabled(bool enabledState)
		{
			if (_gridEnabled == enabledState)
				return;

			_gridEnabled = enabledState;
			SyncMaterialState(true);
		}

		private void SyncMaterialState(bool forceRenderingMaterialUpdate)
		{
			if (_runtimeMaterial == null)
				return;

			if (forceRenderingMaterialUpdate)
				ApplyMaterialState(_runtimeMaterial);

			Material renderingMaterial = materialForRendering;
			if (renderingMaterial != null && renderingMaterial != _runtimeMaterial &&
				(forceRenderingMaterialUpdate || renderingMaterial != _lastRenderingMaterial))
			{
				ApplyMaterialState(renderingMaterial);
			}

			_lastRenderingMaterial = renderingMaterial;
		}

		private void ApplyMaterialState(Material target)
		{
			if (target == null)
				return;

			if (_hasGridParameters)
			{
				target.SetVector(GridPhaseID, _currentPhase);
				target.SetVector(GridCellsID, _currentCells);
				target.SetFloat(MajorThicknessID, _currentMajorThickness);
				target.SetFloat(MinorThicknessID, _currentMinorThickness);
				target.SetColor(MajorColorID, _currentMajorColor);
				target.SetColor(MinorColorID, _currentMinorColor);
			}

			target.SetFloat(GridEnabledID, _gridEnabled ? 1f : 0f);
		}

		protected override void OnPopulateMesh(VertexHelper vertexHelper)
		{
			vertexHelper.Clear();
			if (!_isInitialized)
				return;

			Rect rect = GetPixelAdjustedRect();
			UIVertex vertex = UIVertex.simpleVert;
			vertex.color = Color.white;

			vertex.position = new Vector3(rect.xMin, rect.yMin, 0f);
			vertex.uv0 = new Vector2(0f, 0f);
			vertexHelper.AddVert(vertex);

			vertex.position = new Vector3(rect.xMin, rect.yMax, 0f);
			vertex.uv0 = new Vector2(0f, 1f);
			vertexHelper.AddVert(vertex);

			vertex.position = new Vector3(rect.xMax, rect.yMax, 0f);
			vertex.uv0 = new Vector2(1f, 1f);
			vertexHelper.AddVert(vertex);

			vertex.position = new Vector3(rect.xMax, rect.yMin, 0f);
			vertex.uv0 = new Vector2(1f, 0f);
			vertexHelper.AddVert(vertex);

			vertexHelper.AddTriangle(0, 1, 2);
			vertexHelper.AddTriangle(2, 3, 0);
		}

		protected override void OnDestroy()
		{
			base.OnDestroy();
			if (_runtimeMaterial != null)
			{
				if (Application.isPlaying)
					Destroy(_runtimeMaterial);
				else
					DestroyImmediate(_runtimeMaterial);
				_runtimeMaterial = null;
			}
		}
	}
}