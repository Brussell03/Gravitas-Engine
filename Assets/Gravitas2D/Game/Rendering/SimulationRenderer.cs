using System;
using System.Collections.Generic;
using Gravitas2D;
using GravitasDemo.Interaction;
using TMPro;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Pool;
using UnityEngine.Rendering;
using Unity.Profiling;
using System.Runtime.InteropServices;
using UnityEngine.UI;

namespace GravitasDemo.Rendering
{
	[RequireComponent(typeof(SimulationInteractionController))]
	public class SimulationRenderer : MonoBehaviour
	{
		public static SimulationRenderer Instance { get; private set; }

		[Header("References")]
		[SerializeField] private RectTransform _visualizationArea;
		[SerializeField] private RectTransform _bodiesContainer;
		[SerializeField] private RectTransform _trajectoriesContainer;
		[SerializeField] private CelestialMapObject _celestialPrefab;
		[SerializeField] private EntityGroupMapObject _groupPrefab;
		[SerializeField] private TextMeshProUGUI _simTimeText;
		[SerializeField] private List<Sprite> _bodyImageTypes;
		[SerializeField] private Material _planetShaderMaterial;

		[Header("View Configuration")]
		[Tooltip("Simulation distance from the center of the visualization area to the farthest edge (width or height).")]
		[SerializeField] private double _visualizationRadius = 1.496e11; // Default: ~1 AU in meters
		[SerializeField] private double2 _viewCenter = double2.zero;
		[Tooltip("Expands the culling bounds by this percentage to load objects before they appear on screen. (e.g., 0.2 = 20%)")]
		[SerializeField] private float _boundsBufferPercentage = 0.2f;
		[SerializeField] private float _minPixelSize = 8f;
		[SerializeField] private float _minPixelSizeProjectile = 6f;
		[SerializeField] private float _minPixelSizeCelestial = 32f;
		[SerializeField] private float _pixelSizeGroup = 16f;

		private Bounds2D _cachedBounds;
		private bool _isBoundsDirty = true;
		private TrajectoryRenderer _trajectoryRenderer;
		private bool _isSimulationSystemLoaded = false;

		// UI Event Caching
		private PointerEventData _pointerEventData;
		private List<RaycastResult> _raycastResults = new List<RaycastResult>(10);

		// Kept for interaction hit-testing. Populated in the same pass that builds GPU batches.
		private readonly List<MapObject> _cachedVisualsBuffer = new List<MapObject>(4096);

		[Header("GPU Map Rendering")]
		[SerializeField] private Material _mapObjectInstancedMaterial;
		[SerializeField] private Material _selectionRingInstancedMaterial;
		[SerializeField] private Color _selectionRingColor = new Color(1f, 0.85f, 0.2f, 0.95f);
		[SerializeField, Min(0f)] private float _selectionRingPaddingPixels = 4f;
		[SerializeField, Range(0.005f, 0.15f)] private float _selectionRingThickness = 0.035f;
		[SerializeField, Range(0, 31)] private int _mapRenderLayer = 0;
		[SerializeField, Min(1)] private int _renderTextureResolutionScale = 1;

		// Matrix-array RenderMeshInstanced draws are limited to 511 instances by default.
		// The main map uses RenderMeshPrimitives + a structured instance buffer instead,
		// so each sprite/material batch can be submitted in one draw with no 511-instance cap.
		private const int MaxInstancesPerDraw = 511; // Selection rings still use matrix-array instancing.
		private const int InitialSpriteBatchCapacity = 256;
		private const int GpuInstanceStride = sizeof(float) * 8; // Two float4 values; 16-byte-aligned structured-buffer stride.
		private const string MapObjectBufferShaderName = "GravitasDemo/MapObjectInstancedBuffer";
		private static readonly int MainTexID = Shader.PropertyToID("_MainTex");
		private static readonly int MapInstancesID = Shader.PropertyToID("_MapInstances");
		private static readonly int MapWorldOriginID = Shader.PropertyToID("_MapWorldOrigin");
		private static readonly int MapPositionRightID = Shader.PropertyToID("_MapPositionRight");
		private static readonly int MapPositionUpID = Shader.PropertyToID("_MapPositionUp");
		private static readonly int MapSizeRightID = Shader.PropertyToID("_MapSizeRight");
		private static readonly int MapSizeUpID = Shader.PropertyToID("_MapSizeUp");
		private static readonly int InstanceColorID = Shader.PropertyToID("_InstanceColor");

		[StructLayout(LayoutKind.Sequential, Pack = 4)]
		private struct MapGpuInstance
		{
			// Visualization-area local position, pixel diameter, and rotation in radians.
			public Vector4 PositionSizeRotation;
			// RGBA8-expanded normalized tint. Keeping this as float4 makes the
			// structured-buffer record 32 bytes and portable across graphics APIs.
			public Vector4 Color;
		}

		private sealed class SpriteBatch
		{
			public Sprite Sprite;
			public Mesh Mesh;
			public MapGpuInstance[] Instances = new MapGpuInstance[InitialSpriteBatchCapacity];
			public GraphicsBuffer InstanceBuffer;
			public readonly MaterialPropertyBlock Properties = new MaterialPropertyBlock();
			public RenderParams RenderParams;
			public int Count;
			public int LastUsedFrame = -1;
		}

		private readonly Dictionary<Sprite, SpriteBatch> _spriteBatches = new Dictionary<Sprite, SpriteBatch>();
		private readonly List<SpriteBatch> _activeSpriteBatches = new List<SpriteBatch>(32);
		private readonly Dictionary<Sprite, Mesh> _spriteMeshCache = new Dictionary<Sprite, Mesh>();
		private RectTransform _mapObjectUIContainer;
		private Camera _mapRenderCamera;
		private GameObject _mapRenderCameraObject;
		private RawImage _mapRenderImage;
		private Canvas _mapCanvas;
		private RenderTexture _mapRenderTexture;
		private Mesh _unitQuadMesh;
		private bool _ownsMapObjectMaterial;
		private bool _ownsSelectionRingMaterial;
		private bool _instancingSupported;
		private readonly Matrix4x4[] _selectionRingMatrices = new Matrix4x4[MaxInstancesPerDraw];
		private readonly Vector4[] _selectionRingColors = new Vector4[MaxInstancesPerDraw];
		private MaterialPropertyBlock _selectionRingProperties;
		private RenderParams _selectionRingRenderParams;
		private Color _cachedSelectionRingColor = new Color(float.NaN, float.NaN, float.NaN, float.NaN);
		private float _cachedSelectionRingThickness = float.NaN;

		// Cache the visualization area's basis once per frame, not once per instance.
		private Rect _mapFrameRect;
		private Vector3 _mapFrameCenterWorld;
		private Vector3 _mapFrameRight;
		private Vector3 _mapFrameUp;
		private Vector3 _mapFrameForward;
		private Quaternion _mapFrameRotation;
		private float _mapFrameSignedScaleX;
		private float _mapFrameSignedScaleY;
		private float _mapFrameAbsScaleX;
		private float _mapFrameAbsScaleY;
		private Vector3 _mapWorldOrigin;
		private Vector3 _mapPositionRight;
		private Vector3 _mapPositionUp;
		private Vector3 _mapSizeRight;
		private Vector3 _mapSizeUp;
		private Bounds _mapInstanceWorldBounds;
		private int _lastDisplayedSimulationSecond = int.MinValue;

		private static readonly ProfilerMarker LateUpdateMarker = new ProfilerMarker("Gravitas.Map.LateUpdate");
		private static readonly ProfilerMarker TransformAndBatchMarker = new ProfilerMarker("Gravitas.Map.TransformAndBatch");
		private static readonly ProfilerMarker SubmitBatchesMarker = new ProfilerMarker("Gravitas.Map.SubmitBatches");
		private static readonly ProfilerMarker SelectionBatchMarker = new ProfilerMarker("Gravitas.Map.SelectionBatches");
		private static readonly ProfilerMarker MapCameraRenderMarker = new ProfilerMarker("Gravitas.Map.CameraRender");
		private static readonly ProfilerMarker InstanceBufferUploadMarker = new ProfilerMarker("Gravitas.Map.InstanceBufferUpload");
		private static readonly ProfilerMarker RenderMeshPrimitivesMarker = new ProfilerMarker("Gravitas.Map.RenderMeshPrimitives");
		private static readonly ProfilerMarker TrajectoryUpdateMarker = new ProfilerMarker("Gravitas.Map.TrajectoryUpdate");

		public RectTransform VisualizationArea => _visualizationArea;
		public Bounds2D ViewBounds => _cachedBounds;
		public float BoundsBufferPercentage => _boundsBufferPercentage;
		public Sprite GetBodyImage(EntityType type) => _bodyImageTypes[(int)type];
		public Material PlanetShaderMaterial => _planetShaderMaterial;
		public IReadOnlyList<MapObject> VisibleMapObjects => _cachedVisualsBuffer;

		public float GetHoverMarginPixels()
		{
			float width = _visualizationArea != null ? _visualizationArea.rect.width : 0f;
			float pct = SimulationInteractionController.Instance != null
				? SimulationInteractionController.Instance.HoverMarginScreenPct
				: 0.01f;
			return width > 0f ? width * pct : 10f;
		}

		public double VisualizationRadius
		{
			get => _visualizationRadius;
			set
			{
				if (value <= 0) return;
				_visualizationRadius = value;
				_isBoundsDirty = true;
			}
		}

		public double2 ViewCenter
		{
			get => _viewCenter;
			set
			{
				_viewCenter = value;
				_isBoundsDirty = true;
			}
		}

		void Start()
		{
			// Initial validation check on start
			ValidateSetup();
			
			if (Instance != null && Instance != this)
			{
				Destroy(gameObject);
				return;
			}
			Instance = this;

			Debug.Log($"[{GetType().Name}] Initialized");

			_selectionRingProperties = new MaterialPropertyBlock();
			InitializeInstancedRendering();

			_sceneCelestialsPool = new ObjectPool<CelestialMapObject>(CreateCelestial, OnGetFromPool, OnReleaseToPool, OnDestroyPooledObject, _collectionCheck, _defaultCapacityCelestials, _maxSizeCelestials);

			_sceneEntityGroupsPool = new ObjectPool<EntityGroupMapObject>(CreateEntityGroup, OnGetFromPool, OnReleaseToPool, OnDestroyPooledObject, _collectionCheck, _defaultCapacityGroups, _maxSizeGroups);

			RecalculateBounds();

			_trajectoryRenderer = new TrajectoryRenderer(_trajectoriesContainer);

			SaveManager.Instance.OnLoad += SetIsLoaded;
			SaveManager.Instance.OnReset += ClearRendererState;
		}

		void OnDestroy()
		{
			if (_trajectoryRenderer != null)
				_trajectoryRenderer.Cleanup();

			CleanupInstancedRendering();

			if (SaveManager.Instance)
			{
				SaveManager.Instance.OnLoad -= SetIsLoaded;
				SaveManager.Instance.OnReset -= ClearRendererState;
			}

			
			if (Instance == this) Instance = null;
		}

		private void SetIsLoaded(bool isLoaded)
		{
			_isSimulationSystemLoaded = isLoaded;
		}

		private void ClearRendererState()
		{
			_isSimulationSystemLoaded = false;
		}

		void LateUpdate()
		{
			if (PhysicsManager2D.Instance == null || !PhysicsManager2D.Instance.IsInitialized ||
				_visualizationArea == null || (GameManager.Instance?.IsPaused ?? true))
				return;

			using (LateUpdateMarker.Auto())
			{
				if (_isBoundsDirty)
					RecalculateBounds();

				UpdateVisibleObjectTransforms();

				if (_trajectoryRenderer != null)
				{
					using (TrajectoryUpdateMarker.Auto())
						_trajectoryRenderer.UpdateTrajectories();
				}

				UpdateSimulationTimeText();
			}
		}

		private void UpdateSimulationTimeText()
		{
			if (_simTimeText == null)
				return;

			int wholeSeconds = (int)PhysicsManager2D.Instance.SimulationElapsedTime;
			if (wholeSeconds == _lastDisplayedSimulationSecond)
				return;

			_lastDisplayedSimulationSecond = wholeSeconds;
			TimeSpan time = TimeSpan.FromSeconds(wholeSeconds);
			_simTimeText.text = $"T+{(int)time.TotalHours:0000}:{time.Minutes:00}:{time.Seconds:00}";
		}

		private void ValidateSetup()
		{
			if (_visualizationRadius <= 0)
			{
				Debug.LogWarning($"[{GetType().Name}] _coverageBounds size is invalid or zero.", this);
			}

			if (_visualizationArea == null)
			{
				Debug.LogError($"[{GetType().Name}] Visualization Area RectTransform is not assigned.", this);
				return;
			}
		}

		public void RecalculateBounds()
		{
			if (_visualizationArea == null) return;

			float width = _visualizationArea.rect.width;
			float height = _visualizationArea.rect.height;

			if (width <= 0f || height <= 0f) return;

			double halfWidth;
			double halfHeight;

			// Scale extent based on whichever dimension is larger
			if (width >= height)
			{
				halfWidth = _visualizationRadius;
				halfHeight = _visualizationRadius * (height / (double)width);
			}
			else
			{
				halfHeight = _visualizationRadius;
				halfWidth = _visualizationRadius * (width / (double)height);
			}

			_cachedBounds = new Bounds2D(_viewCenter, halfWidth, halfHeight);

			_isBoundsDirty = false;
		}

		private void UpdateVisibleObjectTransforms()
		{
			_cachedVisualsBuffer.Clear();
			RenderInstancedMapObjects();
		}

		#region GPU Instanced Map Rendering

		private void InitializeInstancedRendering()
		{
			if (_visualizationArea == null)
				return;

			_mapCanvas = _visualizationArea.GetComponentInParent<Canvas>();
			_instancingSupported = SystemInfo.supportsInstancing && SystemInfo.graphicsShaderLevel >= 45;
			if (!_instancingSupported)
				Debug.LogError($"[{GetType().Name}] This platform requires GPU instancing and shader model 4.5 for the structured-buffer map renderer.", this);

			// RenderMeshPrimitives reads compact per-instance data from a StructuredBuffer.
			// The previous matrix-array shader is not compatible with this path.
			if (_mapObjectInstancedMaterial != null &&
				(_mapObjectInstancedMaterial.shader == null || _mapObjectInstancedMaterial.shader.name != MapObjectBufferShaderName))
			{
				Debug.LogWarning($"[{GetType().Name}] Replacing the legacy map-instancing material with {MapObjectBufferShaderName}.", this);
				_mapObjectInstancedMaterial = null;
			}

			if (_mapObjectInstancedMaterial == null)
			{
				Shader shader = Shader.Find(MapObjectBufferShaderName);
				if (shader != null)
				{
					_mapObjectInstancedMaterial = new Material(shader);
					_ownsMapObjectMaterial = true;
				}
			}

			if (_mapObjectInstancedMaterial != null)
				_mapObjectInstancedMaterial.enableInstancing = true;
			else
				Debug.LogError($"[{GetType().Name}] Missing {MapObjectBufferShaderName} shader.", this);

			if (_selectionRingInstancedMaterial == null)
			{
				Shader ringShader = Shader.Find("GravitasDemo/MapObjectSelectionRing");
				if (ringShader != null)
				{
					_selectionRingInstancedMaterial = new Material(ringShader);
					_ownsSelectionRingMaterial = true;
				}
			}
			if (_selectionRingInstancedMaterial != null)
				_selectionRingInstancedMaterial.enableInstancing = true;
			else
				Debug.LogWarning($"[{GetType().Name}] Selection-ring shader not found; normal entities will not have instanced selection outlines.", this);

			_unitQuadMesh = CreateUnitQuadMesh();

			// One shared UI parent preserves the visualization area's local coordinate system
			// for celestial/group Graphics. This is a single container, not one object per entity.
			GameObject mapUIContainer = new GameObject("Map Object UI Container", typeof(RectTransform));
			_mapObjectUIContainer = mapUIContainer.GetComponent<RectTransform>();
			_mapObjectUIContainer.SetParent(_bodiesContainer != null ? _bodiesContainer : _visualizationArea, false);
			_mapObjectUIContainer.anchorMin = Vector2.zero;
			_mapObjectUIContainer.anchorMax = Vector2.one;
			_mapObjectUIContainer.offsetMin = Vector2.zero;
			_mapObjectUIContainer.offsetMax = Vector2.zero;
			_mapObjectUIContainer.localPosition = Vector3.zero;
			_mapObjectUIContainer.localRotation = Quaternion.identity;
			_mapObjectUIContainer.localScale = Vector3.one;

			GameObject cameraObject = new GameObject("Instanced Map Render Camera");
			cameraObject.transform.SetParent(transform, false);
			_mapRenderCameraObject = cameraObject;
			_mapRenderCamera = cameraObject.AddComponent<Camera>();
			_mapRenderCamera.enabled = false; // Render explicitly after submitting this frame's batches.
			_mapRenderCamera.orthographic = true;
			_mapRenderCamera.clearFlags = CameraClearFlags.SolidColor;
			_mapRenderCamera.backgroundColor = new Color(0f, 0f, 0f, 0f);
			_mapRenderCamera.cullingMask = 1 << _mapRenderLayer;
			_mapRenderCamera.useOcclusionCulling = false;
			_mapRenderCamera.allowHDR = false;
			_mapRenderCamera.allowMSAA = false;
			_mapRenderCamera.nearClipPlane = 0.1f;
			_mapRenderCamera.farClipPlane = 50f;

			Transform imageParent = _mapObjectUIContainer != null ? _mapObjectUIContainer : _visualizationArea;
			GameObject imageObject = new GameObject("Instanced Map Output", typeof(RectTransform), typeof(RawImage));
			RectTransform imageRect = imageObject.GetComponent<RectTransform>();
			imageRect.SetParent(imageParent, false);
			imageRect.anchorMin = Vector2.zero;
			imageRect.anchorMax = Vector2.one;
			imageRect.offsetMin = Vector2.zero;
			imageRect.offsetMax = Vector2.zero;
			imageRect.localPosition = Vector3.zero;
			imageRect.localRotation = Quaternion.identity;
			imageRect.localScale = Vector3.one;

			_mapRenderImage = imageObject.GetComponent<RawImage>();
			_mapRenderImage.color = Color.white;
			_mapRenderImage.raycastTarget = false;
			_mapRenderImage.maskable = true;
			_mapRenderImage.transform.SetAsFirstSibling();

			CacheMapFrameTransform();
			RefreshMapRenderTexture();
			UpdateMapCameraTransform();
		}

		private void CleanupInstancedRendering()
		{
			if (_mapRenderCamera != null)
				_mapRenderCamera.targetTexture = null;

			if (_mapRenderTexture != null)
			{
				_mapRenderTexture.Release();
				if (Application.isPlaying) Destroy(_mapRenderTexture);
				else DestroyImmediate(_mapRenderTexture);
				_mapRenderTexture = null;
			}

			foreach (var pair in _spriteMeshCache)
			{
				if (pair.Value == null) continue;
				if (Application.isPlaying) Destroy(pair.Value);
				else DestroyImmediate(pair.Value);
			}
			_spriteMeshCache.Clear();
			foreach (SpriteBatch batch in _spriteBatches.Values)
				batch?.InstanceBuffer?.Release();
			_spriteBatches.Clear();
			_activeSpriteBatches.Clear();

			if (_unitQuadMesh != null)
			{
				if (Application.isPlaying) Destroy(_unitQuadMesh);
				else DestroyImmediate(_unitQuadMesh);
				_unitQuadMesh = null;
			}

			if (_ownsMapObjectMaterial && _mapObjectInstancedMaterial != null)
			{
				if (Application.isPlaying) Destroy(_mapObjectInstancedMaterial);
				else DestroyImmediate(_mapObjectInstancedMaterial);
			}

			if (_ownsSelectionRingMaterial && _selectionRingInstancedMaterial != null)
			{
				if (Application.isPlaying) Destroy(_selectionRingInstancedMaterial);
				else DestroyImmediate(_selectionRingInstancedMaterial);
			}

			if (_mapRenderCameraObject != null)
			{
				if (Application.isPlaying) Destroy(_mapRenderCameraObject);
				else DestroyImmediate(_mapRenderCameraObject);
			}

			if (_mapObjectUIContainer != null)
			{
				if (Application.isPlaying) Destroy(_mapObjectUIContainer.gameObject);
				else DestroyImmediate(_mapObjectUIContainer.gameObject);
				_mapObjectUIContainer = null;
			}

		}

		private void RefreshMapRenderTexture()
		{
			if (_visualizationArea == null || _mapRenderCamera == null)
				return;

			Rect rect = _mapFrameRect;
			float canvasScale = _mapCanvas != null ? _mapCanvas.scaleFactor : 1f;
			float scaleX = Mathf.Max(0.0001f, _mapFrameAbsScaleX);
			float scaleY = Mathf.Max(0.0001f, _mapFrameAbsScaleY);
			int width = Mathf.Clamp(
				Mathf.CeilToInt(rect.width * canvasScale * scaleX * _renderTextureResolutionScale),
				1, SystemInfo.maxTextureSize);
			int height = Mathf.Clamp(
				Mathf.CeilToInt(rect.height * canvasScale * scaleY * _renderTextureResolutionScale),
				1, SystemInfo.maxTextureSize);

			if (_mapRenderTexture != null && _mapRenderTexture.width == width && _mapRenderTexture.height == height)
				return;

			if (_mapRenderTexture != null)
			{
				_mapRenderTexture.Release();
				if (Application.isPlaying) Destroy(_mapRenderTexture);
				else DestroyImmediate(_mapRenderTexture);
			}

			_mapRenderTexture = new RenderTexture(width, height, 0, RenderTextureFormat.ARGB32)
			{
				name = "Instanced Map Render Texture",
				filterMode = FilterMode.Bilinear,
				wrapMode = TextureWrapMode.Clamp,
				useMipMap = false,
				autoGenerateMips = false,
				antiAliasing = 1
			};
			_mapRenderTexture.Create();
			_mapRenderCamera.targetTexture = _mapRenderTexture;
			if (_mapRenderImage != null)
				_mapRenderImage.texture = _mapRenderTexture;
		}

		private void CacheMapFrameTransform()
		{
			_mapFrameRect = _visualizationArea.rect;
			Vector3 scale = _visualizationArea.lossyScale;
			_mapFrameSignedScaleX = scale.x;
			_mapFrameSignedScaleY = scale.y;
			_mapFrameAbsScaleX = Mathf.Abs(scale.x);
			_mapFrameAbsScaleY = Mathf.Abs(scale.y);
			_mapFrameRotation = _visualizationArea.rotation;
			_mapFrameRight = _mapFrameRotation * Vector3.right;
			_mapFrameUp = _mapFrameRotation * Vector3.up;
			_mapFrameForward = _mapFrameRotation * Vector3.forward;
			_mapFrameCenterWorld = _visualizationArea.TransformPoint(_mapFrameRect.center);

			// The buffer shader uses these five shared vectors plus per-instance local data.
			// This is algebraically equivalent to the old per-instance Matrix4x4.TRS path,
			// including non-uniform and negative visualization-area scales.
			_mapPositionRight = _mapFrameRight * _mapFrameSignedScaleX;
			_mapPositionUp = _mapFrameUp * _mapFrameSignedScaleY;
			_mapSizeRight = _mapFrameRight * _mapFrameAbsScaleX;
			_mapSizeUp = _mapFrameUp * _mapFrameAbsScaleY;
			_mapWorldOrigin = _mapFrameCenterWorld
				- _mapPositionRight * _mapFrameRect.center.x
				- _mapPositionUp * _mapFrameRect.center.y;
		}

		private void UpdateMapCameraTransform()
		{
			if (_mapRenderCamera == null || _visualizationArea == null)
				return;

			_mapRenderCamera.transform.SetPositionAndRotation(
				_mapFrameCenterWorld - _mapFrameForward * 20f, _mapFrameRotation);
			float cameraScaleX = Mathf.Max(0.0001f, _mapFrameAbsScaleX);
			float cameraScaleY = Mathf.Max(0.0001f, _mapFrameAbsScaleY);
			_mapRenderCamera.orthographicSize = _mapFrameRect.height * cameraScaleY * 0.5f;
			_mapRenderCamera.aspect = (_mapFrameRect.width * cameraScaleX) /
				Mathf.Max(0.0001f, _mapFrameRect.height * cameraScaleY);
		}

		/// <summary>
		/// Builds an equivalent world matrix using the cached visualization-area basis.
		/// Avoids TransformPoint, lossyScale, Quaternion.Euler and TRS per instance.
		/// </summary>
		private Matrix4x4 GetMapObjectMatrix(MapObject mapObject)
		{
			return GetMapObjectMatrix(mapObject, mapObject.PixelDiameter, mapObject.RenderRotationDegrees);
		}

		private Matrix4x4 GetMapObjectMatrix(MapObject mapObject, float diameter, float rotationDegrees)
		{
			float radians = rotationDegrees * Mathf.Deg2Rad;
			math.sincos(radians, out float sine, out float cosine);

			Vector3 xAxis = (_mapFrameRight * cosine + _mapFrameUp * sine) * (diameter * _mapFrameAbsScaleX);
			Vector3 yAxis = (_mapFrameRight * -sine + _mapFrameUp * cosine) * (diameter * _mapFrameAbsScaleY);
			Vector2 viewPosition = mapObject.ViewPosition;
			Vector3 position = _mapFrameCenterWorld
				+ _mapFrameRight * ((viewPosition.x - _mapFrameRect.center.x) * _mapFrameSignedScaleX)
				+ _mapFrameUp * ((viewPosition.y - _mapFrameRect.center.y) * _mapFrameSignedScaleY);

			Matrix4x4 matrix = Matrix4x4.identity;
			matrix.m00 = xAxis.x; matrix.m10 = xAxis.y; matrix.m20 = xAxis.z;
			matrix.m01 = yAxis.x; matrix.m11 = yAxis.y; matrix.m21 = yAxis.z;
			matrix.m02 = _mapFrameForward.x;
			matrix.m12 = _mapFrameForward.y;
			matrix.m22 = _mapFrameForward.z;
			matrix.m03 = position.x; matrix.m13 = position.y; matrix.m23 = position.z;
			return matrix;
		}

		private void RenderInstancedMapObjects()
		{
			if (_mapRenderCamera == null || _visualizationArea == null)
				return;

			using (TransformAndBatchMarker.Auto())
			{
				CacheMapFrameTransform();
				RefreshMapRenderTexture();
				UpdateMapCameraTransform();

				_activeSpriteBatches.Clear();
				int frame = Time.frameCount;
				float maxInstanceDiameter = 0f;
				bool canReadEntities = _isSimulationSystemLoaded &&
					EntitiesManager.Instance != null && EntityGroupsManager.Instance != null;

				if (canReadEntities)
				{
					Rect rect = _mapFrameRect;
					double minX = _cachedBounds.MinX;
					double minY = _cachedBounds.MinY;
					double sizeX = _cachedBounds.Size.x;
					double sizeY = _cachedBounds.Size.y;
					if (sizeX <= 0.0 || sizeY <= 0.0)
						canReadEntities = false;
					else
					{
						double inverseSizeX = 1.0 / sizeX;
						double inverseSizeY = 1.0 / sizeY;
						float rectWidth = rect.width;
						IReadOnlyCollection<Entity> visibleEntities = EntitiesManager.Instance.VisibleEntities;
						var visibleGroups = EntityGroupsManager.Instance.Groups;
						int estimatedVisibleCount = visibleEntities.Count + visibleGroups.Count;
						if (_cachedVisualsBuffer.Capacity < estimatedVisibleCount)
							_cachedVisualsBuffer.Capacity = Mathf.NextPowerOfTwo(Mathf.Max(estimatedVisibleCount, 16));

						foreach (Entity entity in visibleEntities)
						{
							if (entity == null || entity.IsDestroyed)
								continue;

							bool celestial = entity.IsCelestial;
							EntityMapObject normalVisual = null;
							MapObject mapObject;
							if (celestial)
							{
								mapObject = EntitiesManager.Instance.GetCelestialVisualForEntity(entity);
								if (mapObject == null || !mapObject.IsVisualActive)
									continue;
							}
							else
							{
								normalVisual = EntitiesManager.Instance.GetVisualForEntity(entity);
								if (normalVisual == null)
									continue;
								mapObject = normalVisual;
								if (entity.IsObserved != normalVisual.LastObservedState)
									normalVisual.UpdateVisual();
							}

							double2 position = entity.Position;
							float percentX = (float)((position.x - minX) * inverseSizeX);
							float percentY = (float)((position.y - minY) * inverseSizeY);
							float minPixelSize = celestial ? _minPixelSizeCelestial : _minPixelSize;
							float radiusMultiplier = celestial ? 2.6f : 2.0f;
							double radius = entity.PhysicsBody.Radius;
							float pixelDiameter = Mathf.Max(
								(float)(radius * inverseSizeX) * rectWidth * radiusMultiplier,
								minPixelSize);
							float rotation = (float)entity.Orientation;

							mapObject.ApplyTransformData(
								percentX, percentY, pixelDiameter, rotation, sizeX,
								rect.xMin, rect.xMax, rect.yMin, rect.yMax);
							_cachedVisualsBuffer.Add(mapObject);

							if (normalVisual != null)
							{
								AppendEntityInstance(normalVisual, frame);
								if (normalVisual.PixelDiameter > maxInstanceDiameter)
									maxInstanceDiameter = normalVisual.PixelDiameter;
							}
						}

						const double groupRadius = 50.0;
						foreach (EntityGroup group in visibleGroups)
						{
							if (group == null || group.IsDestroyed)
								continue;

							EntityGroupMapObject groupVisual = EntityGroupsManager.Instance.GetVisualForGroup(group);
							if (groupVisual == null || !groupVisual.IsVisualActive)
								continue;

							double2 position = group.Position;
							float percentX = (float)((position.x - minX) * inverseSizeX);
							float percentY = (float)((position.y - minY) * inverseSizeY);
							float pixelDiameter = Mathf.Max(
								(float)(groupRadius * inverseSizeX) * rectWidth * 2f, _minPixelSize);

							groupVisual.ApplyTransformData(
								percentX, percentY, pixelDiameter, 0f, sizeX,
								rect.xMin, rect.xMax, rect.yMin, rect.yMax);
							_cachedVisualsBuffer.Add(groupVisual);
						}

						UpdateMapInstanceWorldBounds(maxInstanceDiameter);
					}
				}
			}

			using (SubmitBatchesMarker.Auto())
			{
				for (int i = 0; i < _activeSpriteBatches.Count; i++)
					FlushSpriteBatch(_activeSpriteBatches[i]);
			}

			bool hasSelectionRings;
			using (SelectionBatchMarker.Auto())
				hasSelectionRings = RenderSelectedEntityIndicators();

			bool hasRenderedGeometry = _activeSpriteBatches.Count > 0 || hasSelectionRings;
			if (_mapRenderImage != null)
				_mapRenderImage.enabled = hasRenderedGeometry;

			// A camera render clears and redraws the entire target. When there are no map
			// instances or selection rings, hide the RawImage and skip that camera entirely.
			if (hasRenderedGeometry)
			{
				using (MapCameraRenderMarker.Auto())
					_mapRenderCamera.Render();
			}
		}

		private void AppendEntityInstance(EntityMapObject entityVisual, int frame)
		{
			if (entityVisual.PixelDiameter <= 0f || _mapObjectInstancedMaterial == null || !_instancingSupported)
				return;

			Vector2 position = entityVisual.ViewPosition;
			float visualRadius = entityVisual.PixelDiameter * 0.70710678f; // Half-diagonal of the rotated square sprite bounds.
			Rect viewRect = _mapFrameRect;
			// EntitiesManager intentionally keeps a buffered region around the viewport.
			// Do not upload instances whose complete sprite is outside the render camera.
			// They remain in _cachedVisualsBuffer so interaction/protection state is unchanged.
			if (position.x + visualRadius < viewRect.xMin || position.x - visualRadius > viewRect.xMax ||
				position.y + visualRadius < viewRect.yMin || position.y - visualRadius > viewRect.yMax)
				return;

			Sprite sprite = entityVisual.RenderSprite;
			if (sprite == null)
				return;

			if (!_spriteBatches.TryGetValue(sprite, out SpriteBatch batch))
			{
				Mesh spriteMesh = GetOrCreateSpriteMesh(sprite);
				if (spriteMesh == null)
					return;

				batch = new SpriteBatch { Sprite = sprite, Mesh = spriteMesh };
				batch.InstanceBuffer = new GraphicsBuffer(
					GraphicsBuffer.Target.Structured, batch.Instances.Length, GpuInstanceStride);
				batch.Properties.SetTexture(MainTexID, sprite.texture);
				batch.Properties.SetBuffer(MapInstancesID, batch.InstanceBuffer);
				batch.RenderParams = new RenderParams(_mapObjectInstancedMaterial)
				{
					camera = _mapRenderCamera,
					layer = _mapRenderLayer,
					matProps = batch.Properties,
					shadowCastingMode = ShadowCastingMode.Off,
					receiveShadows = false
				};
				_spriteBatches.Add(sprite, batch);
			}

			if (batch.LastUsedFrame != frame)
			{
				batch.LastUsedFrame = frame;
				batch.Count = 0;
				_activeSpriteBatches.Add(batch);
			}

			EnsureSpriteBatchCapacity(batch, batch.Count + 1);
			Color32 color = entityVisual.RenderColor;
			batch.Instances[batch.Count++] = new MapGpuInstance
			{
				PositionSizeRotation = new Vector4(
					position.x,
					position.y,
					entityVisual.PixelDiameter,
					entityVisual.RenderRotationDegrees * Mathf.Deg2Rad),
				Color = new Vector4(
					color.r / 255f,
					color.g / 255f,
					color.b / 255f,
					color.a / 255f)
			};
		}

		private void EnsureSpriteBatchCapacity(SpriteBatch batch, int requiredCapacity)
		{
			if (requiredCapacity <= batch.Instances.Length)
				return;

			int newCapacity = Mathf.NextPowerOfTwo(requiredCapacity);
			Array.Resize(ref batch.Instances, newCapacity);

			GraphicsBuffer newBuffer = new GraphicsBuffer(
				GraphicsBuffer.Target.Structured, newCapacity, GpuInstanceStride);
			if (batch.Count > 0)
				newBuffer.SetData(batch.Instances, 0, 0, batch.Count);

			GraphicsBuffer oldBuffer = batch.InstanceBuffer;
			batch.InstanceBuffer = newBuffer;
			batch.Properties.SetBuffer(MapInstancesID, newBuffer);
			oldBuffer?.Release();
		}

		private void UpdateMapInstanceWorldBounds(float maxInstanceDiameter)
		{
			float expansion = 0.5f * (1f + Mathf.Max(0f, _boundsBufferPercentage));
			float rotatedSpriteExtent = maxInstanceDiameter * 0.70710678f; // Conservative half-diagonal for any sprite rotation.
			float halfWidth = _mapFrameRect.width * expansion + rotatedSpriteExtent;
			float halfHeight = _mapFrameRect.height * expansion + rotatedSpriteExtent;

			Vector3 extents = Abs(_mapPositionRight) * halfWidth +
				Abs(_mapPositionUp) * halfHeight + Abs(_mapFrameForward) * 1f;
			_mapInstanceWorldBounds = new Bounds(_mapFrameCenterWorld, extents * 2f);
		}

		private static Vector3 Abs(Vector3 value)
		{
			return new Vector3(Mathf.Abs(value.x), Mathf.Abs(value.y), Mathf.Abs(value.z));
		}

		/// <summary>
		/// Draws selection outlines for data-only entities. This keeps the normal-entity
		/// path entirely free of per-entity UI indicator GameObjects as well as visual GameObjects.
		/// </summary>
		private bool RenderSelectedEntityIndicators()
		{
			if (_selectionRingInstancedMaterial == null || !_instancingSupported ||
				SimulationInteractionController.Instance == null)
				return false;

			UpdateSelectionRingPropertiesIfNeeded();
			IReadOnlyList<MapObject> selected = SimulationInteractionController.Instance.SelectedMapObjects;
			int count = 0;
			bool drewAny = false;
			for (int i = 0; i < selected.Count; i++)
			{
				if (!(selected[i] is EntityMapObject entityVisual) ||
					!CircularRaycastTarget.IsAlive(entityVisual) || entityVisual.PixelDiameter <= 0f)
					continue;

				_selectionRingMatrices[count] = GetMapObjectMatrix(
					entityVisual, entityVisual.PixelDiameter + 2f * _selectionRingPaddingPixels, 0f);
				count++;

				if (count == MaxInstancesPerDraw)
				{
					FlushSelectionRingBatch(count);
					drewAny = true;
					count = 0;
				}
			}

			if (count > 0)
			{
				FlushSelectionRingBatch(count);
				drewAny = true;
			}

			return drewAny;
		}

		private void UpdateSelectionRingPropertiesIfNeeded()
		{
			if (_selectionRingProperties == null ||
				(_cachedSelectionRingColor == _selectionRingColor && Mathf.Approximately(_cachedSelectionRingThickness, _selectionRingThickness)))
				return;

			_cachedSelectionRingColor = _selectionRingColor;
			_cachedSelectionRingThickness = _selectionRingThickness;
			Vector4 color = _selectionRingColor;
			for (int i = 0; i < _selectionRingColors.Length; i++)
				_selectionRingColors[i] = color;

			_selectionRingProperties.Clear();
			_selectionRingProperties.SetVectorArray(InstanceColorID, _selectionRingColors);
			_selectionRingProperties.SetFloat("_RingRadius", 0.465f);
			_selectionRingProperties.SetFloat("_RingHalfWidth", _selectionRingThickness * 0.5f);
			_selectionRingRenderParams = new RenderParams(_selectionRingInstancedMaterial)
			{
				camera = _mapRenderCamera,
				layer = _mapRenderLayer,
				matProps = _selectionRingProperties,
				shadowCastingMode = ShadowCastingMode.Off,
				receiveShadows = false
			};
		}

		private void FlushSelectionRingBatch(int count)
		{
			Graphics.RenderMeshInstanced(
				_selectionRingRenderParams, _unitQuadMesh, 0, _selectionRingMatrices, count);
		}

		private void FlushSpriteBatch(SpriteBatch batch)
		{
			if (batch == null || batch.Count <= 0 || batch.Mesh == null || batch.Sprite == null || batch.InstanceBuffer == null)
				return;

			using (InstanceBufferUploadMarker.Auto())
				batch.InstanceBuffer.SetData(batch.Instances, 0, 0, batch.Count);

			batch.Properties.SetVector(MapWorldOriginID, _mapWorldOrigin);
			batch.Properties.SetVector(MapPositionRightID, _mapPositionRight);
			batch.Properties.SetVector(MapPositionUpID, _mapPositionUp);
			batch.Properties.SetVector(MapSizeRightID, _mapSizeRight);
			batch.Properties.SetVector(MapSizeUpID, _mapSizeUp);
			batch.RenderParams.worldBounds = _mapInstanceWorldBounds;

			using (RenderMeshPrimitivesMarker.Auto())
				Graphics.RenderMeshPrimitives(batch.RenderParams, batch.Mesh, 0, batch.Count);
			batch.Count = 0;
		}

		private Mesh GetOrCreateSpriteMesh(Sprite sprite)
		{
			if (sprite == null)
				return null;
			if (_spriteMeshCache.TryGetValue(sprite, out Mesh cachedMesh))
				return cachedMesh;

			Vector2[] sourceVertices = sprite.vertices;
			Vector2[] sourceUVs = sprite.uv;
			ushort[] sourceTriangles = sprite.triangles;
			if (sourceVertices == null || sourceUVs == null || sourceTriangles == null ||
				sourceVertices.Length < 3 || sourceUVs.Length != sourceVertices.Length || sourceTriangles.Length < 3)
				return _unitQuadMesh;

			Bounds bounds = sprite.bounds;
			float width = Mathf.Max(bounds.size.x, 0.000001f);
			float height = Mathf.Max(bounds.size.y, 0.000001f);
			Vector3[] vertices = new Vector3[sourceVertices.Length];
			for (int i = 0; i < sourceVertices.Length; i++)
			{
				vertices[i] = new Vector3(
					(sourceVertices[i].x - bounds.center.x) / width,
					(sourceVertices[i].y - bounds.center.y) / height,
					0f);
			}

			int[] triangles = new int[sourceTriangles.Length];
			for (int i = 0; i < sourceTriangles.Length; i++)
				triangles[i] = sourceTriangles[i];

			Mesh mesh = new Mesh
			{
				name = $"Instanced Sprite Mesh - {sprite.name}",
				 hideFlags = HideFlags.DontSave
			};
			mesh.vertices = vertices;
			mesh.uv = sourceUVs;
			mesh.triangles = triangles;
			mesh.RecalculateBounds();
			_spriteMeshCache.Add(sprite, mesh);
			return mesh;
		}

		private static Mesh CreateUnitQuadMesh()
		{
			Mesh mesh = new Mesh { name = "Instanced Map Unit Quad", hideFlags = HideFlags.DontSave };
			mesh.vertices = new[]
			{
				new Vector3(-0.5f, -0.5f, 0f),
				new Vector3( 0.5f, -0.5f, 0f),
				new Vector3( 0.5f,  0.5f, 0f),
				new Vector3(-0.5f,  0.5f, 0f)
			};
			mesh.uv = new[]
			{
				new Vector2(0f, 0f),
				new Vector2(1f, 0f),
				new Vector2(1f, 1f),
				new Vector2(0f, 1f)
			};
			mesh.triangles = new[] { 0, 1, 2, 0, 2, 3 };
			mesh.RecalculateBounds();
			return mesh;
		}

		#endregion

		#region Public API

		/// <summary>
		/// Validates whether the pointer is physically occluded by a UI panel layered on top of the visualization area.
		/// </summary>
		private bool IsPointerOccludedByUI()
		{
			if (EventSystem.current == null) return false;

			// Lazy initialize to prevent garbage generation
			if (_pointerEventData == null)
			{
				_pointerEventData = new PointerEventData(EventSystem.current);
			}
			
			_pointerEventData.position = Input.mousePosition;

			_raycastResults.Clear();
			EventSystem.current.RaycastAll(_pointerEventData, _raycastResults);

			// Results are sorted by depth, top-most first.
			if (_raycastResults.Count > 0)
			{
				GameObject topHit = _raycastResults[0].gameObject;
				
				// Evaluate if the highest depth hit is completely outside the visualization area's structural hierarchy
				if (topHit != _visualizationArea.gameObject && !topHit.transform.IsChildOf(_visualizationArea))
				{
					return true;
				}
			}

			return false;
		}

		public Vector2 SimPositionToViewUI(double2 simPosition)
		{
			float percX = (float)((simPosition.x - _cachedBounds.MinX) / _cachedBounds.Size.x);
			float percY = (float)((simPosition.y - _cachedBounds.MinY) / _cachedBounds.Size.y);
			
			float localX = Mathf.LerpUnclamped(_visualizationArea.rect.xMin, _visualizationArea.rect.xMax, percX);
			float localY = Mathf.LerpUnclamped(_visualizationArea.rect.yMin, _visualizationArea.rect.yMax, percY);
			
			return new Vector2(localX, localY);
		}

		public Vector2 SimPositionToViewUIClamped(double2 simPosition)
		{
			float percX = (float)((simPosition.x - _cachedBounds.MinX) / _cachedBounds.Size.x);
			float percY = (float)((simPosition.y - _cachedBounds.MinY) / _cachedBounds.Size.y);
			
			float localX = Mathf.Lerp(_visualizationArea.rect.xMin, _visualizationArea.rect.xMax, percX);
			float localY = Mathf.Lerp(_visualizationArea.rect.yMin, _visualizationArea.rect.yMax, percY);
			
			return new Vector2(localX, localY);
		}

		public bool TryGetMouseSimPosition(out double2 mouseWorldPos, out Vector2 localUIPos)
		{
			mouseWorldPos = double2.zero;
			localUIPos = Vector2.zero;

			if (_visualizationArea == null) return false;

			Canvas canvas = _visualizationArea.GetComponentInParent<Canvas>();
			Camera cam = (canvas != null && canvas.renderMode != RenderMode.ScreenSpaceOverlay) ? canvas.worldCamera : null;

			if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(_visualizationArea, Input.mousePosition, cam, out localUIPos))
				return false;
			
			// Validate the projected point actually resides within the UI element's physical bounds
			if (!_visualizationArea.rect.Contains(localUIPos))
				return false;

			Rect rect = _visualizationArea.rect;
			double percX = (double)(localUIPos.x - rect.xMin) / (double)rect.width;
			double percY = (double)(localUIPos.y - rect.yMin) / (double)rect.height;

			mouseWorldPos = new double2(
				_viewCenter.x + ((percX - 0.5) * _cachedBounds.Size.x),
				_viewCenter.y + ((percY - 0.5) * _cachedBounds.Size.y)
			);

			if (IsPointerOccludedByUI())
				return false;

			return true;
		}

		/// <summary>
		/// Converts a local UI position within the visualization area into absolute simulation space coordinates.
		/// </summary>
		public double2 ViewUIToSimPosition(Vector2 localUIPos)
		{
			// Prevent division by zero if UI is uninitialized or collapsed
			if (_visualizationArea == null || _visualizationArea.rect.width <= 0f || _visualizationArea.rect.height <= 0f)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot convert UI position to Sim position: Visualization area is invalid.");
				return _viewCenter;
			}

			// Calculate normalized coordinates (0.0 to 1.0) within the UI rect bounds
			Rect rect = _visualizationArea.rect;
			double percX = (double)(localUIPos.x - rect.xMin) / (double)rect.width;
			double percY = (double)(localUIPos.y - rect.yMin) / (double)rect.height;

			// Project normalized coordinates into absolute simulation bounds
			return new double2(
				_cachedBounds.MinX + (percX * _cachedBounds.Size.x),
				_cachedBounds.MinY + (percY * _cachedBounds.Size.y)
			);
		}

		public bool TryGetMouseViewPosition(out Vector2 localUIPos)
		{
			localUIPos = Vector2.zero;

			if (_visualizationArea == null)
			{
				Debug.LogError($"[{GetType().Name}] VisualizationArea RectTransform is null.");
				return false;
			}

			Canvas canvas = _visualizationArea.GetComponentInParent<Canvas>();
			Camera cam = (canvas != null && canvas.renderMode != RenderMode.ScreenSpaceOverlay) ? canvas.worldCamera : null;

			// ScreenPointToLocalPointInRectangle calculates the point in _visualizationArea's actual local coordinate space
			if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(_visualizationArea, Input.mousePosition, cam, out localUIPos))
				return false;

			// Validate the projected point actually resides within the UI element's physical bounds
			if (!_visualizationArea.rect.Contains(localUIPos))
				return false;

			if (IsPointerOccludedByUI())
				return false;

			return true;
		}

		public void SetViewBounds(Bounds2D newBounds)
		{
			_cachedBounds = newBounds;

			// Synchronize the underlying source-of-truth variables so subsequent RecalculateBounds
			// calls (triggered by panning) retain the updated zoom level and anchor center.
			_viewCenter = new double2(
				newBounds.MinX + (newBounds.Size.x * 0.5),
				newBounds.MinY + (newBounds.Size.y * 0.5)
			);

			if (_visualizationArea != null && _visualizationArea.rect.width > 0 && _visualizationArea.rect.height > 0)
			{
				if (_visualizationArea.rect.width >= _visualizationArea.rect.height)
				{
					_visualizationRadius = newBounds.Size.x * 0.5;
				}
				else
				{
					_visualizationRadius = newBounds.Size.y * 0.5;
				}
			}

			_isBoundsDirty = false;
		}

		public void SetViewToSelected()
		{
			Entity entity = SimulationInteractionController.Instance != null
				? SimulationInteractionController.Instance.SelectedEntity
				: null;
			
			if (entity != null)
			{
				_viewCenter = entity.Position;
				_isBoundsDirty = true;
				if (entity != SimulationInteractionController.Instance.TrackedEntity) SimulationInteractionController.Instance.UntrackMapObject();
			}
		}

		#region UI Element Positioning

		/// <summary>
		/// Positions a target RectTransform relative to the visualization area based on a normalized (0.0 to 1.0) coordinate.
		/// Positions are applied via local coordinates, ignoring culling/visibility boundaries.
		/// </summary>
		public void SetUIPositionFromNormalized(RectTransform target, Vector2 normalizedPosition)
		{
			if (target == null || _visualizationArea == null)
			{
				Debug.LogError($"[{GetType().Name}] Target RectTransform or VisualizationArea is null.");
				return;
			}

			// Map normalized coordinates directly to the RectTransform's local rect space boundaries
			float localX = Mathf.LerpUnclamped(_visualizationArea.rect.xMin, _visualizationArea.rect.xMax, normalizedPosition.x);
			float localY = Mathf.LerpUnclamped(_visualizationArea.rect.yMin, _visualizationArea.rect.yMax, normalizedPosition.y);

			// Retain original Z depth to avoid breaking UI layer ordering
			target.localPosition = new Vector3(localX, localY, target.localPosition.z);
		}

		/// <summary>
		/// Positions a target RectTransform relative to the visualization area based on a world simulation coordinate.
		/// </summary>
		public void SetUIPositionFromSimulationPosition(RectTransform target, double2 simPosition)
		{
			if (target == null || _visualizationArea == null)
			{
				Debug.LogError($"[{GetType().Name}] Target RectTransform or VisualizationArea is null.");
				return;
			}

			// Leverage existing coordinate transformation routine to resolve the local UI point[cite: 18]
			Vector2 localUIPos = SimPositionToViewUI(simPosition);

			// Check if either X or Y is NaN
			if (float.IsNaN(localUIPos.x) || float.IsNaN(localUIPos.y)) return;
			
			target.localPosition = new Vector3(localUIPos.x, localUIPos.y, target.localPosition.z);
		}

		/// <summary>
		/// Checks if the current mouse position intersects the valid visualization area rect bounds.
		/// </summary>
		public bool IsMouseInVisualizationArea()
		{
			if (_visualizationArea == null)
				return false;

			if (IsPointerOccludedByUI())
				return false;

			// Determine correct camera context based on Canvas render mode to ensure accurate raycasting[cite: 18]
			Canvas canvas = _visualizationArea.GetComponentInParent<Canvas>();
			Camera cam = (canvas != null && canvas.renderMode != RenderMode.ScreenSpaceOverlay) ? canvas.worldCamera : null;

			return RectTransformUtility.RectangleContainsScreenPoint(_visualizationArea, Input.mousePosition, cam);
		}

		#endregion

		#endregion

		#region Object Pooling

		// throw an exception if we try to return an existing item, already in the pool
		private bool _collectionCheck = false;

		#region Celestial Pooling

		private IObjectPool<CelestialMapObject> _sceneCelestialsPool;
		private int _defaultCapacityCelestials = 16;
		private int _maxSizeCelestials = 128;

		public CelestialMapObject GetCelestialVisualFromPool(Entity entity)
		{
			if (_sceneCelestialsPool == null)
			{
				Debug.LogError($"[{GetType().Name}] Celestial pool is not initialized.", this);
				return null;
			}

			CelestialMapObject visual = _sceneCelestialsPool.Get();
			visual.InitializeBody(entity);
			return visual;
		}

		private CelestialMapObject CreateCelestial()
		{
			if (_celestialPrefab == null)
				throw new InvalidOperationException("SimulationRenderer requires a CelestialMapObject prefab.");

			Transform parent = _mapObjectUIContainer != null ? _mapObjectUIContainer : _visualizationArea;
			CelestialMapObject instance = Instantiate(_celestialPrefab, parent);
			instance.ObjectPool = _sceneCelestialsPool;
			return instance;
		}

		private void OnGetFromPool(CelestialMapObject pooledObject)
		{
			pooledObject.gameObject.SetActive(true);
		}

		private void OnReleaseToPool(CelestialMapObject pooledObject)
		{
			pooledObject.Deinitialize();
			pooledObject.gameObject.SetActive(false);
		}

		private void OnDestroyPooledObject(CelestialMapObject pooledObject)
		{
			pooledObject.Deinitialize();
			Destroy(pooledObject.gameObject);
		}

		#endregion

				#region Entity Group Pooling

		private IObjectPool<EntityGroupMapObject> _sceneEntityGroupsPool;

		// extra options to control the pool capacity and maximum size
		private int _defaultCapacityGroups = 100;
		private int _maxSizeGroups = 1000;
		
		public EntityGroupMapObject GetGroupFromPool(EntityGroup group)
		{
			var body = _sceneEntityGroupsPool.Get();
			body.InitializeBody(group);
			// UpdateGroupTransform(body, _visualizationArea.rect.width, _visualizationArea.rect.height, _cachedBounds.MinX, _cachedBounds.MinY, _cachedBounds.Size.x, _cachedBounds.Size.y);

			return body;
		}

		EntityGroupMapObject CreateEntityGroup()
		{
			EntityGroupMapObject bodyInstance = Instantiate(_groupPrefab, _mapObjectUIContainer != null ? _mapObjectUIContainer : _visualizationArea);
			bodyInstance.ObjectPool = _sceneEntityGroupsPool;

			return bodyInstance;
		}

		void OnGetFromPool(EntityGroupMapObject pooledObject)
		{
			pooledObject.gameObject.SetActive(true);
		}

		void OnReleaseToPool(EntityGroupMapObject pooledObject)
		{
			pooledObject.Deinitialize();
			pooledObject.gameObject.SetActive(false);
		}

		void OnDestroyPooledObject(EntityGroupMapObject pooledObject)
		{
			pooledObject.Deinitialize();
			Destroy(pooledObject.gameObject);
		}

		#endregion

		#endregion
	}

}
