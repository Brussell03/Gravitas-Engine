using System;
using System.Collections.Generic;
using Gravitas2D;
using GravitasDemo.Interaction;
using TMPro;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Pool;
using UnityEngine.Rendering;
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

		private NativeArray<double2> _transformPositions;
		private NativeArray<float> _transformRotations;
		private NativeArray<bool> _transformIsCelestial;
		private NativeArray<double> _transformRadii;
		private NativeArray<float> _transformOutPosX;
		private NativeArray<float> _transformOutPosY;
		private NativeArray<float> _transformOutScale;
		private NativeArray<float> _transformOutRot;
		private List<MapObject> _cachedVisualsBuffer = new List<MapObject>(1024);

		[Header("GPU Map Rendering")]
		[SerializeField] private Material _mapObjectInstancedMaterial;
		[SerializeField] private Material _selectionRingInstancedMaterial;
		[SerializeField] private Color _selectionRingColor = new Color(1f, 0.85f, 0.2f, 0.95f);
		[SerializeField, Min(0f)] private float _selectionRingPaddingPixels = 4f;
		[SerializeField, Range(0.005f, 0.15f)] private float _selectionRingThickness = 0.035f;
		[SerializeField, Range(0, 31)] private int _mapRenderLayer = 0;
		[SerializeField, Min(1)] private int _renderTextureResolutionScale = 1;

		// With the default object-to-world + world-to-object instance data, Unity limits
		// matrix-array instancing to 511 instances per draw.
		private const int MaxInstancesPerDraw = 511;
		private static readonly int MainTexID = Shader.PropertyToID("_MainTex");
		private static readonly int InstanceColorID = Shader.PropertyToID("_InstanceColor");

		private sealed class SpriteBatch
		{
			public Sprite Sprite;
			public Mesh Mesh;
			public readonly Matrix4x4[] Matrices = new Matrix4x4[MaxInstancesPerDraw];
			public readonly Vector4[] Colors = new Vector4[MaxInstancesPerDraw];
			public readonly MaterialPropertyBlock Properties = new MaterialPropertyBlock();
			public int Count;
		}

		private readonly Dictionary<Sprite, SpriteBatch> _spriteBatches = new Dictionary<Sprite, SpriteBatch>();
		private readonly List<SpriteBatch> _activeSpriteBatches = new List<SpriteBatch>(32);
		private readonly HashSet<SpriteBatch> _activeSpriteBatchSet = new HashSet<SpriteBatch>();
		private readonly Dictionary<Sprite, Mesh> _spriteMeshCache = new Dictionary<Sprite, Mesh>();
		private RectTransform _mapObjectUIContainer;
		private Camera _mapRenderCamera;
		private GameObject _mapRenderCameraObject;
		private RawImage _mapRenderImage;
		private RenderTexture _mapRenderTexture;
		private Mesh _unitQuadMesh;
		private bool _ownsMapObjectMaterial;
		private bool _ownsSelectionRingMaterial;
		private bool _instancingSupported;
		private readonly Matrix4x4[] _selectionRingMatrices = new Matrix4x4[MaxInstancesPerDraw];
		private readonly Vector4[] _selectionRingColors = new Vector4[MaxInstancesPerDraw];	
		private MaterialPropertyBlock _selectionRingProperties;

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

			if (_transformPositions.IsCreated)
			{
				_transformPositions.Dispose();
				_transformRotations.Dispose();
				_transformIsCelestial.Dispose();
				_transformRadii.Dispose();
				_transformOutPosX.Dispose();
				_transformOutPosY.Dispose();
				_transformOutScale.Dispose();
				_transformOutRot.Dispose();
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
			if (PhysicsManager2D.Instance == null || !PhysicsManager2D.Instance.IsInitialized || _visualizationArea == null || (GameManager.Instance?.IsPaused ?? true)) return;
			
			if (_isBoundsDirty)
			{
				RecalculateBounds();
			}

			UpdateVisibleObjectTransforms();
			_trajectoryRenderer.UpdateTrajectories();

			double time = PhysicsManager2D.Instance.SimulationElapsedTime;
			TimeSpan t = TimeSpan.FromSeconds(time);
			if (_simTimeText != null)
				_simTimeText.text = $"T+{(int)t.TotalHours:0000}:{t.Minutes:00}:{t.Seconds:00}";
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

		// private void UpdateVisibleObjectTransforms()
		// {
		// 	if (!_isSimulationSystemLoaded) return;

		// 	// Cache external C++ properties to prevent boundary crossing in the loop
		// 	float rectWidth = _visualizationArea.rect.width;
		// 	float rectHeight = _visualizationArea.rect.height;
		// 	double minX = _cachedBounds.MinX;
		// 	double minY = _cachedBounds.MinY;
		// 	double sizeX = _cachedBounds.Size.x;
		// 	double sizeY = _cachedBounds.Size.y;

		// 	var visibleEntities = EntitiesManager.Instance.VisibleEntities;
		// 	foreach (var entity in visibleEntities)
		// 	{
		// 		var bodyVisual = EntitiesManager.Instance.GetVisualForEntity(entity);
		// 		if (bodyVisual != null)
		// 		{
		// 			// Only invoke UpdateVisual when observation state is gained or lost on a visible entity
		// 			if (entity.IsObserved != bodyVisual.LastObservedState)
		// 			{
		// 				bodyVisual.UpdateVisual();
		// 			}
		// 			UpdateEntityTransform(bodyVisual, rectWidth, rectHeight, minX, minY, sizeX, sizeY);
		// 		}
		// 	}

		// 	var visibleGroups = EntityGroupsManager.Instance.Groups;
		// 	foreach (var group in visibleGroups)
		// 	{
		// 		var bodyVisual = EntityGroupsManager.Instance.GetVisualForGroup(group);
		// 		if (bodyVisual != null)
		// 		{
		// 			UpdateGroupTransform(bodyVisual, rectWidth, rectHeight, minX, minY, sizeX, sizeY);
		// 		}
		// 	}
		// }

		private void EnsureTransformArrayCapacities(int capacity)
		{
			if (!_transformPositions.IsCreated || _transformPositions.Length < capacity)
			{
				if (_transformPositions.IsCreated)
				{
					_transformPositions.Dispose();
					_transformRotations.Dispose();
					_transformIsCelestial.Dispose();
					_transformRadii.Dispose();
					_transformOutPosX.Dispose();
					_transformOutPosY.Dispose();
					_transformOutScale.Dispose();
					_transformOutRot.Dispose();
				}

				int padded = Mathf.CeilToInt(capacity * 1.3f);
				_transformPositions = new NativeArray<double2>(padded, Allocator.Persistent);
				_transformRotations = new NativeArray<float>(padded, Allocator.Persistent);
				_transformIsCelestial = new NativeArray<bool>(padded, Allocator.Persistent);
				_transformRadii = new NativeArray<double>(padded, Allocator.Persistent);
				_transformOutPosX = new NativeArray<float>(padded, Allocator.Persistent);
				_transformOutPosY = new NativeArray<float>(padded, Allocator.Persistent);
				_transformOutScale = new NativeArray<float>(padded, Allocator.Persistent);
				_transformOutRot = new NativeArray<float>(padded, Allocator.Persistent);
			}

			if (_cachedVisualsBuffer.Capacity < capacity)
			{
				_cachedVisualsBuffer.Capacity = Mathf.CeilToInt(capacity * 1.3f);
			}
		}

		private void UpdateVisibleObjectTransforms()
		{
			_cachedVisualsBuffer.Clear();
			if (!_isSimulationSystemLoaded || EntitiesManager.Instance == null || EntityGroupsManager.Instance == null)
			{
				RenderInstancedMapObjects();
				return;
			}

			Rect rect = _visualizationArea.rect;
			float rectWidth = rect.width;
			float rectXMin = rect.xMin;
			float rectXMax = rect.xMax;
			float rectYMin = rect.yMin;
			float rectYMax = rect.yMax;

			double minX = _cachedBounds.MinX;
			double minY = _cachedBounds.MinY;
			double sizeX = _cachedBounds.Size.x;
			double sizeY = _cachedBounds.Size.y;

			var visibleEntities = EntitiesManager.Instance.VisibleEntities;
			var visibleGroups = EntityGroupsManager.Instance.Groups;
			int totalCount = visibleEntities.Count + visibleGroups.Count;
			if (totalCount == 0)
			{
				RenderInstancedMapObjects();
				return;
			}

			EnsureTransformArrayCapacities(totalCount);
			int index = 0;

			// Normal entities are data-only handles; celestial entities use a small number of
			// dedicated UI objects because each has its own material/shader configuration.
			foreach (var entity in visibleEntities)
			{
				MapObject mapObject = entity.IsCelestial
					? (MapObject)EntitiesManager.Instance.GetCelestialVisualForEntity(entity)
					: EntitiesManager.Instance.GetVisualForEntity(entity);
				if (!CircularRaycastTarget.IsAlive(mapObject))
					continue;

				if (mapObject is EntityMapObject normalVisual && entity.IsObserved != normalVisual.LastObservedState)
					normalVisual.UpdateVisual();

				_transformPositions[index] = entity.Position;
				_transformRotations[index] = entity.Orientation;
				_transformIsCelestial[index] = entity.IsCelestial;
				_transformRadii[index] = entity.PhysicsBody.Radius;
				_cachedVisualsBuffer.Add(mapObject);
				index++;
			}

			// Groups deliberately retain their pre-existing GameObject/UI Graphic path.
			foreach (var group in visibleGroups)
			{
				var groupVisual = EntityGroupsManager.Instance.GetVisualForGroup(group);
				if (!CircularRaycastTarget.IsAlive(groupVisual))
					continue;

				_transformPositions[index] = group.Position;
				_transformRotations[index] = 0f;
				_transformIsCelestial[index] = false;
				_transformRadii[index] = 50.0;
				_cachedVisualsBuffer.Add(groupVisual);
				index++;
			}

			if (index == 0)
			{
				RenderInstancedMapObjects();
				return;
			}

			var job = new UpdateTransformsJob
			{
				Positions = _transformPositions,
				Rotations = _transformRotations,
				IsCelestial = _transformIsCelestial,
				Radii = _transformRadii,
				MinX = minX,
				MinY = minY,
				SizeX = sizeX,
				SizeY = sizeY,
				RectWidth = rectWidth,
				MinPixelSize = _minPixelSize,
				MinPixelSizeCelestial = _minPixelSizeCelestial,
				MinPixelSizeProjectile = _minPixelSizeProjectile,
				OutPosX = _transformOutPosX,
				OutPosY = _transformOutPosY,
				OutScale = _transformOutScale,
				OutRot = _transformOutRot
			};

			JobHandle handle = job.Schedule(index, 64);
			handle.Complete();

			for (int i = 0; i < index; i++)
			{
				_cachedVisualsBuffer[i].ApplyTransformData(
					_transformOutPosX[i],
					_transformOutPosY[i],
					_transformOutScale[i],
					_transformOutRot[i],
					sizeX,
					rectXMin,
					rectXMax,
					rectYMin,
					rectYMax);
			}

			RenderInstancedMapObjects();
		}

		#region GPU Instanced Map Rendering

		private void InitializeInstancedRendering()
		{
			if (_visualizationArea == null)
				return;

			_instancingSupported = SystemInfo.supportsInstancing;
			if (!_instancingSupported)
				Debug.LogError($"[{GetType().Name}] This platform does not support GPU instancing.", this);

			if (_mapObjectInstancedMaterial == null)
			{
				Shader shader = Shader.Find("GravitasDemo/MapObjectInstanced");
				if (shader != null)
				{
					_mapObjectInstancedMaterial = new Material(shader);
					_ownsMapObjectMaterial = true;
				}
			}

			if (_mapObjectInstancedMaterial != null)
				_mapObjectInstancedMaterial.enableInstancing = true;
			else
				Debug.LogError($"[{GetType().Name}] Missing GravitasDemo/MapObjectInstanced material/shader.", this);

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
			_spriteBatches.Clear();
			_activeSpriteBatches.Clear();
			_activeSpriteBatchSet.Clear();

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

			Rect rect = _visualizationArea.rect;
			Canvas canvas = _visualizationArea.GetComponentInParent<Canvas>();
			float canvasScale = canvas != null ? canvas.scaleFactor : 1f;
			Vector3 areaScale = _visualizationArea.lossyScale;
			float scaleX = Mathf.Max(0.0001f, Mathf.Abs(areaScale.x));
			float scaleY = Mathf.Max(0.0001f, Mathf.Abs(areaScale.y));
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

			_mapRenderTexture = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32)
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

		private void UpdateMapCameraTransform()
		{
			if (_mapRenderCamera == null || _visualizationArea == null)
				return;

			Rect rect = _visualizationArea.rect;
			Vector3 areaScale = _visualizationArea.lossyScale;
			float scaleX = Mathf.Max(0.0001f, Mathf.Abs(areaScale.x));
			float scaleY = Mathf.Max(0.0001f, Mathf.Abs(areaScale.y));
			Vector3 centerWorld = _visualizationArea.TransformPoint(rect.center);
			Quaternion areaRotation = _visualizationArea.rotation;
			Vector3 cameraForward = areaRotation * Vector3.forward;

			_mapRenderCamera.transform.SetPositionAndRotation(centerWorld - cameraForward * 20f, areaRotation);
			_mapRenderCamera.orthographicSize = rect.height * scaleY * 0.5f;
			_mapRenderCamera.aspect = (rect.width * scaleX) / Mathf.Max(0.0001f, rect.height * scaleY);
		}

		private Matrix4x4 GetMapObjectMatrix(MapObject mapObject)
		{
			Vector3 areaScale = _visualizationArea.lossyScale;
			Vector3 worldPosition = _visualizationArea.TransformPoint(
				new Vector3(mapObject.ViewPosition.x, mapObject.ViewPosition.y, 0f));
			Quaternion worldRotation = _visualizationArea.rotation *
				Quaternion.Euler(0f, 0f, mapObject.RenderRotationDegrees);
			Vector3 worldScale = new Vector3(
				mapObject.PixelDiameter * Mathf.Abs(areaScale.x),
				mapObject.PixelDiameter * Mathf.Abs(areaScale.y),
				1f);
			return Matrix4x4.TRS(worldPosition, worldRotation, worldScale);
		}

		private void RenderInstancedMapObjects()
		{
			if (_mapRenderCamera == null || _visualizationArea == null)
				return;

			RefreshMapRenderTexture();
			UpdateMapCameraTransform();

			foreach (SpriteBatch existingBatch in _spriteBatches.Values)
				existingBatch.Count = 0;
			_activeSpriteBatches.Clear();
			_activeSpriteBatchSet.Clear();

			for (int i = 0; i < _cachedVisualsBuffer.Count; i++)
			{
				MapObject mapObject = _cachedVisualsBuffer[i];
				// Only normal entities are submitted to the instanced renderer. Celestials and
				// groups are UI Graphics and render through their own Canvas hierarchy.
				if (!(mapObject is EntityMapObject entityVisual) ||
					!CircularRaycastTarget.IsAlive(entityVisual) || entityVisual.PixelDiameter <= 0f)
					continue;

				Matrix4x4 matrix = GetMapObjectMatrix(entityVisual);
				Sprite sprite = entityVisual.RenderSprite;
				if (sprite == null || _mapObjectInstancedMaterial == null || !_instancingSupported)
					continue;

				if (!_spriteBatches.TryGetValue(sprite, out SpriteBatch batch))
				{
					Mesh spriteMesh = GetOrCreateSpriteMesh(sprite);
					if (spriteMesh == null)
						continue;

					batch = new SpriteBatch { Sprite = sprite, Mesh = spriteMesh };
					_spriteBatches.Add(sprite, batch);
				}

				if (_activeSpriteBatchSet.Add(batch))
					_activeSpriteBatches.Add(batch);

				batch.Matrices[batch.Count] = matrix;
				batch.Colors[batch.Count] = entityVisual.RenderColor;
				batch.Count++;

				if (batch.Count == MaxInstancesPerDraw)
					FlushSpriteBatch(batch);
			}

			for (int i = 0; i < _activeSpriteBatches.Count; i++)
				FlushSpriteBatch(_activeSpriteBatches[i]);

			RenderSelectedEntityIndicators();
			_mapRenderCamera.Render();
		}

		/// <summary>
		/// Draws selection outlines for data-only entities. This keeps the normal-entity
		/// path entirely free of per-entity UI indicator GameObjects as well as visual GameObjects.
		/// </summary>
		private void RenderSelectedEntityIndicators()
		{
			if (_selectionRingInstancedMaterial == null || !_instancingSupported ||
				SimulationInteractionController.Instance == null)
				return;

			IReadOnlyList<MapObject> selected = SimulationInteractionController.Instance.SelectedMapObjects;
			int count = 0;
			for (int i = 0; i < selected.Count; i++)
			{
				if (!(selected[i] is EntityMapObject entityVisual) ||
					!CircularRaycastTarget.IsAlive(entityVisual) || entityVisual.PixelDiameter <= 0f)
					continue;

				float diameter = entityVisual.PixelDiameter + 2f * _selectionRingPaddingPixels;
				Vector3 areaScale = _visualizationArea.lossyScale;
				Vector3 worldPosition = _visualizationArea.TransformPoint(
					new Vector3(entityVisual.ViewPosition.x, entityVisual.ViewPosition.y, 0f));
				Vector3 worldScale = new Vector3(
					diameter * Mathf.Abs(areaScale.x),
					diameter * Mathf.Abs(areaScale.y),
					1f);
				_selectionRingMatrices[count] = Matrix4x4.TRS(worldPosition, _visualizationArea.rotation, worldScale);
				_selectionRingColors[count] = _selectionRingColor;
				count++;

				if (count == MaxInstancesPerDraw)
				{
					FlushSelectionRingBatch(count);
					count = 0;
				}
			}

			if (count > 0)
				FlushSelectionRingBatch(count);
		}

		private void FlushSelectionRingBatch(int count)
		{
			_selectionRingProperties.Clear();
			_selectionRingProperties.SetVectorArray(InstanceColorID, _selectionRingColors);
			_selectionRingProperties.SetFloat("_RingRadius", 0.465f);
			_selectionRingProperties.SetFloat("_RingHalfWidth", _selectionRingThickness * 0.5f);

			RenderParams renderParams = new RenderParams(_selectionRingInstancedMaterial)
			{
				camera = _mapRenderCamera,
				layer = _mapRenderLayer,
				matProps = _selectionRingProperties,
				shadowCastingMode = ShadowCastingMode.Off,
				receiveShadows = false
			};

			Graphics.RenderMeshInstanced(
				renderParams,
				_unitQuadMesh,
				0,
				_selectionRingMatrices,
				count);
		}

		private void FlushSpriteBatch(SpriteBatch batch)
		{
			if (batch == null || batch.Count <= 0 || batch.Mesh == null || batch.Sprite == null)
				return;

			batch.Properties.Clear();
			batch.Properties.SetTexture(MainTexID, batch.Sprite.texture);
			batch.Properties.SetVectorArray(InstanceColorID, batch.Colors);

			RenderParams renderParams = new RenderParams(_mapObjectInstancedMaterial)
			{
				camera = _mapRenderCamera,
				layer = _mapRenderLayer,
				matProps = batch.Properties,
				shadowCastingMode = ShadowCastingMode.Off,
				receiveShadows = false
			};

			Graphics.RenderMeshInstanced(
				renderParams,
				batch.Mesh,
				0,
				batch.Matrices,
				batch.Count);

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

	[BurstCompile]
	public struct UpdateTransformsJob : IJobParallelFor
	{
		[ReadOnly] public NativeArray<double2> Positions;
		[ReadOnly] public NativeArray<float> Rotations;
		[ReadOnly] public NativeArray<bool> IsCelestial;
		[ReadOnly] public NativeArray<double> Radii;
		[ReadOnly] public double MinX;
		[ReadOnly] public double MinY;
		[ReadOnly] public double SizeX;
		[ReadOnly] public double SizeY;
		[ReadOnly] public float RectWidth;
		[ReadOnly] public float MinPixelSize;
		[ReadOnly] public float MinPixelSizeCelestial;
		[ReadOnly] public float MinPixelSizeProjectile;

		[WriteOnly] public NativeArray<float> OutPosX;
		[WriteOnly] public NativeArray<float> OutPosY;
		[WriteOnly] public NativeArray<float> OutScale;
		[WriteOnly] public NativeArray<float> OutRot;

		public void Execute(int index)
		{
			double2 pos = Positions[index];
			float percX = (float)((pos.x - MinX) / SizeX);
			float percY = (float)((pos.y - MinY) / SizeY);

			OutPosX[index] = percX;
			OutPosY[index] = percY;
			OutRot[index] = Rotations[index];

			bool celestial = IsCelestial[index];
			float actualMinPixelSize = celestial ? MinPixelSizeCelestial : MinPixelSize;
			float radiusMultiplier = celestial ? 2.6f : 2.0f;
			float accuratePixelSize = (float)(Radii[index] / SizeX) * RectWidth * radiusMultiplier;

			OutScale[index] = math.max(accuratePixelSize, actualMinPixelSize);
		}
	}
}