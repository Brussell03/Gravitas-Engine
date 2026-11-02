using System;
using System.Collections.Generic;
using Gravitas2D;
using GravitasDemo.Interaction;
using TMPro;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Pool;
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
		[SerializeField] private EntityMapObject _entityPrefab;
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

		public RectTransform VisualizationArea => _visualizationArea;
		public Bounds2D ViewBounds => _cachedBounds;
		public float BoundsBufferPercentage => _boundsBufferPercentage;
		public Sprite GetBodyImage(EntityType type) => _bodyImageTypes[(int)type];
		public Material PlanetShaderMaterial => _planetShaderMaterial;

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

			_sceneEntitiesPool = new ObjectPool<EntityMapObject>(CreateEntity, OnGetFromPool, OnReleaseToPool, OnDestroyPooledObject, _collectionCheck, _defaultCapacity, _maxSize);

			_sceneEntityGroupsPool = new ObjectPool<EntityGroupMapObject>(CreateEntityGroup, OnGetFromPool, OnReleaseToPool, OnDestroyPooledObject, _collectionCheck, _defaultCapacityGroups, _maxSizeGroups);

			RecalculateBounds();

			_trajectoryRenderer = new TrajectoryRenderer(_trajectoriesContainer);

			SaveManager.Instance.OnLoad += SetIsLoaded;
			SaveManager.Instance.OnReset += ClearRendererState;
		}

		void OnDestroy()
		{
			_trajectoryRenderer.Cleanup();

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
			if (PhysicsManager2D.Instance == null || !PhysicsManager2D.Instance.IsInitialized || _visualizationArea == null || (GameManager.Instance?.IsPaused ?? true)) return;
			
			if (_isBoundsDirty)
			{
				RecalculateBounds();
			}

			UpdateVisibleObjectTransforms();
			_trajectoryRenderer.UpdateTrajectories();

			float time = PhysicsManager2D.Instance.SimulationElapsedTime;
			TimeSpan t = TimeSpan.FromSeconds(time);
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

		private void UpdateVisibleObjectTransforms()
		{
			if (!_isSimulationSystemLoaded) return;

			// Cache external C++ properties to prevent boundary crossing in the loop
			float rectWidth = _visualizationArea.rect.width;
			float rectHeight = _visualizationArea.rect.height;
			double minX = _cachedBounds.MinX;
			double minY = _cachedBounds.MinY;
			double sizeX = _cachedBounds.Size.x;
			double sizeY = _cachedBounds.Size.y;

			var visibleEntities = EntitiesManager.Instance.VisibleEntities;
			foreach (var entity in visibleEntities)
			{
				var bodyVisual = EntitiesManager.Instance.GetVisualForEntity(entity);
				if (bodyVisual != null)
				{
					// Only invoke UpdateVisual when observation state is gained or lost on a visible entity
					if (entity.IsObserved != bodyVisual.LastObservedState)
					{
						bodyVisual.UpdateVisual();
					}
					UpdateEntityTransform(bodyVisual, rectWidth, rectHeight, minX, minY, sizeX, sizeY);
				}
			}

			var visibleGroups = EntityGroupsManager.Instance.Groups;
			foreach (var group in visibleGroups)
			{
				var bodyVisual = EntityGroupsManager.Instance.GetVisualForGroup(group);
				if (bodyVisual != null)
				{
					UpdateGroupTransform(bodyVisual, rectWidth, rectHeight, minX, minY, sizeX, sizeY);
				}
			}
		}

		private void UpdateEntityTransform(EntityMapObject entityObject, float rectWidth, float rectHeight, double minX, double minY, double sizeX, double sizeY)
		{
			var position = entityObject.PhysicsBody.Position;

			float percX = (float)((position.x - minX) / sizeX);
			float percY = (float)((position.y - minY) / sizeY);

			float actualMinVisiblePixelSize = entityObject.PhysicsBody.IsCelestial ? _minPixelSizeCelestial : entityObject.Entity.Type == EntityType.Missile || entityObject.Entity.Type == EntityType.Projectile ? _minPixelSizeProjectile : _minPixelSize;

			entityObject.UpdateTransform(percX, percY, rectWidth, rectHeight, sizeX, actualMinVisiblePixelSize);
		}

		private void UpdateGroupTransform(EntityGroupMapObject groupObject, float rectWidth, float rectHeight, double minX, double minY, double sizeX, double sizeY)
		{
			var position = groupObject.SimPos;

			float percX = (float)((position.x - minX) / sizeX);
			float percY = (float)((position.y - minY) / sizeY);

			groupObject.UpdateTransform(percX, percY, rectWidth, rectHeight, sizeX, _pixelSizeGroup);
		}

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

		#region Entity Pooling

		private IObjectPool<EntityMapObject> _sceneEntitiesPool;

		// extra options to control the pool capacity and maximum size
		private int _defaultCapacity = 1000;
		private int _maxSize = 50000;
		
		public EntityMapObject GetEntityFromPool(Entity entity)
		{
			var body = _sceneEntitiesPool.Get();
			body.InitializeBody(entity);
			UpdateEntityTransform(body, _visualizationArea.rect.width, _visualizationArea.rect.height, _cachedBounds.MinX, _cachedBounds.MinY, _cachedBounds.Size.x, _cachedBounds.Size.y);

			return body;
		}

		EntityMapObject CreateEntity()
		{
			EntityMapObject bodyInstance = Instantiate(_entityPrefab, _bodiesContainer);
			bodyInstance.ObjectPool = _sceneEntitiesPool;

			return bodyInstance;
		}

		void OnGetFromPool(EntityMapObject pooledObject)
		{
			pooledObject.gameObject.SetActive(true);
		}

		void OnReleaseToPool(EntityMapObject pooledObject)
		{
			pooledObject.Deinitialize();
			pooledObject.gameObject.SetActive(false);
		}

		void OnDestroyPooledObject(EntityMapObject pooledObject)
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
			UpdateGroupTransform(body, _visualizationArea.rect.width, _visualizationArea.rect.height, _cachedBounds.MinX, _cachedBounds.MinY, _cachedBounds.Size.x, _cachedBounds.Size.y);

			return body;
		}

		EntityGroupMapObject CreateEntityGroup()
		{
			EntityGroupMapObject bodyInstance = Instantiate(_groupPrefab, _bodiesContainer);
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