using System;
using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;
using Gravitas2D;
using System.Linq;
using UnityEngine.UI;
using TMPro;
using UnityEngine.EventSystems;
using GravitasDemo.Rendering;

namespace GravitasDemo.Interaction
{
	[RequireComponent(typeof(SimulationRenderer))]
	public class SimulationInteractionController : MonoBehaviour
	{
		public static SimulationInteractionController Instance { get; private set; }

		[Header("Interaction Configuration")]
		[Tooltip("Keyboard view panning speed as a percentage of the view width per second.")]
		[SerializeField, Min(0.01f)] private float _keyboardPanSpeed = 0.25f;

		[Header("Interaction Configuration")]
		[Tooltip("Additional hover margin as a percentage of the visualization area's max dimension.")]
		[SerializeField, Min(0)] private float _hoverMarginScreenPct = 0.01f;

		[SerializeField, Min(0)] private float _zoomSensitivity = 0.1f;
		[SerializeField, Min(1)] private double _minZoomSize = 1000f; // Prevents zooming in infinitely
		[SerializeField, Min(1)] private double _maxZoomSize = 8e12;

		[Header("Hover Indicator UI")]
		[SerializeField] private RectTransform _hoverIndicator;
		[SerializeField] private Color _defaultHoverColor = new Color(1f, 1f, 1f, 0.8f);
		private Image _hoverIndicatorImage;
		private TextMeshProUGUI _hoverIndicatorText;

		[Header("Selection Indicator UI")]
		[SerializeField] private RectTransform _selectionIndicatorPrefab;
		[SerializeField] private RectTransform _selectionIndicatorsContainer;
		private Dictionary<MapObject, RectTransform> _selectionIndicators = new Dictionary<MapObject, RectTransform>();
		
		private MapObject _hoveredMapObject;
		private List<MapObject> _selectedMapObjects = new List<MapObject>();
		private MapObject _targetedMapObject;
		private MapObject _trackedMapObject;
		private HashSet<MapObject> _pinnedMapObjects = new HashSet<MapObject>();
		private int _selectionIndex = 0;

		private enum InteractionMode { Selection, Firing, Navigation, Attitude, OrbitMatch }
		private InteractionMode _mode = InteractionMode.Selection;
		
		private bool _isDragging;
		private Vector3 _lastMousePosition;

		private SimulationRenderer _renderer;

		public MapObject HoveredMapObject => _hoveredMapObject;
		public Entity HoveredEntity => (_hoveredMapObject as EntityMapObject)?.Entity;
		public EntityGroup HoveredGroup => (_hoveredMapObject as EntityGroupMapObject)?.Group;
		
		public MapObject SelectedMapObject => _selectedMapObjects.Count > _selectionIndex ? _selectedMapObjects[_selectionIndex] : null;
		public Entity SelectedEntity => (SelectedMapObject as EntityMapObject)?.Entity;
		public EntityGroup SelectedGroup => (SelectedMapObject as EntityGroupMapObject)?.Group;
		public List<MapObject> SelectedMapObjects => _selectedMapObjects;
		
		public MapObject TargetedMapObject => _targetedMapObject;
		public Entity TargetedEntity => (_targetedMapObject as EntityMapObject)?.Entity;
		
		public MapObject TrackedMapObject => _trackedMapObject;
		public Entity TrackedEntity => (_trackedMapObject as EntityMapObject)?.Entity;
		
		public HashSet<MapObject> PinnedMapObjects => _pinnedMapObjects;

		public bool IsSelectedTargeted => SelectedMapObject != null && SelectedMapObject == _targetedMapObject;
		public bool IsSelectedPinned => SelectedMapObject != null && _pinnedMapObjects.Contains(SelectedMapObject);
		public bool IsSelectedTracked => SelectedMapObject != null && SelectedMapObject == _trackedMapObject;

		public bool IsEntitySelected(Entity entity)
		{
			if (entity == null) return false;
			foreach (var obj in _selectedMapObjects)
			{
				if (obj is EntityMapObject ego && ego.Entity == entity) return true;
			}
			return false;
		}

		public bool IsEntityPinned(Entity entity)
		{
			if (entity == null) return false;
			foreach (var obj in _pinnedMapObjects)
			{
				if (obj is EntityMapObject ego && ego.Entity == entity) return true;
			}
			return false;
		}

		public bool IsEntityHovered(Entity entity)
		{
			if (entity == null) return false;
			return _hoveredMapObject is EntityMapObject ego && ego.Entity == entity;
		}

		public bool IsEntitySelectedPinnedOrHovered(Entity entity)
		{
			return IsEntitySelected(entity) || IsEntityPinned(entity) || IsEntityHovered(entity);
		}

		public float HoverMarginScreenPct => _hoverMarginScreenPct;

		public event Action<MapObject> OnMapObjectHovered;
		public event Action<MapObject, bool> OnMapObjectUnhovered;
		public event Action<MapObject> OnMapObjectSelected;
		public event Action<MapObject, bool> OnMapObjectDeselected;
		public event Action<MapObject> OnMapObjectTargeted;
		public event Action<MapObject, bool> OnMapObjectUntargeted;
		public event Action<MapObject> OnMapObjectPinned;
		public event Action<MapObject, bool> OnMapObjectUnpinned;
		public event Action<MapObject> OnMapObjectTracked;
		public event Action<MapObject> OnMapObjectUntracked;

		private void Awake()
		{
			if (Instance != null && Instance != this)
			{
				Destroy(gameObject);
				return;
			}
			Instance = this;

			_renderer = GetComponent<SimulationRenderer>();
			
			// Input Validation
			if (_renderer == null)
			{
				Debug.LogError($"[{GetType().Name}] Missing required SimulationRenderer component.");
				enabled = false;
			}

			if (_hoverIndicator != null)
			{
				_hoverIndicator.gameObject.SetActive(false);
				_hoverIndicatorImage = _hoverIndicator.transform.GetComponentInChildren<Image>();
				_hoverIndicatorText = _hoverIndicator.transform.GetComponentInChildren<TextMeshProUGUI>();
			}

			SaveManager.Instance.OnLoad += ClearInteractions;
		}

		void OnDestroy()
		{
			ClearInteractions(true);

			if (Instance == this) Instance = null;
			if (SaveManager.Instance) SaveManager.Instance.OnLoad -= ClearInteractions;
		}

		private void OnValidate()
		{
			if (_maxZoomSize <= _minZoomSize)
			{
				_maxZoomSize = (double)_minZoomSize + 1.0;
				Debug.LogWarning($"[{GetType().Name}] _maxZoomSize cannot be less than or equal to _minZoomSize. Automatically adjusted.");
			}
		}

		private void Update()
		{
			if (PhysicsManager2D.Instance == null || _renderer.VisualizationArea == null || (GameManager.Instance?.IsPaused ?? true))
				return;

			HandleDestroyedObjects();
			if (SimulationOrbitMatchController.Instance != null && !SimulationOrbitMatchController.Instance.IsPreviewingOrbitMatch)
			{
				bool isOverViewArea = HandleHover();
			
				if (isOverViewArea && _mode == InteractionMode.Selection) 
				{
					HandleSelection();
				}
			}

			UpdateHoverIndicator();
			UpdateSelectionIndicators();

			HandlePanning();
			HandleZooming();
		}

		private void ClearInteractions(bool loaded)
		{
			DeselectAll();
			UntargetMapObject();
			UntrackMapObject();
			UnpinAll();
		}

		private void UpdateHoverIndicator()
		{
			if (_hoverIndicator == null) return;

			Entity hoveredEnt = HoveredEntity;
			if (_hoveredMapObject != null && (hoveredEnt == null || !hoveredEnt.IsDestroyed))
			{
				_hoverIndicator.gameObject.SetActive(true);
				Vector2 uiPos = _hoveredMapObject.RectTransform.localPosition;
				_hoverIndicator.localPosition = new Vector3(uiPos.x, uiPos.y, _hoverIndicator.localPosition.z);

				string text = hoveredEnt != null ? $"{hoveredEnt.Name}\nControlled: {(hoveredEnt.IsOwned ? "Yes" : "No")}" : "Group Object";
				Color indicatorColor = _defaultHoverColor;

				if (hoveredEnt != null)
				{
					if (_mode == InteractionMode.Firing && SimulationFiringController.Instance != null && SimulationFiringController.Instance.IsActive)
					{
						SimulationFiringController.Instance.AppendHoverIndicatorInfo(hoveredEnt, ref text, ref indicatorColor);
					}
					else if (_mode == InteractionMode.Navigation && SimulationNavigationController.Instance != null && SimulationNavigationController.Instance.IsActive)
					{
						SimulationNavigationController.Instance.AppendHoverIndicatorInfo(hoveredEnt, ref text, ref indicatorColor);
					}
					else if (_mode == InteractionMode.OrbitMatch && SimulationOrbitMatchController.Instance != null && SimulationOrbitMatchController.Instance.IsActive)
					{
						SimulationOrbitMatchController.Instance.AppendHoverIndicatorInfo(hoveredEnt, ref text, ref indicatorColor);
					}
				}

				if (_hoverIndicatorImage != null) _hoverIndicatorImage.color = indicatorColor;
				if (_hoverIndicatorText != null) _hoverIndicatorText.text = text;
			}
			else
			{
				if (_mode == InteractionMode.OrbitMatch && SimulationOrbitMatchController.Instance != null && SimulationOrbitMatchController.Instance.IsActive)
				{
					SimulationOrbitMatchController.Instance.UpdateHoverIndicatorForNonEntity(_hoverIndicator, _hoverIndicatorImage, _hoverIndicatorText);
				}
				else
				{
					_hoverIndicator.gameObject.SetActive(false);
				}
			}
		}

		private void UpdateSelectionIndicators()
		{
			foreach (var kvp in _selectionIndicators)
			{
				MapObject mapObj = kvp.Key;
				RectTransform indicator = kvp.Value;

				if (mapObj == null || mapObj.IsDestroyed) continue;

				bool shouldShow = (mapObj != _hoveredMapObject);
				if (indicator.gameObject.activeSelf != shouldShow)
				{
					indicator.gameObject.SetActive(shouldShow);
				}

				if (shouldShow)
				{
					indicator.localPosition = new Vector3(mapObj.RectTransform.localPosition.x, mapObj.RectTransform.localPosition.y, indicator.localPosition.z);
				}
			}
		}

		// /// <summary>
		// /// Finds the closest visible entity to a given local UI position within the visualization area, accounting for hover margin.
		// /// </summary>
		// public Entity FindClosestEntityAtUIPosition(Vector2 localUIPos)
		// {
		// 	RectTransform visArea = _renderer.VisualizationArea;
		// 	float rectWidth = visArea.rect.width;
		// 	float rectHeight = visArea.rect.height;

		// 	if (rectWidth <= 0f || rectHeight <= 0f) return null;

		// 	float hoverMarginPixels = rectWidth * _hoverMarginScreenPct;
		// 	Entity closestEntity = null;
		// 	float minSqrDistance = float.MaxValue;

		// 	foreach (var entity in EntitiesManager.Instance.VisibleEntities)
		// 	{
		// 		Entity target = entity;

		// 		Vector2 bodyPosUI = _renderer.SimPositionToViewUI(target.Position);

		// 		float bodyRadiusUI = (float)(target.Radius / _renderer.ViewBounds.Size.x) * rectWidth;
		// 		float totalHoverRadius = bodyRadiusUI + hoverMarginPixels;

		// 		float sqrDist = (localUIPos - bodyPosUI).sqrMagnitude;
		// 		if (sqrDist <= (totalHoverRadius * totalHoverRadius) && sqrDist < minSqrDistance)
		// 		{
		// 			minSqrDistance = sqrDist;
		// 			closestEntity = target;
		// 		}
		// 	}

		// 	return closestEntity;
		// }

		/// <summary>
		/// Finds the closest visible map object to the cursor position among all 
		/// raycast hits using consistent visualization area local UI coordinates.
		/// </summary>
		public MapObject FindClosestMapObjectAtUIPosition(Vector2 localUIPos)
		{
			RectTransform visArea = _renderer.VisualizationArea;
			if (visArea == null || EventSystem.current == null) return null;

			Canvas canvas = visArea.GetComponentInParent<Canvas>();
			Camera cam = (canvas != null && canvas.renderMode != RenderMode.ScreenSpaceOverlay) ? canvas.worldCamera : null;

			// Convert visualization area local UI position to screen point for raycasting
			Vector2 screenPoint = RectTransformUtility.WorldToScreenPoint(cam, visArea.TransformPoint(localUIPos));

			PointerEventData pointerData = new PointerEventData(EventSystem.current)
			{
				position = screenPoint
			};

			List<RaycastResult> raycastResults = new List<RaycastResult>();
			EventSystem.current.RaycastAll(pointerData, raycastResults);

			MapObject closestMapObject = null;
			float minSqrDistance = float.MaxValue;

			// Evaluate distance across all valid raycast hits to find the truly closest entity/group
			for (int i = 0; i < raycastResults.Count; i++)
			{
				GameObject hitObject = raycastResults[i].gameObject;
				if (hitObject == null) continue;

				// 1. Check if hit object is an individual entity visual
				EntityMapObject entityVisual = hitObject.GetComponent<EntityMapObject>();
				if (entityVisual != null && entityVisual.Entity != null && !entityVisual.Entity.IsDestroyed)
				{
					Vector2 bodyUIPos = _renderer.SimPositionToViewUI(entityVisual.Entity.Position);
					float sqrDist = (localUIPos - bodyUIPos).sqrMagnitude;
					if (sqrDist < minSqrDistance)
					{
						minSqrDistance = sqrDist;
						closestMapObject = entityVisual;
					}
					continue;
				}

				// 2. Check if hit object is an entity group visual
				EntityGroupMapObject groupVisual = hitObject.GetComponent<EntityGroupMapObject>();
				if (groupVisual != null && groupVisual.Group != null && !groupVisual.Group.IsDestroyed)
				{
					Vector2 groupUIPos = _renderer.SimPositionToViewUI(groupVisual.Group.Position);
					float sqrDist = (localUIPos - groupUIPos).sqrMagnitude;
					if (sqrDist < minSqrDistance)
					{
						minSqrDistance = sqrDist;
						closestMapObject = groupVisual;
					}
				}
			}

			return closestMapObject;
		}

		private bool HandleHover()
		{
			RectTransform visArea = _renderer.VisualizationArea;
			float rectWidth = visArea.rect.width;
			float rectHeight = visArea.rect.height;

			// Error Handling: Prevent division by zero or invalid calculations
			if (rectWidth <= 0f || rectHeight <= 0f) return false;

			// Determine mouse position in the visualization area
			if (_renderer.TryGetMouseViewPosition(out Vector2 mouseUIPos))
			{
				MapObject prevHovered = _hoveredMapObject;
				_hoveredMapObject = FindClosestMapObjectAtUIPosition(mouseUIPos);

				if (_hoveredMapObject != prevHovered)
				{
					if (prevHovered != null) OnMapObjectUnhovered?.Invoke(prevHovered, _selectedMapObjects.Contains(prevHovered) || prevHovered == _targetedMapObject || _pinnedMapObjects.Contains(prevHovered));
					if (_hoveredMapObject != null) OnMapObjectHovered?.Invoke(_hoveredMapObject);
				}

				return true;
			}
			else
			{
				return false;
			}
		}

		private void HandleSelection()
		{
			if (Input.GetMouseButtonDown(0))
			{
				if (_hoveredMapObject != null)
				{
					if (!Input.GetKey(KeyCode.LeftShift)) DeselectAll();

					if (_selectedMapObjects.Contains(_hoveredMapObject)) DeselectMapObject(_hoveredMapObject);
					else SelectMapObject(_hoveredMapObject);
				}
				else
				{
					DeselectAll();
				}
			}
		}

		private void HandlePanning()
		{
			bool isMouseOverUI = RectTransformUtility.RectangleContainsScreenPoint(_renderer.VisualizationArea, Input.mousePosition, null);

			// --- Middle-Mouse Drag Panning ---
			if (Input.GetMouseButtonDown(2) && isMouseOverUI)
			{
				_isDragging = true;
				_lastMousePosition = Input.mousePosition;
			}
			else if (Input.GetMouseButtonUp(2))
			{
				_isDragging = false;
			}

			if (_isDragging)
			{
				Vector3 delta = Input.mousePosition - _lastMousePosition;
				_lastMousePosition = Input.mousePosition;

				double physicalDeltaX = -(delta.x / _renderer.VisualizationArea.rect.width) * _renderer.ViewBounds.Size.x;
				double physicalDeltaY = -(delta.y / _renderer.VisualizationArea.rect.height) * _renderer.ViewBounds.Size.y;
				
				_renderer.ViewCenter += new double2(physicalDeltaX, physicalDeltaY);
			}

			// --- WASD / Arrow Keys Keyboard Panning ---
			double2 keyDir = double2.zero;
			if (Input.GetKey(KeyCode.W) || Input.GetKey(KeyCode.UpArrow)) keyDir.y += 1.0;
			if (Input.GetKey(KeyCode.S) || Input.GetKey(KeyCode.DownArrow)) keyDir.y -= 1.0;
			if (Input.GetKey(KeyCode.A) || Input.GetKey(KeyCode.LeftArrow)) keyDir.x -= 1.0;
			if (Input.GetKey(KeyCode.D) || Input.GetKey(KeyCode.RightArrow)) keyDir.x += 1.0;

			if (math.lengthsq(keyDir) > 0.0001)
			{
				keyDir = math.normalize(keyDir);
				double panDistance = _renderer.ViewBounds.Size.x * _keyboardPanSpeed * Time.unscaledDeltaTime;
				_renderer.ViewCenter += keyDir * panDistance;
			}
		}

		private void HandleTrackPanning(double2 deltaPos)
		{
			if (_trackedMapObject != null)
			{
				_renderer.ViewCenter += deltaPos;
			}
		}

		private void HandleZooming()
		{
			float scrollDelta = Input.mouseScrollDelta.y;
			if (Mathf.Abs(scrollDelta) < 0.01f) return;

			if (_renderer.VisualizationArea == null) return;

			// Retrieve exact mouse simulation position using full double precision to prevent float drift at close zoom
			if (!_renderer.TryGetMouseSimPosition(out double2 mouseWorldPos, out Vector2 localMousePos))
				return;

			double currentSizeX = _renderer.ViewBounds.Size.x;
			double zoomFactor = scrollDelta > 0 ? (1.0 - _zoomSensitivity) : (1.0 + _zoomSensitivity);
			double newSizeX = currentSizeX * zoomFactor;

			if (newSizeX < _minZoomSize || newSizeX > _maxZoomSize) return;

			// Keep the exact mouse world position pinned under the cursor by adjusting the view center
			double2 mouseOffsetFromCenter = mouseWorldPos - _renderer.ViewCenter;
			double scaleRatio = newSizeX / currentSizeX;
			double2 newCenter = mouseWorldPos - (mouseOffsetFromCenter * scaleRatio);

			_renderer.ViewCenter = newCenter;
			_renderer.VisualizationRadius = newSizeX * 0.5;
			_renderer.RecalculateBounds();
		}

		private void HandleDestroyedObjects()
		{
			if (_hoveredMapObject != null && _hoveredMapObject.IsDestroyed)
			{
				OnMapObjectUnhovered?.Invoke(_hoveredMapObject, _selectedMapObjects.Contains(_hoveredMapObject) || _hoveredMapObject == _targetedMapObject || _pinnedMapObjects.Contains(_hoveredMapObject));
				_hoveredMapObject = null;
			}

			for (int i = _selectedMapObjects.Count - 1; i >= 0; i--)
			{
				if (_selectedMapObjects[i].IsDestroyed)
				{
					if (i == _selectionIndex) _selectionIndex = 0;
					DeselectMapObject(_selectedMapObjects[i]);
				}
			}

			if (_targetedMapObject != null && _targetedMapObject.IsDestroyed) UntargetMapObject();

			// Untrack if the tracked object is destroyed
			if (_trackedMapObject != null && _trackedMapObject.IsDestroyed)
			{
				UntrackMapObject();
			}

			List<MapObject> objectsToUnpin = new List<MapObject>();
			foreach (var mapObj in _pinnedMapObjects)
			{
				if (mapObj.IsDestroyed) objectsToUnpin.Add(mapObj);
			}

			foreach (var mapObj in objectsToUnpin)
			{
				UnpinMapObject(mapObj);
			}
		}

		public void SelectMapObject(MapObject mapObj)
		{
			Entity ent = (mapObj as EntityMapObject)?.Entity;
			if (mapObj == null || (ent != null && ent.IsDestroyed) || _selectedMapObjects.Contains(mapObj)) return;

			_selectedMapObjects.Add(mapObj);

			// Instantiate and initialize selection indicator
			if (_selectionIndicatorPrefab != null)
			{
				Transform parentTransform = _selectionIndicatorsContainer != null ? _selectionIndicatorsContainer : _renderer.VisualizationArea;
				RectTransform indicatorInstance = Instantiate(_selectionIndicatorPrefab, parentTransform);
				
				TextMeshProUGUI textComp = indicatorInstance.GetComponentInChildren<TextMeshProUGUI>();
				if (textComp != null)
				{
					textComp.text = ent != null ? ent.Name : "Group";
				}

				indicatorInstance.localPosition = new Vector3(mapObj.RectTransform.localPosition.x, mapObj.RectTransform.localPosition.y, indicatorInstance.localPosition.z);
				indicatorInstance.gameObject.SetActive(mapObj != _hoveredMapObject);

				_selectionIndicators[mapObj] = indicatorInstance;
			}

			OnMapObjectSelected?.Invoke(mapObj);
		}

		public void DeselectMapObject(MapObject mapObj)
		{
			if (mapObj == null) return;

			if (_selectedMapObjects.Remove(mapObj))
			{
				if (_selectionIndicators.TryGetValue(mapObj, out RectTransform indicator))
				{
					if (indicator != null)
					{
						Destroy(indicator.gameObject);
					}
					_selectionIndicators.Remove(mapObj);
				}
				
				OnMapObjectDeselected?.Invoke(mapObj, mapObj == _hoveredMapObject || mapObj == _targetedMapObject || _pinnedMapObjects.Contains(mapObj));
			}
		}

		public void DeselectAll()
		{
			for (int i = _selectedMapObjects.Count - 1; i >= 0; i--)
			{
				DeselectMapObject(_selectedMapObjects[i]);
			}
			_selectionIndex = 0;
		}

		public void CycleSelection(int offset)
		{
			_selectionIndex += offset;
			_selectionIndex = Mathf.Clamp(_selectionIndex, 0, _selectedMapObjects.Count - 1);
		}

		public void NextSelection() => CycleSelection(1);
		public void BackSelection() => CycleSelection(-1);

		public void TargetMapObject(MapObject mapObj)
		{
			Entity ent = (mapObj as EntityMapObject)?.Entity;
			if (mapObj == null || (ent != null && ent.IsDestroyed) || _targetedMapObject == mapObj) return;

			if (_targetedMapObject != null) UntargetMapObject();
			_targetedMapObject = mapObj;
			OnMapObjectTargeted?.Invoke(mapObj);
		}

		public void UntargetMapObject()
		{
			if (_targetedMapObject == null) return;
			OnMapObjectUntargeted?.Invoke(_targetedMapObject, _targetedMapObject == _hoveredMapObject || _selectedMapObjects.Contains(_targetedMapObject) || _pinnedMapObjects.Contains(_targetedMapObject));
			_targetedMapObject = null;
		}

		public void TargetSelected()
		{
			if (SelectedMapObject == null || _targetedMapObject == SelectedMapObject) return;

			if (_targetedMapObject != null) UntargetMapObject();
			_targetedMapObject = SelectedMapObject;
			OnMapObjectTargeted?.Invoke(_targetedMapObject);
		}

		public void ToggleSelectedTargeted()
		{
			if (SelectedMapObject != null && SelectedMapObject != _targetedMapObject) TargetSelected();
			else UntargetMapObject();
		}

		public void PinMapObject(MapObject mapObj)
		{
			Entity ent = (mapObj as EntityMapObject)?.Entity;
			if (mapObj == null || (ent != null && ent.IsDestroyed)) return;
			
			if (_pinnedMapObjects.Add(mapObj))
			{
				OnMapObjectPinned?.Invoke(mapObj);
			}
		}

		public void UnpinMapObject(MapObject mapObj)
		{
			if (_pinnedMapObjects.Remove(mapObj))
			{
				OnMapObjectUnpinned?.Invoke(mapObj, mapObj == _hoveredMapObject || _selectedMapObjects.Contains(mapObj) || mapObj == _targetedMapObject);
			}
		}

		public void PinSelected()
		{
			if (SelectedMapObject == null) return;
			PinMapObject(SelectedMapObject);
		}

		public void UnpinSelected()
		{
			if (SelectedMapObject == null) return;
			UnpinMapObject(SelectedMapObject);
		}

		public void ToggleSelectedPinned()
		{
			bool isOneNotPinned = false;
			foreach (var mapObj in _selectedMapObjects)
			{
				if (!_pinnedMapObjects.Contains(mapObj))
				{
					isOneNotPinned = true;
					break;
				}
			}

			foreach (var mapObj in _selectedMapObjects)
			{
				if (isOneNotPinned) PinMapObject(mapObj);
				else UnpinMapObject(mapObj);
			}
		}

		public void UnpinAll()
		{
			foreach (var mapObj in _pinnedMapObjects.ToList())
			{
				UnpinMapObject(mapObj);
			}
		}

		public void TrackMapObject(MapObject mapObj)
		{
			if (mapObj == null) return;

			UntrackMapObject();
			_trackedMapObject = mapObj;
			
			Entity ent = (mapObj as EntityMapObject)?.Entity;
			if (ent != null)
			{
				ent.SetTracked(true);
				ent.PhysicsBody.OnTrackedPositionUpdated += HandleTrackPanning;
			}

			OnMapObjectTracked?.Invoke(mapObj);
		}

		public void TrackSelected()
		{
			TrackMapObject(SelectedMapObject);
		}

		public void UntrackMapObject()
		{
			if (_trackedMapObject != null)
			{
				Entity ent = (_trackedMapObject as EntityMapObject)?.Entity;
				if (ent != null)
				{
					ent.PhysicsBody.OnTrackedPositionUpdated -= HandleTrackPanning;
					ent.SetTracked(false);
				}

				OnMapObjectUntracked?.Invoke(_trackedMapObject);
				_trackedMapObject = null;
			}
		}

		public void ToggleSelectedTracked()
		{
			if (SelectedMapObject != null && SelectedMapObject != _trackedMapObject) TrackMapObject(SelectedMapObject);
			else UntrackMapObject();
		}

		public void FractureSelected()
		{
			SelectedEntity?.Fracture();
		}

		public void SetSelectionMode() => _mode = InteractionMode.Selection;
		public void SetFiringMode() => _mode = InteractionMode.Firing;
		public void SetNavigationMode() => _mode = InteractionMode.Navigation;
		public void SetAttitudeMode() => _mode = InteractionMode.Attitude;
		public void SetOrbitMatchMode() => _mode = InteractionMode.OrbitMatch;

		public void FormGroupWithSelected()
		{
			if (_selectedMapObjects.Count <= 1) return;
			Entity firstEnt = SelectedEntity;
			if (firstEnt != null)
			{
				List<Entity> ents = new List<Entity>();
				foreach (var obj in _selectedMapObjects)
				{
					Entity e = (obj as EntityMapObject)?.Entity;
					if (e != null) ents.Add(e);
				}
				if (ents.Count > 1)
				{
					new EntityGroup(ents[0], ents.ToArray());
				}
			}
		}

		/// <summary>
		/// Determines whether the currently selected map object is either pinned or being hovered over.
		/// </summary>
		public bool IsSelectedPinnedOrHovered()
		{
			MapObject selected = SelectedMapObject;
			if (selected == null) return false;

			return _pinnedMapObjects.Contains(selected) || selected == _hoveredMapObject;
		}

		public void StartMiningLoopOnSelected()
		{
			if (SelectedEntity == null) return;

			SelectedEntity.StartMiningLoopHard();
		}

		#region Navigation

		/// <summary>
		/// Starts navigation of the selected entity towards the currently targeted entity.
		/// </summary>
		public void NavigateSelectedToTarget()
		{
			if (SelectedEntity == null)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot navigate: No entity selected.");
				return;
			}

			SelectedEntity.InterceptEntity(TargetedEntity);
		}

		/// <summary>
		/// Starts navigation of the selected entity towards a specified world position.
		/// </summary>
		public void NavigateSelectedToPosition(double2 targetPosition)
		{
			if (SelectedEntity == null)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot navigate: No entity selected.");
				return;
			}

			SelectedEntity.InterceptPosition(targetPosition);
		}

		/// <summary>
		/// Starts navigation of the selected entity to rendevous with the currently targeted entity.
		/// </summary>
		public void RendezvousSelectedToTarget()
		{
			if (SelectedEntity == null)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot navigate: No entity selected.");
				return;
			}

			SelectedEntity.RendezvousWithEntity(TargetedEntity);
		}

		/// <summary>
		/// Starts navigation of the selected entity to rendevous a specified world position.
		/// </summary>
		public void RendezvousSelectedToPosition(double2 targetPosition)
		{
			if (SelectedEntity == null)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot navigate: No entity selected.");
				return;
			}

			SelectedEntity.RendezvousWithPosition(targetPosition);
		}

		/// <summary>
		/// Starts position hold (station-keeping) of the selected entity relative to the currently targeted entity.
		/// </summary>
		public void StationKeepSelectedToTarget()
		{
			if (SelectedEntity == null)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot station keep: No entity selected.");
				return;
			}

			SelectedEntity.StationKeepWithEntity(TargetedEntity);
		}

		/// <summary>
		/// Starts position hold (station-keeping) of the selected entity relative to the target position.
		/// </summary>
		public void StationKeepSelectedToPosition(double2 targetPosition)
		{
			if (SelectedEntity == null)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot station keep: No entity selected.");
				return;
			}

			SelectedEntity.StationKeepWithPosition(targetPosition);
		}

		/// <summary>
		/// Sets the selected entity's target orientation.
		/// </summary>
		public void SetSelectedTargetOrientation(float targetOrientation)
		{
			if (SelectedEntity == null)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot set orientation: No entity selected.");
				return;
			}

			SelectedEntity.SetTargetOrientation(targetOrientation);
		}

		/// <summary>
		/// Sets the selected entity's angular velocity.
		/// </summary>
		public void SetSelectedSpin()
		{
			if (SelectedEntity == null)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot set spin: No entity selected.");
				return;
			}

			float revRate = 1f / 60f; // 1 revolution a minute
			float angularVelocity = revRate * Mathf.PI * 2;

			SelectedEntity.StartConstantSpin(angularVelocity);
		}

		#endregion
	}
}