using Gravitas2D;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Pool;
using UnityEngine.UI;

namespace GravitasDemo.Rendering
{
	[RequireComponent(typeof(RectTransform), typeof(CircularRaycastTarget))]
	public sealed class EntityGroupMapObject : MonoBehaviour, MapObject
	{
		private EntityGroup _group;
		private IObjectPool<EntityGroupMapObject> _objectPool;
		private RectTransform _rectTransform;
		private Image _imageComponent;
		private RectTransform _imageRectTransform;
		private CircularRaycastTarget _raycastTarget;
		private Vector2 _viewPosition;
		private float _pixelDiameter;

		public Entity Entity => null;
		public EntityGroup Group => _group;
		public double2 SimPos => _group != null ? _group.Position : double2.zero;
		public RectTransform RectTransform => _rectTransform;
		public bool IsDestroyed => _group == null || _group.IsDestroyed;
		public bool IsVisualActive => this != null && isActiveAndEnabled;
		public bool IsCelestial => false;
		public string Name => _group != null ? _group.Name : string.Empty;

		public Vector2 ViewPosition => _viewPosition;
		public float PixelDiameter => _pixelDiameter;
		public float RenderRotationDegrees => 0f;
		public Sprite RenderSprite => _imageComponent != null ? _imageComponent.sprite : null;
		public Color RenderColor => _imageComponent != null ? _imageComponent.color : Color.floralWhite;
		public Material RenderMaterial => null;

		public IObjectPool<EntityGroupMapObject> ObjectPool { set => _objectPool = value; }

		private void Awake()
		{
			_rectTransform = GetComponent<RectTransform>();
			_imageComponent = GetComponentInChildren<Image>(true);
			_imageRectTransform = _imageComponent != null ? _imageComponent.rectTransform : null;
			_raycastTarget = GetComponent<CircularRaycastTarget>();

			if (_imageRectTransform != null && _imageRectTransform != _rectTransform)
			{
				_imageRectTransform.anchorMin = new Vector2(0.5f, 0.5f);
				_imageRectTransform.anchorMax = new Vector2(0.5f, 0.5f);
				_imageRectTransform.anchoredPosition = Vector2.zero;
			}

			if (_imageComponent != null)
				_imageComponent.raycastTarget = false;
		}

		public void InitializeBody(EntityGroup group)
		{
			if (group == null)
				throw new System.ArgumentNullException(nameof(group));

			_group = group;
			_viewPosition = Vector2.zero;
			_pixelDiameter = 0f;

			if (_rectTransform != null)
			{
				_rectTransform.anchorMin = Vector2.zero;
				_rectTransform.anchorMax = Vector2.zero;
			}

			if (_imageComponent != null)
			{
				_imageComponent.enabled = true;
				_imageComponent.material = null;
				_imageComponent.color = Color.floralWhite;
			}
		}

		public void UpdateVisual()
		{
			// Group appearance is constant.
		}

		public void Deinitialize()
		{
			_group = null;
			_pixelDiameter = 0f;
			_viewPosition = Vector2.zero;
			if (_imageComponent != null)
				_imageComponent.material = null;
		}

		public void Deactivate()
		{
			if (_objectPool != null)
				_objectPool.Release(this);
		}

		public void ApplyTransformData(
			float percX,
			float percY,
			float finalPixelSize,
			float rotationRadians,
			double boundsSizeX,
			float rectXMin,
			float rectXMax,
			float rectYMin,
			float rectYMax)
		{
			float localX = Mathf.LerpUnclamped(rectXMin, rectXMax, percX);
			float localY = Mathf.LerpUnclamped(rectYMin, rectYMax, percY);
			_viewPosition = new Vector2(localX, localY);

			if (_rectTransform != null)
				_rectTransform.localPosition = new Vector3(localX, localY, _rectTransform.localPosition.z);

			float rectWidth = rectXMax - rectXMin;
			float accuratePixelSize = boundsSizeX > 0.0
				? (float)(100.0 / boundsSizeX) * rectWidth
				: 0f;
			_pixelDiameter = Mathf.Max(accuratePixelSize, finalPixelSize);

			float hoverMarginPixels = SimulationRenderer.Instance != null
				? SimulationRenderer.Instance.GetHoverMarginPixels()
				: 10f;
			if (_imageRectTransform != null)
				_imageRectTransform.sizeDelta = new Vector2(_pixelDiameter, _pixelDiameter);
			if (_rectTransform != null && _imageRectTransform != _rectTransform)
			{
				float totalRectSize = _pixelDiameter + hoverMarginPixels * 2f;
				_rectTransform.sizeDelta = new Vector2(totalRectSize, totalRectSize);
			}
		}

		public bool Raycast(Vector2 localUIPosition, float hoverMarginPixels)
		{
			return _raycastTarget != null
				? _raycastTarget.Raycast(this, localUIPosition, hoverMarginPixels)
				: CircularRaycastTarget.IsHit(this, localUIPosition, hoverMarginPixels);
		}
	}
}
