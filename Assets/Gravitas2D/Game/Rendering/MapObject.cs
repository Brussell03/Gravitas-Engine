using UnityEngine;
using UnityEngine.UI;

namespace GravitasDemo
{
	[RequireComponent(typeof(CircularRaycastTarget))]
	public abstract class MapObject : MonoBehaviour
	{
		protected RectTransform _rectTransform;
		protected Image _imageComponent;
		protected RectTransform _imageRectTransform;
		protected CircularRaycastTarget _raycastTarget;
		protected double _lastScaleReference = -1;
		protected Color _baseColor;

		public RectTransform RectTransform => _rectTransform;
		public abstract bool IsDestroyed { get; }
		public abstract string Name { get; }

		protected virtual void Awake()
		{
			_rectTransform = GetComponent<RectTransform>();
			_imageComponent = GetComponentInChildren<Image>();

			_raycastTarget = GetComponent<CircularRaycastTarget>();
			if (_raycastTarget == null)
			{
				_raycastTarget = gameObject.AddComponent<CircularRaycastTarget>();
			}

			if (_imageComponent != null)
			{
				_imageComponent.raycastTarget = false;

				_imageRectTransform = _imageComponent.GetComponent<RectTransform>();
				if (_imageRectTransform != null && _imageRectTransform != _rectTransform)
				{
					_imageRectTransform.anchorMin = new Vector2(0.5f, 0.5f);
					_imageRectTransform.anchorMax = new Vector2(0.5f, 0.5f);
					_imageRectTransform.anchoredPosition = Vector2.zero;
				}
			}

			if (_rectTransform == null)
			{
				Debug.LogError($"[{GetType().Name}] Missing RectTransform component.", this);
			}
		}

		public abstract void UpdateVisual();
		public abstract void Deinitialize();

		public virtual void UpdateTransform(float percX, float percY, float rectWidth, float rectHeight, double boundsSizeX, float minVisiblePixelSize)
		{
			_rectTransform.anchoredPosition = new Vector2(percX * rectWidth, percY * rectHeight);
		}
	}
}