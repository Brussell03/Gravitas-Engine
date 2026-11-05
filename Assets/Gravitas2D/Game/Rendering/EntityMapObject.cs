using Gravitas2D;
using UnityEngine;

namespace GravitasDemo.Rendering
{
	/// <summary>
	/// Data-only visual and interaction handle for a normal (non-celestial) entity.
	/// This class intentionally does not derive from MonoBehaviour and creates no GameObject.
	/// All visible pixels are submitted by SimulationRenderer through Graphics.RenderMeshInstanced.
	/// </summary>
	public sealed class EntityMapObject : MapObject
	{
		private Entity _entity;
		private bool _lastObservedState;
		private Vector2 _viewPosition;
		private float _pixelDiameter;
		private float _renderRotationDegrees;
		private Sprite _renderSprite;
		private Color _renderColor = Color.white;
		private Color _baseColor = Color.white;

		public Entity Entity => _entity;
		public PhysicsBody PhysicsBody => _entity != null ? _entity.PhysicsBody : null;
		public bool LastObservedState => _lastObservedState;
		public RectTransform RectTransform => null;
		public bool IsDestroyed => _entity == null || _entity.IsDestroyed;
		public bool IsVisualActive => _entity != null && !_entity.IsDestroyed;
		public bool IsCelestial => false;
		public string Name => _entity != null ? _entity.Name : string.Empty;

		public Vector2 ViewPosition => _viewPosition;
		public float PixelDiameter => _pixelDiameter;
		public float RenderRotationDegrees => _renderRotationDegrees;
		public Sprite RenderSprite => _renderSprite;
		public Color RenderColor => _renderColor;
		public Material RenderMaterial => null;

		public void InitializeBody(Entity entity)
		{
			if (entity == null)
				throw new System.ArgumentNullException(nameof(entity));
			if (entity.IsCelestial)
				throw new System.ArgumentException("Celestial entities must use CelestialMapObject.", nameof(entity));

			_entity = entity;
			_lastObservedState = entity.IsObserved;
			_renderSprite = SimulationRenderer.Instance != null
				? SimulationRenderer.Instance.GetBodyImage(entity.Type)
				: null;

			_baseColor = entity.IsOwned && (entity.IsVehicle || entity.IsStation)
				? Color.royalBlue
				: Color.floralWhite;
			_renderColor = entity.IsObserved ? _baseColor : _baseColor * 0.6f;

			_pixelDiameter = 0f;
			_renderRotationDegrees = 0f;
			_viewPosition = Vector2.zero;
		}

		public void UpdateVisual()
		{
			if (_entity == null)
				return;

			_lastObservedState = _entity.IsObserved;
			_renderColor = _entity.IsObserved ? _baseColor : _baseColor * 0.6f;
		}

		public void Deinitialize()
		{
			_entity = null;
			_renderSprite = null;
			_pixelDiameter = 0f;
			_renderRotationDegrees = 0f;
			_viewPosition = Vector2.zero;
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
			_pixelDiameter = Mathf.Max(0f, finalPixelSize);

			// Match the orientation previously applied to the entity's RectTransform.
			_renderRotationDegrees = rotationRadians * Mathf.Rad2Deg - 90f;
		}

		public bool Raycast(Vector2 localUIPosition, float hoverMarginPixels)
		{
			return CircularRaycastTarget.IsHit(this, localUIPosition, hoverMarginPixels);
		}
	}
}