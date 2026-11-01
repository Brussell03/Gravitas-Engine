using UnityEngine;
using UnityEngine.UI;

namespace GravitasDemo
{
	public class CircularRaycastTarget : Graphic
	{
		private EntityMapObject _entityMapObject;
		private EntityGroupMapObject _groupMapObject;

		protected override void Awake()
		{
			base.Awake();
			raycastTarget = true;
			_entityMapObject = GetComponent<EntityMapObject>();
			_groupMapObject = GetComponent<EntityGroupMapObject>();
		}

		// Disable rendering/drawing since this is purely a raycast target
		public override void Rebuild(CanvasUpdate update) { }
		public override Texture mainTexture => null;
		protected override void OnPopulateMesh(VertexHelper vh) { vh.Clear(); }

		public override bool Raycast(Vector2 sp, Camera eventCamera)
		{
			if (!isActiveAndEnabled) return false;

			Vector2 localPoint;
			if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(rectTransform, sp, eventCamera, out localPoint))
				return false;

			// Retrieve hover margin dynamically
			float hoverMarginPixels = 10f;
			if (SimulationRenderer.Instance != null && SimulationRenderer.Instance.VisualizationArea != null)
			{
				float pct = SimulationInteractionController.Instance != null 
					? SimulationInteractionController.Instance.HoverMarginScreenPct 
					: 0.01f;
				hoverMarginPixels = SimulationRenderer.Instance.VisualizationArea.rect.width * pct;
			}

			// Since sizeDelta now encompasses (visualSize + margin * 2), 
			// baseRadius is derived by subtracting the margin from the half-size.
			float halfSize = rectTransform.sizeDelta.x * 0.5f;
			float baseRadius = Mathf.Max(0f, halfSize - hoverMarginPixels);

			// Account for celestial atmospheric scale multiplier (2.6f vs 2.0f) 
			if (_entityMapObject != null && _entityMapObject.Entity != null && _entityMapObject.Entity.IsCelestial)
			{
				baseRadius *= (2.0f / 2.6f);
			}

			float totalHitRadius = baseRadius + hoverMarginPixels;
			
			// Strict circular radius check including hover margin
			return localPoint.sqrMagnitude <= (totalHitRadius * totalHitRadius);
		}
	}
}