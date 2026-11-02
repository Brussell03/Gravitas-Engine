using Gravitas2D;
using GravitasDemo.Interaction;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Pool;
using UnityEngine.UI;

namespace GravitasDemo.Rendering
{
	public class EntityGroupMapObject : MapObject
	{
		private EntityGroup _group;
		private IObjectPool<EntityGroupMapObject> _objectPool;

		public EntityGroup Group => _group;
		public double2 SimPos => _group.Position;
		public override bool IsDestroyed => _group == null || _group.IsDestroyed;
		public override string Name => _group.Name;

		public IObjectPool<EntityGroupMapObject> ObjectPool { set => _objectPool = value; }

		public void InitializeBody(EntityGroup group)
		{
			_group = group;

			_imageComponent.material = null;

			_baseColor = Color.floralWhite;
			_imageComponent.color = _baseColor;
			
			_lastScaleReference = -1;

			_rectTransform.anchorMin = Vector2.zero;
			_rectTransform.anchorMax = Vector2.zero;

			UpdateVisual();
		}

		public override void UpdateVisual()
		{
		}

		public override void Deinitialize()
		{
			_group = null;
			if (_imageComponent != null)
			{
				_imageComponent.material = null;
			}
		}

		public void Deactivate()
		{
			_objectPool.Release(this);
		}

		public override void UpdateTransform(float percX, float percY, float rectWidth, float rectHeight, double boundsSizeX, float minVisiblePixelSize)
		{
			base.UpdateTransform(percX, percY, rectWidth, rectHeight, boundsSizeX, minVisiblePixelSize);

			// Change Detection: Only update Unity's transform if the zoom level actually changed
			if (math.abs(_lastScaleReference - boundsSizeX) < 1e-5) return;
			
			_lastScaleReference = boundsSizeX;

			float accuratePixelSize = (float)(100 / boundsSizeX) * rectWidth;;

			// Clamp to ensure visibility
			float finalPixelSize = Mathf.Max(accuratePixelSize, minVisiblePixelSize);

			if (_imageRectTransform != null)
			{
				_imageRectTransform.sizeDelta = new Vector2(finalPixelSize, finalPixelSize);
			}

			float hoverMarginPixels = 10f;
			if (SimulationRenderer.Instance != null && SimulationRenderer.Instance.VisualizationArea != null)
			{
				float pct = SimulationInteractionController.Instance != null 
					? SimulationInteractionController.Instance.HoverMarginScreenPct 
					: 0.01f;
				hoverMarginPixels = SimulationRenderer.Instance.VisualizationArea.rect.width * pct;
			}

			float totalRectSize = finalPixelSize + (hoverMarginPixels * 2f);
			_rectTransform.sizeDelta = new Vector2(totalRectSize, totalRectSize);
		}
	}
}