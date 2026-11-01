using Gravitas2D;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Pool;
using UnityEngine.UI;

namespace GravitasDemo
{
	public class EntityMapObject : MapObject
	{
		private Entity _entity;
		private IObjectPool<EntityMapObject> _objectPool;
		private bool _lastObservedState = false;

		private Material _celestialMaterialInstance;
		private static readonly int LightDirectionID = Shader.PropertyToID("_LightDirection");
		private static readonly int PlanetRotationID = Shader.PropertyToID("_PlanetRotation");

		public Entity Entity => _entity;
		public PhysicsBody PhysicsBody => _entity.PhysicsBody;
		public bool LastObservedState => _lastObservedState;
		public override bool IsDestroyed => _entity == null || _entity.IsDestroyed;
		public override string Name => _entity.Name;

		public IObjectPool<EntityMapObject> ObjectPool { set => _objectPool = value; }

		public void InitializeBody(Entity entity)
		{
			_entity = entity;
			_lastObservedState = entity.IsObserved;
			_imageComponent.sprite = SimulationRenderer.Instance.GetBodyImage(entity.Type);

			if (entity.IsCelestial)
			{
				_imageComponent.sprite = null;

				Material baseMat = SimulationRenderer.Instance != null ? SimulationRenderer.Instance.PlanetShaderMaterial : null;
				if (baseMat != null)
				{
					if (_celestialMaterialInstance == null)
					{
						_celestialMaterialInstance = new Material(baseMat);
					}

					if (entity.Name.Contains("Sun") || entity.Name.Contains("Star"))
					{
						_celestialMaterialInstance.SetColor("_OceanColor", new Color(1.0f, 0.95f, 0.7f));
						_celestialMaterialInstance.SetColor("_LandColor", new Color(1.0f, 0.85f, 0.4f));
						_celestialMaterialInstance.SetColor("_RayleighColor", new Color(1.0f, 0.6f, 0.1f));
						_celestialMaterialInstance.SetFloat("_RayleighDensity", 4.0f);
					}
					else
					{
						_celestialMaterialInstance.SetColor("_OceanColor", new Color(0.008f, 0.035f, 0.095f));
						_celestialMaterialInstance.SetColor("_LandColor", new Color(0.06f, 0.18f, 0.07f));
						_celestialMaterialInstance.SetColor("_DesertColor", new Color(0.32f, 0.25f, 0.15f));
						_celestialMaterialInstance.SetColor("_RayleighColor", new Color(0.12f, 0.35f, 0.88f));
						_celestialMaterialInstance.SetColor("_MiegColor", new Color(0.85f, 0.92f, 1.0f));
						_celestialMaterialInstance.SetColor("_SunsetColor", new Color(1.0f, 0.38f, 0.08f));
						_celestialMaterialInstance.SetFloat("_RayleighDensity", 4.5f);
						_celestialMaterialInstance.SetFloat("_MieDensity", 1.0f);
					}

					_imageComponent.material = _celestialMaterialInstance;
				}

				_baseColor = Color.white;
				_imageComponent.color = _baseColor;
			}
			else
			{
				_imageComponent.material = null;
				_imageComponent.sprite = SimulationRenderer.Instance.GetBodyImage(entity.Type);

				_baseColor = _entity.IsOwned && _entity.IsVehicle ? Color.royalBlue : Color.floralWhite;
				_imageComponent.color = entity.IsObserved ? _baseColor : _baseColor * 0.6f;
			}
			
			_lastScaleReference = -1;

			_rectTransform.anchorMin = Vector2.zero;
			_rectTransform.anchorMax = Vector2.zero;

			UpdateVisual();
		}

		public override void UpdateVisual()
		{
			_lastObservedState = _entity.IsObserved;
			if (_entity.IsCelestial) return;

			bool isObserved = _entity.IsObserved;
			_imageComponent.color = isObserved ? _baseColor : _baseColor * 0.6f;
		}

		public override void Deinitialize()
		{
			_entity = null;
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

			if (_entity != null && _entity.IsCelestial)
			{
				_rectTransform.rotation = Quaternion.identity;

				if (_celestialMaterialInstance != null)
				{
					// Calculate world-space light direction from dominant entity (Sun)
					double2 lightDir = new double2(1, 0);
					if (EntitiesManager.Instance.EntityLookup.TryGetValue("Sun", out var sunEntity))
					{
						double2 relPos = sunEntity.Position - _entity.Position;
						if (math.lengthsq(relPos) > 1e-6)
						{
							lightDir = math.normalize(relPos);
						}
					}

					// Pass world-space light direction directly (do not transform by orientation)
					_celestialMaterialInstance.SetVector(LightDirectionID, new Vector4((float)lightDir.x, (float)lightDir.y, 0, 0));
					_celestialMaterialInstance.SetFloat(PlanetRotationID, (float)_entity.Orientation);
				}
			}
			else
			{
				_rectTransform.rotation = Quaternion.Euler(0, 0, Mathf.Rad2Deg * _entity.Orientation - 90f);
			}

			// Change Detection: Only update Unity's transform if the zoom level actually changed
			if (math.abs(_lastScaleReference - boundsSizeX) < 1e-5) return;
			
			_lastScaleReference = boundsSizeX;

			// Add additional visual padding to accommodate atmospheric halo
			float radiusMultiplier = _entity != null && _entity.IsCelestial ? 2.6f : 2.0f;
			float accuratePixelSize = (float)(PhysicsBody.Radius / boundsSizeX) * rectWidth * radiusMultiplier;

			// Clamp to ensure visibility
			float finalPixelSize = Mathf.Max(accuratePixelSize, minVisiblePixelSize);

			// Explicitly set child visual image to its correct, unbloated size
			if (_imageRectTransform != null)
			{
				_imageRectTransform.sizeDelta = new Vector2(finalPixelSize, finalPixelSize);
			}

			// Inflate parent RectTransform size so the raycaster can detect the hover margin
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