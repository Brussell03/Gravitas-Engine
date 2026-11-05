using Gravitas2D;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Pool;
using UnityEngine.UI;

namespace GravitasDemo.Rendering
{
	/// <summary>
	/// GameObject-backed UI representation for a celestial body. Celestial materials are
	/// unique per visual and remain on the Canvas; they are deliberately excluded from the
	/// normal-entity GPU-instancing batches.
	/// </summary>
	[RequireComponent(typeof(RectTransform), typeof(CircularRaycastTarget))]
	public sealed class CelestialMapObject : MonoBehaviour, MapObject
	{
		private Entity _entity;
		private IObjectPool<CelestialMapObject> _objectPool;
		private RectTransform _rectTransform;
		private Image _imageComponent;
		private RectTransform _imageRectTransform;
		private CircularRaycastTarget _raycastTarget;
		private Material _uniqueMaterial;
		private string _materialOwnerEntityName;
		private Vector2 _viewPosition;
		private float _pixelDiameter;

		private static readonly int LightDirectionID = Shader.PropertyToID("_LightDirection");
		private static readonly int PlanetRotationID = Shader.PropertyToID("_PlanetRotation");

		public Entity Entity => _entity;
		public RectTransform RectTransform => _rectTransform;
		public bool IsDestroyed => _entity == null || _entity.IsDestroyed;
		public bool IsVisualActive => this != null && isActiveAndEnabled;
		public bool IsCelestial => true;
		public string Name => _entity != null ? _entity.Name : string.Empty;

		public Vector2 ViewPosition => _viewPosition;
		public float PixelDiameter => _pixelDiameter;
		public float RenderRotationDegrees => 0f;
		public Sprite RenderSprite => null;
		public Color RenderColor => Color.white;
		public Material RenderMaterial => _uniqueMaterial;
		public IObjectPool<CelestialMapObject> ObjectPool { set => _objectPool = value; }

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

			if (_imageComponent == null)
				Debug.LogError($"[{GetType().Name}] Celestial prefab requires an Image component.", this);
			else
				_imageComponent.raycastTarget = false;
		}

		public void InitializeBody(Entity entity)
		{
			if (entity == null)
				throw new System.ArgumentNullException(nameof(entity));
			if (!entity.IsCelestial)
				throw new System.ArgumentException("CelestialMapObject can only represent celestial entities.", nameof(entity));

			_entity = entity;
			_viewPosition = Vector2.zero;
			_pixelDiameter = 0f;

			if (_rectTransform != null)
			{
				_rectTransform.anchorMin = Vector2.zero;
				_rectTransform.anchorMax = Vector2.zero;
				_rectTransform.localRotation = Quaternion.identity;
			}

			Material baseMaterial = SimulationRenderer.Instance != null
				? SimulationRenderer.Instance.PlanetShaderMaterial
				: null;

			if (baseMaterial != null)
			{
				if (_uniqueMaterial == null || _materialOwnerEntityName != entity.Name)
				{
					DestroyUniqueMaterial();
					_uniqueMaterial = new Material(baseMaterial) { name = $"{entity.Name} Celestial Material" };
					_materialOwnerEntityName = entity.Name;
				}

				ConfigureUniqueMaterial(entity);
			}
			else
			{
				DestroyUniqueMaterial();
			}

			if (_imageComponent != null)
			{
				_imageComponent.enabled = true;
				_imageComponent.sprite = null;
				_imageComponent.color = Color.white;
				_imageComponent.material = _uniqueMaterial;
				_imageComponent.raycastTarget = false;
			}
		}

		private void ConfigureUniqueMaterial(Entity entity)
		{
			if (_uniqueMaterial == null)
				return;

			if (entity.Name.Contains("Sun") || entity.Name.Contains("Star"))
			{
				_uniqueMaterial.SetColor("_OceanColor", new Color(1.0f, 0.95f, 0.7f));
				_uniqueMaterial.SetColor("_LandColor", new Color(1.0f, 0.85f, 0.4f));
				_uniqueMaterial.SetColor("_RayleighColor", new Color(1.0f, 0.6f, 0.1f));
				_uniqueMaterial.SetFloat("_RayleighDensity", 4.0f);
			}
			else
			{
				_uniqueMaterial.SetColor("_OceanColor", new Color(0.008f, 0.035f, 0.095f));
				_uniqueMaterial.SetColor("_LandColor", new Color(0.06f, 0.18f, 0.07f));
				_uniqueMaterial.SetColor("_DesertColor", new Color(0.32f, 0.25f, 0.15f));
				_uniqueMaterial.SetColor("_RayleighColor", new Color(0.12f, 0.35f, 0.88f));
				_uniqueMaterial.SetColor("_MiegColor", new Color(0.85f, 0.92f, 1.0f));
				_uniqueMaterial.SetColor("_SunsetColor", new Color(1.0f, 0.38f, 0.08f));
				_uniqueMaterial.SetFloat("_RayleighDensity", 4.5f);
				_uniqueMaterial.SetFloat("_MieDensity", 1.0f);
			}
		}

		public void UpdateVisual()
		{
			// Celestial appearance and material are configured during InitializeBody/transform.
		}

		public void Deinitialize()
		{
			_entity = null;
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
			_pixelDiameter = Mathf.Max(0f, finalPixelSize);

			if (_rectTransform != null)
				_rectTransform.localPosition = new Vector3(localX, localY, _rectTransform.localPosition.z);
			if (_imageRectTransform != null)
				_imageRectTransform.sizeDelta = new Vector2(_pixelDiameter, _pixelDiameter);

			float margin = SimulationRenderer.Instance != null
				? SimulationRenderer.Instance.GetHoverMarginPixels()
				: 10f;
			if (_rectTransform != null && _imageRectTransform != _rectTransform)
			{
				float outerSize = _pixelDiameter + margin * 2f;
				_rectTransform.sizeDelta = new Vector2(outerSize, outerSize);
			}

			if (_entity != null && _uniqueMaterial != null)
			{
				double2 lightDirection = new double2(1.0, 0.0);
				if (EntitiesManager.Instance != null &&
					EntitiesManager.Instance.EntityLookup.TryGetValue("Sun", out Entity sunEntity))
				{
					double2 relativePosition = sunEntity.Position - _entity.Position;
					if (math.lengthsq(relativePosition) > 1e-6)
						lightDirection = math.normalize(relativePosition);
				}

				_uniqueMaterial.SetVector(LightDirectionID,
					new Vector4((float)lightDirection.x, (float)lightDirection.y, 0f, 0f));
				_uniqueMaterial.SetFloat(PlanetRotationID, (float)_entity.Orientation);
			}
		}

		public bool Raycast(Vector2 localUIPosition, float hoverMarginPixels)
		{
			return _raycastTarget != null
				? _raycastTarget.Raycast(this, localUIPosition, hoverMarginPixels)
				: CircularRaycastTarget.IsHit(this, localUIPosition, hoverMarginPixels);
		}

		private void DestroyUniqueMaterial()
		{
			if (_uniqueMaterial == null)
				return;

			if (Application.isPlaying)
				Destroy(_uniqueMaterial);
			else
				DestroyImmediate(_uniqueMaterial);

			_uniqueMaterial = null;
			_materialOwnerEntityName = null;
		}

		private void OnDestroy()
		{
			DestroyUniqueMaterial();
		}
	}
}
