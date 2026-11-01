using System.Collections.Generic;
using Gravitas2D;
using TMPro;
using Unity.Mathematics;
using UnityEngine;

namespace GravitasDemo
{
	[DefaultExecutionOrder(100)]
	public class SensorRangeVisualizer : MonoBehaviour
	{
		[Header("Reference Target Profile")]
		[SerializeField] private List<double> _targetRadii = new List<double> { 4.0, 100.0, 400.0, 1000.0 }; // Meters (used for Optical & Radar)
		[SerializeField] private List<double> _targetMasses = new List<double> { 500.0, 2000.0, 10000.0 }; // kg (used for Infrared)

		[Header("Visualization Styling")]
		[SerializeField] private Color _opticalColor = Color.cyan;
		[SerializeField] private Color _infraredColor = Color.red;
		[SerializeField] private Color _radarColor = Color.green;
		[SerializeField] private float _lineWidth = 1.5f;
		[SerializeField] private Material _lineMaterial;
		[SerializeField] private RectTransform _rangeContainer;

		private Entity _currentEntity;

		private class SensorLineGroup
		{
			public SensorCapability Sensor;
			public List<LineGraphic> Lines = new List<LineGraphic>();
			public List<TextMeshProUGUI> Labels = new List<TextMeshProUGUI>();
			public List<double> TargetValues = new List<double>();
		}

		private readonly List<SensorLineGroup> _sensorGroups = new List<SensorLineGroup>();

		private Color GetHalfSaturationColor(Color baseColor)
		{
			Color.RGBToHSV(baseColor, out float h, out float s, out float v);
			s *= 0.5f;
			Color halvedColor = Color.HSVToRGB(h, s, v);
			halvedColor.a = baseColor.a;
			return halvedColor;
		}

		public void RebuildSensorLines()
		{
			ClearLines();

			if (_currentEntity == null) return;

			foreach (var sensor in _currentEntity.GetCapabilities<SensorCapability>())
			{
				Color baseColor = sensor.Type switch
				{
					SensorType.Optical => _opticalColor,
					SensorType.Infrared => _infraredColor,
					SensorType.Radar => _radarColor,
					_ => Color.white
				};

				// Select target list based on sensor physics type
				List<double> values = (sensor.Type == SensorType.Infrared) ? _targetMasses : _targetRadii;
				if (values == null || values.Count == 0) continue;

				var group = new SensorLineGroup { Sensor = sensor };
				Color labelColor = GetHalfSaturationColor(baseColor);

				for (int i = 0; i < values.Count; i++)
				{
					// Fixed interpolation from lowest to highest interval, making higher/larger targets progressively darker
					float t = values.Count > 1 ? (float)i / (values.Count - 1) : 0f;
					Color intervalColor = Color.Lerp(baseColor, new Color(baseColor.r * 0.6f, baseColor.g * 0.6f, baseColor.b * 0.6f, baseColor.a), t);

					// Create Line Graphic
					LineGraphic graphic = LineGraphic.CreateLine($"{_currentEntity.Name}_{sensor.Type}_band_{i}", _lineWidth, intervalColor, _lineMaterial, LineType.Basic, _rangeContainer);
					group.Lines.Add(graphic);
					group.TargetValues.Add(values[i]);

					// Create Text Label for Line Ring
					GameObject labelObj = new GameObject($"SensorLabel_{sensor.Type}_{i}");
					labelObj.transform.SetParent(_rangeContainer, false);

					RectTransform rect = labelObj.AddComponent<RectTransform>();
					rect.anchorMin = new Vector2(0.5f, 0.5f);
					rect.anchorMax = new Vector2(0.5f, 0.5f);
					rect.pivot = new Vector2(0.0f, 0.5f); // Left-aligned next to apex

					TextMeshProUGUI tmp = labelObj.AddComponent<TextMeshProUGUI>();
					tmp.fontSize = 11;
					tmp.color = labelColor;
					tmp.raycastTarget = false;
					tmp.alignment = TextAlignmentOptions.Left;

					double val = values[i];
					if (sensor.Type == SensorType.Infrared)
					{
						tmp.text = val >= 1000.0 ? $"{val / 1000.0:0.#}t" : $"{val:0}kg";
					}
					else
					{
						tmp.text = val >= 1000.0 ? $"{val / 1000.0:0.#}km" : $"{val:0}m";
					}

					group.Labels.Add(tmp);
				}

				_sensorGroups.Add(group);
			}
		}

		private void ClearLines()
		{
			foreach (var group in _sensorGroups)
			{
				foreach (var line in group.Lines)
				{
					if (line != null) Destroy(line.gameObject);
				}
				foreach (var label in group.Labels)
				{
					if (label != null) Destroy(label.gameObject);
				}
			}
			_sensorGroups.Clear();
		}

		void LateUpdate()
		{
			if (SimulationRenderer.Instance == null) return;
			if (SimulationInteractionController.Instance == null) return;

			var entity = SimulationInteractionController.Instance.SelectedEntity;

			if (entity != _currentEntity)
			{
				_currentEntity = entity;
				RebuildSensorLines();
			}

			if (entity == null || entity.IsDestroyed) return;

			var renderer = SimulationRenderer.Instance;
			double2 centerSim = _currentEntity.Position;

			foreach (var group in _sensorGroups)
			{
				for (int i = 0; i < group.Lines.Count; i++)
				{
					var graphic = group.Lines[i];
					var label = group.Labels[i];
					if (graphic == null) continue;

					double targetValue = group.TargetValues[i];
					float effectiveRange = CalculateEffectiveRange(group.Sensor, targetValue);

					Vector2[] points = GenerateSensorShapePoints(_currentEntity, group.Sensor, effectiveRange, out double2 apexSimPos);
					graphic.SetPoints(points);
					graphic.gameObject.SetActive(true);

					if (label != null)
					{
						Vector2 uiPos = renderer.SimPositionToViewUI(apexSimPos);
						label.rectTransform.anchoredPosition = uiPos + new Vector2(6f, 6f); // Slight offset from line apex
						label.gameObject.SetActive(true);
					}
				}
			}
		}

		private float CalculateEffectiveRange(SensorCapability sensor, double targetValue)
		{
			double calculatedDist = sensor.Range;

			// Inverting FilterEntitiesJob detection scoring formulas
			switch (sensor.Type)
			{
				case SensorType.Optical:
					// score = (targetRadius^2 / dist^2) * 1e4 * sensitivity >= 1.0
					double optDist = targetValue * math.sqrt(1e4 * sensor.Sensitivity);
					calculatedDist = math.min(sensor.Range, optDist);
					break;
				case SensorType.Infrared:
					// score = (targetMass / 1000.0 / dist^2) * 1e8 * sensitivity >= 1.0
					double irDist = math.sqrt((targetValue / 1000.0) * 1e8 * sensor.Sensitivity);
					calculatedDist = math.min(sensor.Range, irDist);
					break;
				case SensorType.Radar:
					// score = (targetRadius^2 / dist^4) * 1e15 * sensitivity >= 1.0
					double radarDist = math.pow((targetValue * targetValue) * 1e15 * sensor.Sensitivity, 0.25);
					calculatedDist = math.min(sensor.Range, radarDist);
					break;
			}

			return (float)calculatedDist;
		}

		private Vector2[] GenerateSensorShapePoints(Entity entity, SensorCapability sensor, float range, out double2 apexSimPos)
		{
			double2 centerSim = entity.Position;
			float fov = sensor.FieldOfViewDegrees;

			var renderer = SimulationRenderer.Instance;

			// Default apex position at 45 degrees along the range circle
			apexSimPos = centerSim + new double2(math.cos(math.PI * 0.25), math.sin(math.PI * 0.25)) * range;

			if (fov >= 360f)
			{
				// Full Circle Generation
				int segments = 64;
				Vector2[] points = new Vector2[segments + 1];
				for (int i = 0; i <= segments; i++)
				{
					float angle = (i / (float)segments) * 2.0f * Mathf.PI;
					double2 simPos = centerSim + new double2(math.cos(angle), math.sin(angle)) * range;

					points[i] = renderer.SimPositionToViewUI(simPos);
				}
				return points;
			}
			else
			{
				// Sector Wedge: Center -> Arc Start -> Arc Points -> Arc End -> Center
				float baseHeading = entity.Orientation + sensor.OrientationOffset;
				float halfFovRad = (fov * 0.5f) * Mathf.Deg2Rad;
				float startAngle = baseHeading - halfFovRad;
				float endAngle = baseHeading + halfFovRad;

				// Place label at center of wedge arc
				float midAngle = (startAngle + endAngle) * 0.5f;
				apexSimPos = centerSim + new double2(math.cos(midAngle), math.sin(midAngle)) * range;

				int arcSegments = 32;
				Vector2[] points = new Vector2[arcSegments + 3];

				// Center point calculation
				Vector2 centerUI = renderer.SimPositionToViewUI(centerSim);
				points[0] = centerUI;

				for (int i = 0; i <= arcSegments; i++)
				{
					float t = i / (float)arcSegments;
					float angle = Mathf.Lerp(startAngle, endAngle, t);
					double2 simPos = centerSim + new double2(math.cos(angle), math.sin(angle)) * range;

					points[i + 1] = renderer.SimPositionToViewUI(simPos);
				}

				points[arcSegments + 2] = centerUI;
				return points;
			}
		}

		void OnDestroy()
		{
			ClearLines();
		}
	}
}