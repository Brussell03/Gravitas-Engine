using System.Collections.Generic;
using Gravitas2D;
using GravitasDemo.Interaction;
using Unity.Mathematics;
using UnityEngine;

namespace GravitasDemo.Rendering
{
	[DefaultExecutionOrder(100)]
	public class FiringRangeVisualizer : MonoBehaviour
	{
		[Header("Time-of-Flight Intervals (Seconds)")]
		[SerializeField] private List<double> _timeOfFlights = new List<double> { 600.0, 1800.0, 3600.0 }; // e.g., 10m, 30m, 1h

		[Header("Resolution & Collision Checking")]
		[SerializeField, Range(36, 360)] private int _circleResolution = 36;
		[SerializeField, Range(2, 50)] private int _collisionCheckSubSteps = 10;

		[Header("Visualization Styling")]
		[SerializeField] private Color _baseFiringColor = Color.yellow;
		[SerializeField] private float _lineWidth = 1.5f;
		[SerializeField] private Material _lineMaterial;
		[SerializeField] private RectTransform _rangeContainer;

		private Entity _currentEntity;

		private class FiringLineGroup
		{
			public double TimeOfFlight;
			public Color IntervalColor;
			public readonly List<LineGraphic> ActiveGraphics = new List<LineGraphic>();
		}

		private readonly List<FiringLineGroup> _firingGroups = new List<FiringLineGroup>();

		public void RebuildFiringLines()
		{
			ClearLines();

			if (_currentEntity == null) return;

			var launchCap = _currentEntity.GetCapability<RailgunCapability>();
			if (launchCap == null) return;

			if (_timeOfFlights == null || _timeOfFlights.Count == 0) return;

			for (int i = 0; i < _timeOfFlights.Count; i++)
			{
				float t = _timeOfFlights.Count > 1 ? (float)i / (_timeOfFlights.Count - 1) : 0f;
				Color intervalColor = Color.Lerp(_baseFiringColor, new Color(_baseFiringColor.r * 0.4f, _baseFiringColor.g * 0.4f, _baseFiringColor.b * 0.4f, _baseFiringColor.a), t);

				_firingGroups.Add(new FiringLineGroup
				{
					TimeOfFlight = _timeOfFlights[i],
					IntervalColor = intervalColor
				});
			}
		}

		private void ClearLines()
		{
			foreach (var group in _firingGroups)
			{
				foreach (var graphic in group.ActiveGraphics)
				{
					if (graphic != null) Destroy(graphic.gameObject);
				}
				group.ActiveGraphics.Clear();
			}
			_firingGroups.Clear();
		}

		void LateUpdate()
		{
			if (SimulationRenderer.Instance == null) return;
			if (SimulationInteractionController.Instance == null) return;

			var entity = SimulationInteractionController.Instance.SelectedEntity;

			if (entity != _currentEntity)
			{
				_currentEntity = entity;
				RebuildFiringLines();
			}

			if (entity == null || entity.IsDestroyed) return;

			var launchCap = entity.GetCapability<RailgunCapability>();
			if (launchCap == null)
			{
				ClearLines();
				return;
			}

			PhysicsBody parentSoI = entity.PhysicsBody.ParentSoI;
			if (parentSoI == null) return;

			Entity dominantEntity = entity.DominantEntity;

			// Calculate projectile mass from composition profile
			double projectileDensity = PhysicsManager2D.Instance.CompositionProfiles[launchCap.ProjectileCompositionIndex].Density;
			double projectileMass = 4.0 / 3.0 * math.PI * math.pow(launchCap.ProjectileRadius, 3) * projectileDensity;
			double mu = OrbitalMath.GetStandardGravitationalParameter(parentSoI.Mass, projectileMass);

			double2 entityPos = entity.Position;
			double2 entityVel = entity.Velocity;
			double fireSpeed = launchCap.FireSpeed;

			var renderer = SimulationRenderer.Instance;
			Bounds2D bounds = renderer.ViewBounds;
			Rect rect = renderer.VisualizationArea.rect;

			foreach (var group in _firingGroups)
			{
				double tof = group.TimeOfFlight;
				
				bool[] isValid = new bool[_circleResolution + 1];
				Vector2[] uiPoints = new Vector2[_circleResolution + 1];

				// 1. Evaluate sampling angles and check collisions
				for (int i = 0; i <= _circleResolution; i++)
				{
					float angle = (i / (float)_circleResolution) * 2.0f * Mathf.PI;
					double2 dir = new double2(math.cos(angle), math.sin(angle));
					double2 launchVel = entityVel + (dir * fireSpeed);

					bool collided = false;
					if (dominantEntity != null)
					{
						int totalSteps = _collisionCheckSubSteps + 1;
						for (int step = 1; step <= totalSteps; step++)
						{
							double subT = (step / (double)totalSteps) * tof;
							OrbitalMath.GetLocalPosAtTime(entityPos, launchVel, parentSoI, subT, mu, out double2 subPos);

							if (math.length(subPos) <= dominantEntity.Radius)
							{
								collided = true;
								break;
							}
						}
					}

					if (collided)
					{
						isValid[i] = false;
						continue;
					}

					// Propagate forward to target time of flight
					OrbitalMath.GetLocalPosAtTime(entityPos, launchVel, parentSoI, tof, mu, out double2 projectedPos);

					if (dominantEntity != null && math.length(projectedPos) <= dominantEntity.Radius)
					{
						isValid[i] = false;
						continue;
					}

					isValid[i] = true;
					uiPoints[i] = renderer.SimPositionToViewUI(projectedPos + dominantEntity.Position);
				}

				// 2. Extract continuous segments of valid points (accounting for circular wrap-around at index 0 / resolution)
				List<List<Vector2>> segments = new List<List<Vector2>>();
				List<Vector2> currentSegment = null;

				bool wrapped = isValid[0] && isValid[_circleResolution];

				for (int i = 0; i < _circleResolution; i++)
				{
					if (isValid[i])
					{
						if (currentSegment == null) currentSegment = new List<Vector2>();
						currentSegment.Add(uiPoints[i]);
					}
					else
					{
						if (currentSegment != null)
						{
							segments.Add(currentSegment);
							currentSegment = null;
						}
					}
				}

				if (isValid[_circleResolution])
				{
					if (wrapped && segments.Count > 0)
					{
						segments[0].Add(uiPoints[_circleResolution]);
					}
					else
					{
						if (currentSegment == null) currentSegment = new List<Vector2>();
						currentSegment.Add(uiPoints[_circleResolution]);
					}
				}

				if (currentSegment != null)
				{
					segments.Add(currentSegment);
				}

				// 3. Process segments: Close open arcs by connecting their endpoints directly to each other
				List<Vector2[]> finalizedSegments = new List<Vector2[]>();
				for (int s = 0; s < segments.Count; s++)
				{
					var seg = segments[s];
					if (seg.Count < 2) continue;

					bool isClosedLoop = wrapped && segments.Count == 1 && s == 0;
					if (!isClosedLoop)
					{
						// Open arc clipped by planet: close the gap by connecting the end point back to the start point
						List<Vector2> closedSeg = new List<Vector2>(seg);
						closedSeg.Add(seg[0]);
						finalizedSegments.Add(closedSeg.ToArray());
					}
					else
					{
						finalizedSegments.Add(seg.ToArray());
					}
				}

				// 4. Pool and update LineGraphics for each finalized segment
				while (group.ActiveGraphics.Count < finalizedSegments.Count)
				{
					LineGraphic graphic = LineGraphic.CreateLine($"{_currentEntity.Name}_FiringRange_{tof}_seg_{group.ActiveGraphics.Count}", _lineWidth, group.IntervalColor, _lineMaterial, LineType.Basic, _rangeContainer);
					group.ActiveGraphics.Add(graphic);
				}

				for (int s = 0; s < group.ActiveGraphics.Count; s++)
				{
					var graphic = group.ActiveGraphics[s];
					if (graphic == null) continue;

					if (s < finalizedSegments.Count)
					{
						graphic.SetPoints(finalizedSegments[s]);
						graphic.gameObject.SetActive(true);
					}
					else
					{
						graphic.gameObject.SetActive(false);
					}
				}
			}
		}

		void OnDestroy()
		{
			ClearLines();
		}
	}
}