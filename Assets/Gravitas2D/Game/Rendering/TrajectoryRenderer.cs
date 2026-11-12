using System.Collections.Generic;
using Gravitas2D;
using GravitasDemo.Interaction;
using TMPro;
using Unity.Mathematics;
using UnityEngine;

namespace GravitasDemo.Rendering
{
	public class TrajectoryRenderer
	{
		private float _lineWidth = 1.0f;
		private int _orbitResolution = 360;
		private Material _lineMaterial;
		private RectTransform _container;
		private Color _selectedColor = new Color(1f, 0.92f, 0.015f, 1f);  // Yellow
		private Color _hoveredColor = new Color(0.2f, 0.8f, 1f, 0.8f);    // Cyan
		private Color _targetedColor = new Color(1f, 0.25f, 0.25f, 0.8f); // Red
		private Color _pinnedColor = new Color(0.25f, 1f, 0.4f, 0.8f);    // Green
		private Color _soiLineColor = new Color(0.2f, 0.8f, 1f, 0.4f);
		private Color _navLineColor = new Color(1f, 0.6f, 0.2f, 0.7f);    // Amber for navigation line
		private Color _orbitMatchColor = new Color(0.8f, 0.2f, 1f, 0.6f); // Magenta for orbit match preview

		// Configurable time offsets in seconds for trajectory time-to labels
		public List<double> TimeLabelsOffsets = new List<double> { 120.0, 600.0, 1800.0, 3600.0 };

		private Dictionary<Entity, LineGraphic> _trajectories = new Dictionary<Entity, LineGraphic>();
		private Dictionary<Entity, LineGraphic> _soiGraphics = new Dictionary<Entity, LineGraphic>();
		private Dictionary<Entity, LineGraphic> _navLines = new Dictionary<Entity, LineGraphic>();
		private Dictionary<Entity, LineGraphic> _orbitMatchGraphics = new Dictionary<Entity, LineGraphic>();
		private Dictionary<Entity, List<TextMeshProUGUI>> _timeLabels = new Dictionary<Entity, List<TextMeshProUGUI>>();
		private Dictionary<Entity, Color> _activeEntityColors = new Dictionary<Entity, Color>();
		private List<Entity> _entitiesToRemove = new List<Entity>();
		private Vector2[] _pathBuffer;

		public TrajectoryRenderer(RectTransform container)
		{
			_pathBuffer = new Vector2[_orbitResolution];
			_container = container;
		}

		public void Cleanup()
		{
			ClearTrajectories();
		}

		public void ClearTrajectories()
		{
			foreach (var kvp in _trajectories)
			{
				if (kvp.Value != null) GameObject.Destroy(kvp.Value.gameObject);
			}
			_trajectories.Clear();

			foreach (var kvp in _soiGraphics)
			{
				if (kvp.Value != null) GameObject.Destroy(kvp.Value.gameObject);
			}
			_soiGraphics.Clear();

			foreach (var kvp in _navLines)
			{
				if (kvp.Value != null) GameObject.Destroy(kvp.Value.gameObject);
			}
			_navLines.Clear();

			foreach (var kvp in _orbitMatchGraphics)
			{
				if (kvp.Value != null) GameObject.Destroy(kvp.Value.gameObject);
			}
			_orbitMatchGraphics.Clear();

			ClearAllTimeLabels();

			_activeEntityColors.Clear();
		}

		private void ClearAllTimeLabels()
		{
			foreach (var kvp in _timeLabels)
			{
				if (kvp.Value != null)
				{
					foreach (var label in kvp.Value)
					{
						if (label != null) GameObject.Destroy(label.gameObject);
					}
				}
			}
			_timeLabels.Clear();
		}

		private Color GetHalfSaturationColor(Color baseColor)
		{
			Color.RGBToHSV(baseColor, out float h, out float s, out float v);
			s *= 0.5f;
			Color halvedColor = Color.HSVToRGB(h, s, v);
			halvedColor.a = baseColor.a;
			return halvedColor;
		}

		private void AddEntityTrajectory(Entity entity, Color initialColor)
		{
			if (entity == null || entity.IsDestroyed) return;
			
			// Add standard orbital trajectory
			if (!_trajectories.ContainsKey(entity))
			{
				var traj = LineGraphic.CreateLine(entity.Name, _lineWidth, initialColor, _lineMaterial, LineType.Trajectory, _container);
				_trajectories.Add(entity, traj);
			}

			// Add Sphere of Influence graphic for celestials with a parent primary
			if (entity.IsCelestial && entity.DominantEntity != null && !_soiGraphics.ContainsKey(entity))
			{
				var soiGraphic = LineGraphic.CreateLine($"SoI_{entity.Name}", _lineWidth, _soiLineColor, _lineMaterial, LineType.Trajectory, _container);
				_soiGraphics.Add(entity, soiGraphic);
			}

			// Initialize time-to labels if configured
			if (!_timeLabels.ContainsKey(entity) && TimeLabelsOffsets != null && TimeLabelsOffsets.Count > 0)
			{
				var labels = new List<TextMeshProUGUI>();
				Color labelColor = GetHalfSaturationColor(initialColor);

				for (int i = 0; i < TimeLabelsOffsets.Count; i++)
				{
					GameObject labelObj = new GameObject($"TimeLabel_{entity.Name}_{i}");
					labelObj.transform.SetParent(_container, false);

					RectTransform rect = labelObj.AddComponent<RectTransform>();
					rect.anchorMin = new Vector2(0.5f, 0.5f);
					rect.anchorMax = new Vector2(0.5f, 0.5f);
					rect.pivot = new Vector2(0.5f, 0.5f);

					TextMeshProUGUI tmp = labelObj.AddComponent<TextMeshProUGUI>();
					tmp.fontSize = 12;
					tmp.color = labelColor;
					tmp.raycastTarget = false;
					tmp.alignment = TextAlignmentOptions.Center;

					double tOffset = TimeLabelsOffsets[i];
					tmp.text = FormatTimeOffset(tOffset);

					labels.Add(tmp);
				}
				_timeLabels.Add(entity, labels);
			}
		}

		private string FormatTimeOffset(double seconds)
		{
			if (seconds < 60) return $"+{seconds:0}s";
			if (seconds < 3600) return $"+{seconds / 60:0}m";
			if (seconds < 86400) return $"+{seconds / 3600:0.0}h";
			return $"+{seconds / 86400:0.0}d";
		}

		private void RemoveEntityTrajectory(Entity entity)
		{
			if (entity == null) return;
			
			if (_trajectories.TryGetValue(entity, out var trajGraphic))
			{
				if (trajGraphic != null) GameObject.Destroy(trajGraphic.gameObject);
				_trajectories.Remove(entity);
			}

			if (_soiGraphics.TryGetValue(entity, out var soiGraphic))
			{
				if (soiGraphic != null) GameObject.Destroy(soiGraphic.gameObject);
				_soiGraphics.Remove(entity);
			}

			if (_navLines.TryGetValue(entity, out var navGraphic))
			{
				if (navGraphic != null) GameObject.Destroy(navGraphic.gameObject);
				_navLines.Remove(entity);
			}

			if (_orbitMatchGraphics.TryGetValue(entity, out var matchGraphic))
			{
				if (matchGraphic != null) GameObject.Destroy(matchGraphic.gameObject);
				_orbitMatchGraphics.Remove(entity);
			}

			if (_timeLabels.TryGetValue(entity, out var labels))
			{
				if (labels != null)
				{
					foreach (var label in labels)
					{
						if (label != null) GameObject.Destroy(label.gameObject);
					}
				}
				_timeLabels.Remove(entity);
			}
		}

		private double2[] ComputeTrajectory(Entity target)
		{
			List<double2> path = new List<double2>();
			if (target == null || target.IsDestroyed || target.DominantEntity == null) return null;

			PhysicsBody dominantBody = target.DominantEntity.PhysicsBody;
			double2 relPos = target.Position - dominantBody.Position;
			double2 relVel = target.Velocity - dominantBody.Velocity;

			if (OrbitalMath.TryFindSoIExitState(relPos, relVel, target.PhysicsBody.GravParam, dominantBody, out _, out _, out double exitTime))
			{
				return OrbitalMath.CalculateTwoBodyOrbit(relPos, relVel, dominantBody.Position, target.PhysicsBody.GravParam, exitTime, _orbitResolution);
			}
			else
			{
				return OrbitalMath.CalculateTwoBodyOrbit(relPos, relVel, dominantBody.Position, target.PhysicsBody.GravParam, _orbitResolution);
			}

			// // Generate orbit points centered around where the dominant body currently is at segment start time
			// double2[] orbitPoints = OrbitalMath.CalculateTwoBodyOrbit(relPos, relVel, dominantBody.Position, target.PhysicsBody.GravParam, _orbitResolution);

			// path.Add(orbitPoints[0]);
			// for (int i = 1; i < orbitPoints.Length; i++)
			// {
			// 	double2 pt = orbitPoints[i];
			// 	path.Add(pt);

			// 	if (OrbitalMath.HasExitedSoI(pt, dominantBody, dominantBody.Position)) break;
			// }

			// return path;
		}

		public void UpdateTrajectories()
		{
			_activeEntityColors.Clear();

			var controller = SimulationInteractionController.Instance;
			if (controller != null)
			{
				// Evaluate in reverse precedence so higher priorities overwrite lower priorities
				// Priority 4: Pinned
				foreach (var body in controller.PinnedMapObjects)
				{
					if (body is EntityMapObject entityGameObject)
					{
						if (entityGameObject.Entity != null && !entityGameObject.Entity.IsDestroyed)
						{
							_activeEntityColors[entityGameObject.Entity] = _pinnedColor;
						}
					}
				}

				// Priority 3: Targeted
				Entity targetedEnt = controller.TargetedEntity;
				if (targetedEnt != null && !targetedEnt.IsDestroyed)
				{
					_activeEntityColors[targetedEnt] = _targetedColor;
				}

				// Priority 2: Hovered
				Entity hoveredEnt = controller.HoveredEntity;
				if (hoveredEnt != null && !hoveredEnt.IsDestroyed)
				{
					_activeEntityColors[hoveredEnt] = _hoveredColor;
				}

				// Priority 1: Selected (Top priority across all selected map objects)
				foreach (var mapObj in controller.SelectedMapObjects)
				{
					if (mapObj is EntityMapObject ego && ego.Entity != null && !ego.Entity.IsDestroyed)
					{
						_activeEntityColors[ego.Entity] = _selectedColor;
					}
				}
			}

			// 1. Instantiate missing trajectories or synchronize changed colors
			foreach (var kvp in _activeEntityColors)
			{
				Entity entity = kvp.Key;
				Color targetColor = kvp.Value;
				Color targetLabelColor = GetHalfSaturationColor(targetColor);

				if (!_trajectories.TryGetValue(entity, out var trajGraphic) || trajGraphic == null)
				{
					AddEntityTrajectory(entity, targetColor);
				}
				else
				{
					if (trajGraphic.color != targetColor)
					{
						trajGraphic.color = targetColor;
					}
					if (_timeLabels.TryGetValue(entity, out var labels))
					{
						foreach (var label in labels)
						{
							if (label != null && label.color != targetLabelColor)
							{
								label.color = targetLabelColor;
							}
						}
					}
				}
			}

			// 2. Identify and destroy trajectories no longer present in interaction states
			_entitiesToRemove.Clear();
			foreach (var kvp in _trajectories)
			{
				Entity entity = kvp.Key;
				if (entity == null || entity.IsDestroyed || !_activeEntityColors.ContainsKey(entity))
				{
					_entitiesToRemove.Add(entity);
				}
			}

			for (int i = 0; i < _entitiesToRemove.Count; i++)
			{
				RemoveEntityTrajectory(_entitiesToRemove[i]);
			}

			// Track active navigation lines and orbit match graphics for selection/pin/hover states
			HashSet<Entity> activeNavEntities = new HashSet<Entity>();
			HashSet<Entity> activeMatchEntities = new HashSet<Entity>();
			
			// 1. Render Keplerian Orbits
			foreach (var kvp in _trajectories)
			{
				Entity target = kvp.Key;
				LineGraphic lineUI = kvp.Value;

				if (target == null || target.IsDestroyed || target.DominantEntity == null)
				{
					lineUI.SetPoints(null);
					if (_timeLabels.TryGetValue(target, out var labels))
					{
						foreach (var label in labels) if (label != null) label.gameObject.SetActive(false);
					}
					continue;
				}

				double2[] orbitPath = ComputeTrajectory(target);
				LineGraphic.RenderPathToGraphic(orbitPath, lineUI, _pathBuffer);

				// Render time-to position labels along the trajectory
				if (_timeLabels.TryGetValue(target, out var timeLabelsList) && TimeLabelsOffsets != null)
				{
					PhysicsBody dominantBody = target.DominantEntity.PhysicsBody;
					for (int i = 0; i < TimeLabelsOffsets.Count && i < timeLabelsList.Count; i++)
					{
						TextMeshProUGUI label = timeLabelsList[i];
						if (label == null) continue;

						double timeOffset = TimeLabelsOffsets[i];
						double mu = target.PhysicsBody.GravParam;
						
						OrbitalMath.GetLocalPosAtTime(target.PhysicsBody, dominantBody, timeOffset, mu, out double2 futurePos);
						Vector2 uiPos = SimulationRenderer.Instance.SimPositionToViewUI(futurePos + dominantBody.Position);
						label.rectTransform.anchoredPosition = uiPos;
						label.gameObject.SetActive(true);
					}
				}
				
				// Evaluate Navigation Capability Visualizations for Selected, Pinned, or Hovered entities
				bool isSelectedOrPinnedOrHovered = controller != null && controller.IsEntitySelectedPinnedOrHovered(target);

				if (isSelectedOrPinnedOrHovered && target.HasCapability<NavigationCapability>())
				{
					var navCap = target.GetCapability<NavigationCapability>();
					if (navCap != null && navCap.IsNavigating)
					{
						if (navCap.NavigationMode == NavigationMode.OrbitMatch)
						{
							// Orbit Match Mode: Render target orbit preview instead of a straight line
							activeMatchEntities.Add(target);
							if (!_orbitMatchGraphics.TryGetValue(target, out var matchGraphic) || matchGraphic == null)
							{
								matchGraphic = LineGraphic.CreateLine($"OrbitMatch_{target.Name}", _lineWidth, _orbitMatchColor, _lineMaterial, LineType.Trajectory, _container);
								_orbitMatchGraphics[target] = matchGraphic;
							}

							PhysicsBody parentSoI = target.PhysicsBody != null ? target.PhysicsBody.ParentSoI : null;
							if (parentSoI != null)
							{
								double2 tPos = navCap.TargetEntity != null && !navCap.TargetEntity.IsDestroyed ? navCap.TargetEntity.Position : parentSoI.Position + navCap.OrbitController.TargetOrbitPositionOffset;
								double2 tVel = navCap.TargetEntity != null && !navCap.TargetEntity.IsDestroyed ? navCap.TargetEntity.Velocity : navCap.OrbitController.TargetOrbitVelocity;
								
								double2[] matchPath = OrbitalMath.CalculateTwoBodyOrbit(tPos - parentSoI.Position, tVel - parentSoI.Velocity, parentSoI.Position, target.PhysicsBody.GravParam, _orbitResolution);
								LineGraphic.RenderPathToGraphic(matchPath, matchGraphic, _pathBuffer);
							}
							else
							{
								matchGraphic.SetPoints(null);
							}
						}
						else
						{
							// Standard Navigation Mode: Render predicted powered intercept path
							activeNavEntities.Add(target);
							List<double2> path = new List<double2>();

							if (navCap.NavigationMode == NavigationMode.Intercept)
							{
								path = InterceptController.PredictInterceptPath(target.Position, target.Velocity, navCap.TargetPosition, navCap.TargetVelocity, target.Mass, navCap.MaxForce, target.PhysicsBody.GravParam, navCap.TimeOfFlight, target.DominantEntity.PhysicsBody, out _, steps: 150);
							}

							if (path != null && path.Count > 1)
							{
								if (!_navLines.TryGetValue(target, out var navGraphic) || navGraphic == null)
								{
									navGraphic = LineGraphic.CreateLine($"NavPath_{target.Name}", _lineWidth, _navLineColor, _lineMaterial, LineType.Trajectory, _container);
									_navLines[target] = navGraphic;
								}

								LineGraphic.RenderPathToGraphic(path.ToArray(), navGraphic, _pathBuffer);
							}
						}
					}
				}
			}

			// Clean up inactive navigation lines
			_entitiesToRemove.Clear();
			foreach (var kvp in _navLines)
			{
				if (!activeNavEntities.Contains(kvp.Key)) _entitiesToRemove.Add(kvp.Key);
			}
			for (int i = 0; i < _entitiesToRemove.Count; i++)
			{
				if (_navLines.TryGetValue(_entitiesToRemove[i], out var g) && g != null) GameObject.Destroy(g.gameObject);
				_navLines.Remove(_entitiesToRemove[i]);
			}

			// Clean up inactive orbit match graphics
			_entitiesToRemove.Clear();
			foreach (var kvp in _orbitMatchGraphics)
			{
				if (!activeMatchEntities.Contains(kvp.Key)) _entitiesToRemove.Add(kvp.Key);
			}
			for (int i = 0; i < _entitiesToRemove.Count; i++)
			{
				if (_orbitMatchGraphics.TryGetValue(_entitiesToRemove[i], out var g) && g != null) GameObject.Destroy(g.gameObject);
				_orbitMatchGraphics.Remove(_entitiesToRemove[i]);
			}

			// 2. Render Spheres of Influence for Celestials
			foreach (var kvp in _soiGraphics)
			{
				Entity celestial = kvp.Key;
				LineGraphic soiUI = kvp.Value;

				if (celestial == null || celestial.IsDestroyed || celestial.DominantEntity == null)
				{
					soiUI.SetPoints(null);
					continue;
				}

				double2[] soiPath = OrbitalMath.CalculateSoIBoundary(celestial.PhysicsBody, _orbitResolution);
				LineGraphic.RenderPathToGraphic(soiPath, soiUI, _pathBuffer);
			}
		}
	}
}