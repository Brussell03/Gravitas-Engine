using System.Collections.Generic;
using Gravitas2D;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.UI;

namespace GravitasDemo.Rendering
{
	public enum LineType
	{
		Basic,
		Grid,
		Trajectory
	}

	/// <summary>
	/// Custom UI Graphic component to natively draw lines in the Canvas pipeline.
	/// Bypasses standard LineRenderer Z-culling issues.
	/// </summary>
	[RequireComponent(typeof(CanvasRenderer))]
	public class LineGraphic : MaskableGraphic
	{
		public float Thickness = 2.0f;
		private Vector2[] _points;

		public static LineGraphic CreateLine(string name, float lineWidth, Color lineColor, Material lineMaterial, LineType type, Transform parent)
		{
			GameObject lineObj = new GameObject($"Line_{name}");
			lineObj.transform.SetParent(parent, false);
			
			RectTransform rect = lineObj.AddComponent<RectTransform>();
			// Align anchors and pivot to center (0.5, 0.5) to match SimulationRenderer coordinate mappings perfectly
			rect.anchorMin = new Vector2(0.5f, 0.5f);
			rect.anchorMax = new Vector2(0.5f, 0.5f);
			rect.pivot = new Vector2(0.5f, 0.5f);
			rect.anchoredPosition = Vector2.zero;

			LineGraphic lineUI = lineObj.AddComponent<LineGraphic>();
			lineUI.raycastTarget = false; // Prevents interference with pan/zoom pointer events
			lineUI.color = lineColor;
			lineUI.Thickness = lineWidth;
			lineUI.material = lineMaterial;

			return lineUI;
		}

		public void SetMaterial(Material material)
		{
			this.material = material;
		}

		public void SetPoints(Vector2[] points)
		{
			if (points == null || points.Length == 0)
			{
				if (_points != null)
				{
					_points = null;
					SetVerticesDirty();
				}
				return;
			}

			if (_points == null || _points.Length != points.Length)
			{
				_points = new Vector2[points.Length];
			}
			
			System.Array.Copy(points, _points, points.Length);
			SetVerticesDirty();
		}

		public void RenderPathToGraphic(double2[] path, LineGraphic graphic, Vector2[] pathBuffer, Bounds2D bounds, float width, float height)
		{
			if (pathBuffer.Length != path.Length)
			{
				pathBuffer = new Vector2[path.Length];
			}

			for (int i = 0; i < path.Length; i++)
			{
				pathBuffer[i] = SimulationRenderer.Instance.SimPositionToViewUI(path[i]);
			}

			graphic.SetPoints(pathBuffer);
		}

		public void RenderPathToGraphic(List<double2> path, LineGraphic graphic, Vector2[] pathBuffer, Bounds2D bounds, float width, float height)
		{
			if (pathBuffer.Length != path.Count)
			{
				pathBuffer = new Vector2[path.Count];
			}

			for (int i = 0; i < path.Count; i++)
			{
				pathBuffer[i] = SimulationRenderer.Instance.SimPositionToViewUI(path[i]);
			}

			graphic.SetPoints(pathBuffer);
		}

		protected override void OnPopulateMesh(VertexHelper vh)
		{
			vh.Clear();

			if (_points == null || _points.Length < 2) return;

			UIVertex vertex = UIVertex.simpleVert;
			vertex.color = color;

			for (int i = 0; i < _points.Length - 1; i++)
			{
				Vector2 p1 = _points[i];
				Vector2 p2 = _points[i + 1];

				Vector2 dir = (p2 - p1).normalized;
				
				Vector2 perp = new Vector2(-dir.y, dir.x) * (Thickness * 0.5f);

				vertex.position = p1 - perp;
				vh.AddVert(vertex);
				
				vertex.position = p1 + perp;
				vh.AddVert(vertex);
				
				vertex.position = p2 + perp;
				vh.AddVert(vertex);
				
				vertex.position = p2 - perp;
				vh.AddVert(vertex);

				int offset = i * 4;
				vh.AddTriangle(offset, offset + 1, offset + 2);
				vh.AddTriangle(offset + 2, offset + 3, offset);
			}
		}
	}
}