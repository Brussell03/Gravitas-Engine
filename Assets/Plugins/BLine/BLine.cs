using UnityEngine;

namespace BLine
{
	public class BLine
	{
		public Vector3[] points { get; private set; }
		public Color color { get; private set; }
		public bool loop; 

		public BLine(Vector3[] points, Color color, bool loop = false)
		{
			this.points = points;
			this.color = color;
			this.loop = loop;
			Draw();
		}

		public void SetPoints(Vector3[] points)
		{
			this.points = points;
			BLineManager.MarkBuffersDirty();
		}

		public void SetColor(Color color)
		{
			this.color = color;
			BLineManager.MarkBuffersDirty();
		}

		public float GetLength()
		{
			if (points == null || points.Length < 2) return 0f;
            float length = 0f;
            for (int j = 0; j < points.Length - 1; j++)
            {
                length += Vector3.Distance(points[j], points[j + 1]);
            }
            if (loop && points.Length >= 2) // Add closing segment if looping
            {
                length += Vector3.Distance(points[points.Length - 1], points[0]);
            }
            return length;
		}

		public int numPoints => points?.Length ?? 0;

		public void Draw()
		{
			BLineManager.AddLine(this);
		}

		public void Remove()
		{
			BLineManager.RemoveLine(this);
		}

		public static BLine Circle(int numPoints, float radius, Color color)
		{
			Vector3[] points = new Vector3[numPoints];

			for (int i = 0; i < numPoints; i++)
			{
				// Calculate the true anomaly for this point
				float trueAnomaly = Mathf.Lerp(0, Mathf.PI * 2, (float)i / (numPoints - 1));

				// Convert polar coordinates to Cartesian coordinates in the orbital plane
				Vector3 orbitalPlanePos = new Vector3(
					radius * Mathf.Cos(trueAnomaly),
					radius * Mathf.Sin(trueAnomaly),
					0
				);

				// Rotate the point into the correct 3D space
				points[i] = orbitalPlanePos;
			}

			return new BLine(points, color, true);
		}
	}
}