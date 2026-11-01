using UnityEngine;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine.Rendering;

namespace BLine
{
	public static class BLineManager
	{
		// --- Data Management ---
		private static List<BLine> lines = new List<BLine>();

		// --- Resources ---
		private static Material lineMaterial;
		private static ComputeBuffer pointsBuffer;
		private static ComputeBuffer colorsBuffer;
		private static ComputeBuffer lineDataBuffer;
		private static ComputeBuffer argsBuffer;

		[StructLayout(LayoutKind.Sequential)]
		private struct LineInstanceData
		{
			public uint startIndex;
			public uint pointCount;
		}
		private static uint[] args = new uint[5] { 0, 1, 0, 0, 0 }; // Adjust if using instancing

		static bool isInitialized = false;
		static bool isBuffersDirty = false;

		public static void Initialize()
		{
			if (isInitialized) return;
		
			// Load the material
			lineMaterial = Resources.Load<Material>("BLineMaterial");
			if (lineMaterial == null)
			{
				Debug.LogError("BLineRenderer: Failed to load line material!");
				return;
			}

			// Initialize args buffer
			argsBuffer?.Release();
			argsBuffer = new ComputeBuffer(1, args.Length * sizeof(uint), ComputeBufferType.IndirectArguments);

			isInitialized = true;
			//Debug.Log("BLineRenderer Initialized.");
		}

		public static void UpdateLineData()
		{
			if (!isInitialized) Initialize();

			if (lines == null || lines.Count == 0)
			{
				args[0] = 0; // Set vertex count to 0
				return;
			}

			uint maxPointsPerLine = 0;
			
			List<Vector3> allPoints = new List<Vector3>(lines.Count);
			List<Color> allColors = new List<Color>(lines.Count);
			List<LineInstanceData> allLineData = new List<LineInstanceData>();

			for (int i = 0; i < lines.Count; i++)
			{
				uint numPoints = (uint)lines[i].numPoints;

				allPoints.AddRange(lines[i].points);
				if (lines[i].loop)
				{
					allPoints.Add(lines[i].points[0]);
					numPoints++;
				}

				allLineData.Add(new LineInstanceData
				{
					startIndex = (uint)allPoints.Count - numPoints,
					pointCount = numPoints
				});

				allColors.Add(lines[i].color);

				if (numPoints > maxPointsPerLine)
				{
					maxPointsPerLine = numPoints;
				}
			}

			CreateStructuredBuffer(ref pointsBuffer, allPoints);
			CreateStructuredBuffer(ref colorsBuffer, allColors);
			CreateStructuredBuffer(ref lineDataBuffer, allLineData);

			// For a line strip, the number of vertices to draw IS the point count.
			args[0] = maxPointsPerLine;
			args[1] = (uint)allLineData.Count; // Number of lines to draw
			argsBuffer.SetData(args);

			lineMaterial.SetBuffer("_PointsBuffer", pointsBuffer);
			lineMaterial.SetBuffer("_ColorsBuffer", colorsBuffer);
			//lineMaterial.SetBuffer("_CentersBuffer", trajectoryCentersBuffer);
			lineMaterial.SetBuffer("_LineDataBuffer", lineDataBuffer);
		}

		public static void Render()
		{


			// Check if there's anything to draw
			if (!isInitialized)
				return;

			if (isBuffersDirty) UpdateLineData();

			if (args[0] < 2 || lineMaterial == null || pointsBuffer == null || argsBuffer == null)
				return;

			// Issue the draw call
			Graphics.DrawProceduralIndirect(
				lineMaterial,
				new Bounds(Vector3.zero, Vector3.one * 100000), // Large bounds
				MeshTopology.LineStrip,
				argsBuffer,
				0 // args buffer offset
				  // materialPropertyBlock: null // Can't easily use MPB without a Renderer
			);
		}
		
		public static void AddLine(BLine line)
		{
			if (!lines.Contains(line))
			{
				lines.Add(line);
			}

			MarkBuffersDirty();
		}

		public static void RemoveLine(BLine line)
		{
			lines.Remove(line);

			MarkBuffersDirty();
		}
		
		public static void MarkBuffersDirty()
		{
			isBuffersDirty = true;
		}

		// --- Cleanup ---
		public static void ReleaseBuffers()
		{
			pointsBuffer?.Release();
			pointsBuffer = null;
			colorsBuffer?.Release();
			colorsBuffer = null;
			lineDataBuffer?.Release();
			lineDataBuffer = null;
			argsBuffer?.Release();
			argsBuffer = null;

			isInitialized = false;

			//Debug.Log("BLineRenderer Buffers Released.");
		}

		private static void CreateStructuredBuffer<T>(ref ComputeBuffer buffer, List<T> data, ComputeBufferMode mode = ComputeBufferMode.Dynamic) where T : struct
		{
			CreateStructuredBuffer<T>(ref buffer, data.Count);
			buffer.SetData(data);
		}
		
		public static void CreateStructuredBuffer<T>(ref ComputeBuffer buffer, int count, ComputeBufferMode mode = ComputeBufferMode.Dynamic) {
			int stride = System.Runtime.InteropServices.Marshal.SizeOf(typeof(T));
			bool createNewBuffer = buffer == null || !buffer.IsValid() || buffer.count != count || buffer.stride != stride;
			if (createNewBuffer) {
				buffer?.Release();
				buffer = new ComputeBuffer(count, stride, ComputeBufferType.Structured, mode);
			}
		}
	}
}