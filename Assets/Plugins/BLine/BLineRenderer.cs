using UnityEngine;
using UnityEngine.Rendering;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace BLine
{
	internal static class BLineRenderer
	{
		// Initialization called automatically when the game loads
		[RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
		private static void Initialize()
		{
			BLineManager.Initialize();

			// Subscribe to the render event
            RenderPipelineManager.beginCameraRendering -= OnBeginCameraRendering; // Prevent double registration
            RenderPipelineManager.beginCameraRendering += OnBeginCameraRendering;

			// Ensure buffers are released when the application quits
			Application.quitting -= ReleaseBuffers;
			Application.quitting += ReleaseBuffers;
		}

		// Callback after camera rendering
		private static void OnBeginCameraRendering(ScriptableRenderContext context, Camera camera)
		{
			// Optional: Filter which cameras trigger the drawing
			// if (cam != Camera.main) return;
			// if (cam.cameraType != CameraType.Game) return;

			BLineManager.Render();

			// CommandBuffer cmd = CommandBufferPool.Get("BLineRender"); // Get a temporary CB
            // BLineManager.Render(cmd); // Pass CB to the manager's render function
            // context.ExecuteCommandBuffer(cmd); // Execute the commands
            // CommandBufferPool.Release(cmd);
		}

		// --- Cleanup ---
		private static void ReleaseBuffers()
		{
			BLineManager.ReleaseBuffers();

			// Unsubscribe from events
			RenderPipelineManager.endCameraRendering -= OnBeginCameraRendering;


			Application.quitting -= ReleaseBuffers;
		}
	}
}