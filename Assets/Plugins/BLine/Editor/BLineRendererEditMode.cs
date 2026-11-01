#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;
using BLine;

namespace BLine
{
    // Makes the static constructor run when the editor loads
    [InitializeOnLoad]
    internal static class BLineRendererEditMode
    {
        // Static constructor
        static BLineRendererEditMode()
        {
            // Subscribe to Scene view drawing and editor quitting events
            // Use += instead of = to avoid replacing existing delegates
            SceneView.duringSceneGui -= OnSceneGUI; // Prevent double registration on reloads
            SceneView.duringSceneGui += OnSceneGUI;

            EditorApplication.playModeStateChanged -= HandlePlayModeStateChange;
            EditorApplication.playModeStateChanged += HandlePlayModeStateChange;

            AssemblyReloadEvents.beforeAssemblyReload -= OnBeforeAssemblyReload;
            AssemblyReloadEvents.beforeAssemblyReload += OnBeforeAssemblyReload;

            EditorApplication.quitting -= OnEditorQuitting;
            EditorApplication.quitting += OnEditorQuitting;

            BLineManager.Initialize();
        }

        // --- Drawing Callback for Scene View ---
        private static void OnSceneGUI(SceneView sceneView)
        {
            if (!EditorApplication.isPlayingOrWillChangePlaymode)
            {
                BLineManager.Render();
            }
        }

        // Called just BEFORE the C# domain is unloaded during script recompiles
        private static void OnBeforeAssemblyReload()
        {
            //Debug.Log("BLineRendererEditMode: Before Assembly Reload - Releasing Buffers...");
            ReleaseAndUnsubscribe(); // Release GPU resources *before* domain unload
        }

        // Called when Play Mode state changes
        private static void HandlePlayModeStateChange(PlayModeStateChange state)
        {
            // Cleanup when EXETING Play Mode
            // if (state == PlayModeStateChange.ExitingPlayMode)
            // {
            //     Debug.Log("BLineRendererEditMode: Exiting Play Mode - Releasing Buffers...");
            //     ReleaseAndUnsubscribe(); // Release buffers used in edit mode
            //     // Re-initialize after a short delay for edit mode next frame
            //     EditorApplication.delayCall += InitializeResourcesIfNeeded;
            // }
        }

        // --- Cleanup on Editor Quit ---
        private static void OnEditorQuitting() // Renamed from Cleanup
        {
            //Debug.Log("BLineRendererEditMode: Editor Quitting - Releasing Buffers...");
            ReleaseAndUnsubscribe();
        }

        // Centralized release and unsubscribe logic
        private static void ReleaseAndUnsubscribe()
        {
            BLineManager.ReleaseBuffers();

            SceneView.duringSceneGui -= OnSceneGUI;

            //Debug.Log("BLineRendererEditMode: Buffers Released.");
        }

        // --- Cleanup ---
        // private static void Cleanup()
        // {
        //     Debug.Log("Cleaning up from edit mode");
        //     BLineManager.ReleaseBuffers();

        //     // Unsubscribe
        //     SceneView.duringSceneGui -= OnSceneGUI;
        //     EditorApplication.quitting -= Cleanup;
        // }
    }
}
#endif