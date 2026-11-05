using Gravitas2D;
using UnityEngine;

namespace GravitasDemo.Rendering
{
	/// <summary>
	/// Shared logical contract for map objects. It is deliberately an interface rather than
	/// a MonoBehaviour base class so ordinary entities can be represented by managed data only.
	/// Entity groups and celestials implement the same contract from their small set of UI objects.
	/// </summary>
	public interface MapObject
	{
		Entity Entity { get; }
		RectTransform RectTransform { get; }
		bool IsDestroyed { get; }
		bool IsVisualActive { get; }
		bool IsCelestial { get; }
		string Name { get; }

		Vector2 ViewPosition { get; }
		float PixelDiameter { get; }
		float RenderRotationDegrees { get; }
		Sprite RenderSprite { get; }
		Color RenderColor { get; }
		Material RenderMaterial { get; }

		void UpdateVisual();
		void Deinitialize();
		void ApplyTransformData(
			float percX,
			float percY,
			float finalPixelSize,
			float rotationRadians,
			double boundsSizeX,
			float rectXMin,
			float rectXMax,
			float rectYMin,
			float rectYMax);
		bool Raycast(Vector2 localUIPosition, float hoverMarginPixels);
	}
}
