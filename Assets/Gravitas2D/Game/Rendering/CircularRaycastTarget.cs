using UnityEngine;

namespace GravitasDemo.Rendering
{
	/// <summary>
	/// Circular interaction geometry shared by data-only entity map objects and the few
	/// GameObject-backed map objects. It is no longer a Graphic and is not registered with
	/// the Unity UI EventSystem; SimulationInteractionController invokes it directly.
	/// </summary>
	[DisallowMultipleComponent]
	public sealed class CircularRaycastTarget : MonoBehaviour
	{
		public bool Raycast(MapObject mapObject, Vector2 localUIPosition, float hoverMarginPixels)
		{
			if (!isActiveAndEnabled || !IsAlive(mapObject))
				return false;

			return IsHit(mapObject, localUIPosition, hoverMarginPixels);
		}

		public static bool IsAlive(MapObject mapObject)
		{
			if (ReferenceEquals(mapObject, null))
				return false;

			// Interface references do not use UnityEngine.Object's overloaded null operator.
			if (mapObject is Object unityObject && unityObject == null)
				return false;

			return mapObject.IsVisualActive && !mapObject.IsDestroyed;
		}

		public static bool IsHit(MapObject mapObject, Vector2 localUIPosition, float hoverMarginPixels)
		{
			if (!IsAlive(mapObject) || mapObject.PixelDiameter <= 0f)
				return false;

			// PixelDiameter is the visible diameter and deliberately excludes the interaction
			// margin. This reproduces the previous Graphic's effective radius calculation.
			float visualRadius = mapObject.PixelDiameter * 0.5f;

			// Celestial graphics are rendered at 2.6x physical radius while their selectable
			// physical radius uses the 2.0x scale, preserving the former hitbox correction.
			if (mapObject.IsCelestial)
				visualRadius *= (2.0f / 2.6f);

			float hitRadius = Mathf.Max(0f, visualRadius + Mathf.Max(0f, hoverMarginPixels));
			return (localUIPosition - mapObject.ViewPosition).sqrMagnitude <= hitRadius * hitRadius;
		}
	}
}