using UnityEngine;

namespace Gravitas2D
{
	[CreateAssetMenu(fileName = "NewCompositionProfile", menuName = "Gravitas/Composition Profile")]
	public class CompositionProfile : ScriptableObject
	{
		public string Name;

		[Tooltip("Kilograms per cubic meter.")]
		public float Density = 1000;

		[Tooltip("Instantaneous change in speed required for the body to be destroyed or fractured.")]
		public float BreakSpeedThreshold = 45;
	}
}