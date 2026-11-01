using UnityEngine;

namespace Gravitas2D
{
	[CreateAssetMenu(fileName = "NewFractureProfile", menuName = "Gravitas/Fracture Profile")]
	public class FractureProfile : ScriptableObject
	{
		public string ProfileName;

		[Tooltip("Minimum radius of a body for this profile to apply.")]
		public float MinRadiusThreshold;

		[Tooltip("Maximum radius of a body for this profile to apply.")]
		public float MaxRadiusThreshold;

		[Tooltip("Minimum radius of a debris chunk as a percentage of the original radius.")]
		[Min(0.00000001f)]
		public float MinChunkRadiusPerc;
		
		[Tooltip("Maximum possible radius of a chunk as a percentage of the original radius.")]
		[Min(0.00000001f)]
		public float MaxChunkRadiusPerc;
		
		[Tooltip("1.0 = Uniform size. 4.0+ = Logarithmic/Power-law (millions of small dust particles, few large cores).")]
		public float DistributionExponent;
		
		[Tooltip("Base outward expansion velocity of the explosion.")]
		public float EjectionEnergy;

		[Tooltip("Fractional variation in ejection magnitude (e.g., 0.3 = ±30% speed variation).")]
		public float EjectionEnergyVariation;

		[Tooltip("Maximum angular deflection from strictly radial ejection in radians (e.g., 0.35 = ~20 degrees).")]
		public float DirectionScatter;
		
		[Tooltip("Multiplier for how violently the chunks spin.")]
		public float SpinEnergy;
	}
}