using Unity.Mathematics;

namespace GravitasDemo
{
	public class WarheadCapability : IEntityCapability
	{
		public double WarheadEnergy { get; set; } // Joules

		public static WarheadCapability Missile => new WarheadCapability
		{
			WarheadEnergy = MissileEnergy
		};

		public static double MissileEnergy => 1.25e12;
	}
}