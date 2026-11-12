namespace GravitasDemo
{
	public class RailgunCapability : IEntityCapability
	{
		public int ProjectileCompositionIndex { get; set; }
		public float ProjectileRadius { get; set; }
		public float FireSpeed { get; set; }
		public float ReloadTime { get; set; }
		public int Ammo { get; set; }
	}
}