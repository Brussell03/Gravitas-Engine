using Gravitas2D;

namespace GravitasDemo
{
	public class StationEntity : Entity
	{
		public StationEntity(PhysicsBody body, string name, bool isOwned = false) : base(body, name, EntityType.Station, isOwned)
		{
			
		}
	}
}