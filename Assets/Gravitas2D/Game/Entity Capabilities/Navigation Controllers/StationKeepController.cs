using Unity.Mathematics;

namespace GravitasDemo
{
	public class StationKeepController
	{
		public bool IsEnabled { get; private set; }
		protected NavigationCapability _manager;

		public StationKeepController(NavigationCapability manager)
		{
			_manager = manager;
		}

		public void Clear()
		{
			IsEnabled = false;
		}

		/// <summary>
		/// Computes the trajectory adjustment for relative position hold (station-keeping) relative to the target entity or position.
		/// </summary>
		public bool ComputeStationKeepingTrajectory(out float burnOrientation, out float burnThrottle)
		{
			Entity ownerEntity = _manager.OwnerEntity;
			double2 targetPos = _manager.TargetPosition;
			double2 targetVel = _manager.TargetVelocity;

			double mass = ownerEntity.Mass;
			double2 currentPos = ownerEntity.Position;
			double2 currentVel = ownerEntity.Velocity;

			burnOrientation = 0f;
			burnThrottle = 0f;

			if (mass <= 0.0) return false;

			if (double.IsNaN(targetPos.x) || double.IsNaN(targetPos.y)) return false;

			double2 R = targetPos - currentPos; // Relative position error
			double2 V = currentVel - targetVel; // Relative velocity error

			// PD controller for maintaining relative position and nulling relative velocity
			double2 requiredVelChange = R - (V * 2.0);

			double2 desiredAccel = (R * 0.2) - (V * 1.5);
			double desiredForce = math.length(desiredAccel) * ownerEntity.Mass;

			burnThrottle = (float)math.clamp(desiredForce / _manager.MaxForce, 0.0, 1.0);

			double len = math.length(requiredVelChange);
			if (len > 1e-5 && !double.IsNaN(len) && !double.IsInfinity(len))
			{
				double2 burnDir = requiredVelChange / len;
				if (!double.IsNaN(burnDir.x) && !double.IsNaN(burnDir.y))
				{
					burnOrientation = math.atan2((float)burnDir.y, (float)burnDir.x);
					return true;
				}
			}

			return false;
		}
	}
}