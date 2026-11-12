using Unity.Mathematics;

namespace GravitasDemo
{
	public enum NavigationMode
	{
		Inactive,
		Attitude,
		Intercept,
		Rendezvous,
		OrbitMatch,
		StationKeep
	}

	public class NavigationCapability : IEntityCapability
	{
		public Entity OwnerEntity { get; set; }
		public Entity TargetEntity { get; set; }
		public double2 TargetPosition { get; set; }
		public double2 TargetVelocity { get; set; }
		public float TargetOrientation { get; set; } = 0f; // Radians
		public float MaxTorque { get; set; } = 1000f; // Newton-meters
		public float MaxForce { get; set; } = 10000f; // Newtons
		public NavigationMode NavigationMode { get; private set; } = NavigationMode.Inactive;
		public float BurnOrientation { get; private set; }
		/// <summary>Requested thrust fraction for guidance, in [0, 1].</summary>
		public float BurnThrottle { get; private set; } = 1f;

		private AttitudeController _attitudeController;
		private InterceptController _interceptController;
		private RendezvousController _rendezvousController;
		private OrbitMatchController _orbitMatchController;
		private StationKeepController _stationKeepController;

		public bool RequireAttitudeAlignmentForThrust { get; set; } = true;

		// Rendezvous Clearance
		public float ClearanceDistance { get; set; } = 1000;

		private float _timeOfFlight;
		public float TimeOfFlight => _timeOfFlight;
		public bool IsNavigating => NavigationMode != NavigationMode.Inactive;
		public OrbitMatchController OrbitController => _orbitMatchController;

		public static NavigationCapability Missile => new NavigationCapability
		{
			MaxForce = MaxMissileForce, // 50g acceleration
			MaxTorque = 10000000
		};

		public static float MaxMissileForce => (float)2.5797e8; // 50g acceleration

		public void Initialize()
		{
			_attitudeController = new AttitudeController(this);
			_interceptController = new InterceptController(this);
			_rendezvousController = new RendezvousController(this);
			_orbitMatchController = new OrbitMatchController(this);
			_stationKeepController = new StationKeepController(this);
		}

		public void EndNavigation()
		{
			_timeOfFlight = -1f;
			TargetEntity = null;
			TargetPosition = double2.zero;
			TargetVelocity = double2.zero;
			TargetOrientation = OwnerEntity.Orientation;
			NavigationMode = NavigationMode.Inactive;

			_attitudeController.Clear();
			_interceptController.Clear();
			_rendezvousController.Clear();
			_orbitMatchController.Clear();
			_stationKeepController.Clear();
		}

		public void EnterIntercept(float arrivalThreshold, Entity targetEntity = null, double2 targetPos = default)
		{
			EndNavigation();

			_interceptController.ArrivalThreshold = arrivalThreshold;

			if (targetEntity != null)
				TargetEntity = targetEntity;
			else
				TargetPosition = targetPos;

			NavigationMode = NavigationMode.Intercept;
		}

		public void EnterRendezvous(float arrivalThreshold, Entity targetEntity = null, double2 targetPos = default)
		{
			EndNavigation();

			_rendezvousController.ArrivalThreshold = arrivalThreshold;

			if (targetEntity != null)
				TargetEntity = targetEntity;
			else
				TargetPosition = targetPos;

			NavigationMode = NavigationMode.Rendezvous;
		}

		public void EnterOrbitMatch(Entity targetEntity = null, double2 targetPosOffset = default, double2 targetVel = default)
		{
			EndNavigation();

			if (targetEntity != null)
				TargetEntity = targetEntity;

			NavigationMode = NavigationMode.OrbitMatch;

			_orbitMatchController.TargetOrbitPositionOffset = targetPosOffset;
			_orbitMatchController.TargetOrbitVelocity = targetVel;
		}

		public void EnterStationKeep(Entity targetEntity = null, double2 targetPos = default)
		{
			EndNavigation();

			if (targetEntity != null)
				TargetEntity = targetEntity;
			else
				TargetPosition = targetPos;

			NavigationMode = NavigationMode.StationKeep;
		}

		/// <summary>Returns true when the force direction for the current physics step is aligned.</summary>
		public bool IsBurnAligned(float deltaTime) => _attitudeController.IsBurnAligned(deltaTime);

		public void SetTargetOrientation(float targetOrientation)
		{
			_attitudeController.SetTargetOrientation(targetOrientation);
			NavigationMode = NavigationMode.Attitude;
		}

		/// <summary>
		/// Updates target position and velocity.
		/// </summary>
		private void UpdateTargetState()
		{
			if (TargetEntity == null || TargetEntity.IsDestroyed) return;

			TargetPosition = TargetEntity.Position;
			TargetVelocity = TargetEntity.Velocity;
		}

		public void UpdateAttitude(float targetOrientation, float dt)
		{
			if (NavigationMode == NavigationMode.Inactive) return;

			dt = _attitudeController.PrepareAttitudeControl(dt);
			_attitudeController.UpdateTargetOrientation(targetOrientation, dt);
			float analogTorque = _attitudeController.EvaluateAnalogTorque(dt);
			if (analogTorque != 0f)
			{
				OwnerEntity.PhysicsBody.AddTorque(analogTorque);
			}
		}

		public bool UpdateIntercept()
		{
			if (NavigationMode != NavigationMode.Intercept) return false;
			
			UpdateTargetState();

			if (!_interceptController.ComputeInterceptTrajectory(out float timeOfFlight, out float burnOrientation))
			{
				return false;
			}

			_timeOfFlight = timeOfFlight;
			BurnOrientation = burnOrientation;
			BurnThrottle = 1;
			return true;
		}

		public bool UpdateRendezvous(float dt)
		{
			if (NavigationMode != NavigationMode.Rendezvous) return false;
			
			UpdateTargetState();

			if (!_rendezvousController.ComputeRendezvousTrajectory(ref _timeOfFlight, out float burnOrientation, out float burnThrottle, dt))
			{
				return false;
			}

			BurnOrientation = burnOrientation;
			BurnThrottle = burnThrottle;
			return true;
		}

		public bool UpdateOrbitMatch(float dt)
		{
			if (NavigationMode != NavigationMode.OrbitMatch) return false;

			if (!_orbitMatchController.ComputeOrbitMatchTrajectory(dt, out float burnOrientation, out float burnThrottle))
			{
				return false;
			}

			BurnOrientation = burnOrientation;
			BurnThrottle = burnThrottle;
			return true;
		}

		public bool UpdateStationKeep()
		{
			if (NavigationMode != NavigationMode.StationKeep) return false;
			
			UpdateTargetState();

			if (!_stationKeepController.ComputeStationKeepingTrajectory(out float burnOrientation, out float burnThrottle))
			{
				return false;
			}

			BurnOrientation = burnOrientation;
			BurnThrottle = burnThrottle;
			return true;
		}

		public bool CheckCompleted(float dt)
		{
			switch (NavigationMode)
			{
				case NavigationMode.Attitude:
					return _attitudeController.CheckCompleted(dt);
				case NavigationMode.Intercept:
					return _interceptController.CheckCompleted();
				case NavigationMode.Rendezvous:
					return _rendezvousController.CheckCompleted();
				case NavigationMode.OrbitMatch:
					return _orbitMatchController.CheckCompleted();
				case NavigationMode.StationKeep:
					return false; // Station keeping is indefinite, lasting until externally cancelled
				default:
					return true;
			}
		}
	}
}