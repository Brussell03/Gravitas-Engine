using Unity.Mathematics;

namespace GravitasDemo
{
	public class RendezvousController
	{
		public bool IsEnabled { get; private set; }
		protected NavigationCapability _manager;

		public double2 TargetOffset { get; private set; }
		public float ArrivalThreshold { get; set; } = 10.0f; // Meters
		public float VelocityThreshold { get; private set; } = 0.25f; // m/s relative speed required for capture
		public bool SolutionFailed { get; private set; }

		private readonly RendezvousGuidanceSolver.PlanMemory _rendezvousPlanMemory = new RendezvousGuidanceSolver.PlanMemory();

		public RendezvousController(NavigationCapability manager)
		{
			_manager = manager;
		}

		public void Clear()
		{
			IsEnabled = false;
			TargetOffset = double2.zero;
			SolutionFailed = false;
			_rendezvousPlanMemory.Reset();
		}

		public bool CheckCompleted()
		{
			// Use the actual guidance aim point (including rendezvous clearance) and its
			// matching velocity. The previous check used the target centre for entity targets
			// and compared position against velocity for position targets.
			double distanceToTarget = math.distance(_manager.OwnerEntity.Position, _manager.TargetPosition);
			double relativeSpeed = math.distance(_manager.OwnerEntity.Velocity, _manager.TargetVelocity);

			return distanceToTarget <= ArrivalThreshold && relativeSpeed <= VelocityThreshold;
		}

		public double2 GetRendezvousOffset()
		{
			Entity ownerEntity = _manager.OwnerEntity;

			return GetRendezvousOffset(ownerEntity.Position, ownerEntity.Radius, _manager.TargetEntity, ownerEntity.InGroup, ownerEntity.Group, ownerEntity.Group?.IndexOfMember(ownerEntity) ?? -1, _manager.ClearanceDistance);
		}

		public static double2 GetRendezvousOffset(double2 ownerPos, double ownerRadius, Entity targetEntity = null, bool inGroup = false, EntityGroup ownerGroup = null, int ownerGroupIndex = -1, double clearanceDistance = 1000)
		{
			if (targetEntity == null) return double2.zero;

			double targetRadius = targetEntity.Radius;
			double targetClearance = targetEntity.IsCelestial ? targetRadius * 0.1 : clearanceDistance;

			double2 approachDirection;
			if (inGroup && ownerGroup.Leader != null)
				approachDirection = ownerGroup.Leader.Position - targetEntity.Position;
			else
				approachDirection = ownerPos - targetEntity.Position;

			approachDirection = math.lengthsq(approachDirection) > 1e-12
				? math.normalize(approachDirection)
				: new double2(1.0, 0.0);

			double approachBodyRadius = inGroup && ownerGroup.Leader != null
				? ownerGroup.Leader.Radius
				: ownerRadius;
			double separation = targetRadius + approachBodyRadius + targetClearance;
			double2 targetOffset = separation * approachDirection;

			if (inGroup)
				targetOffset += ownerGroup.GetFormationPosition(ownerGroupIndex);
			
			return targetOffset;
		}

		private static bool IsFinite(double value) => !double.IsNaN(value) && !double.IsInfinity(value);

		public bool ComputeRendezvousTrajectory(ref float timeOfFlight, out float burnOrientation, out float burnThrottle, float dt = 0.02f)
		{
			TargetOffset = GetRendezvousOffset();
			Entity ownerEntity = _manager.OwnerEntity;
			Entity targetEntity = _manager.TargetEntity;
			double2 targetPos = _manager.TargetPosition;
			double2 targetVel = _manager.TargetVelocity;

			burnThrottle = 0f;
			burnOrientation = 0f;
			SolutionFailed = false;
			if (ownerEntity == null || ownerEntity.IsDestroyed || ownerEntity.PhysicsBody == null || ownerEntity.Mass <= 0.0)
			{
				SolutionFailed = true;
				return false;
			}

			if (!IsFinite(targetPos.x) || !IsFinite(targetPos.y) ||
				!IsFinite(targetVel.x) || !IsFinite(targetVel.y) ||
				!IsFinite(TargetOffset.x) || !IsFinite(TargetOffset.y))
			{
				SolutionFailed = true;
				return false;
			}

			// Rendezvous is to the commanded standoff point, not the target body's center.
			double2 rendezvousPosition = targetPos + TargetOffset;
			double2 relativePosition = rendezvousPosition - ownerEntity.Position;
			double2 relativeVelocity = targetVel - ownerEntity.Velocity;
			double distance = math.length(relativePosition);
			double relativeSpeed = math.length(relativeVelocity);

			// Capture requires both position and velocity convergence. Reset the cached plan so
			// a subsequent navigation command cannot inherit the completed rendezvous solution.
			if (distance <= ArrivalThreshold && relativeSpeed <= VelocityThreshold)
			{
				timeOfFlight = 0f;
				_rendezvousPlanMemory.Reset();
				burnThrottle = 0f;
				burnOrientation = ownerEntity.Orientation;
				return false;
			}

			if (_manager.MaxForce <= 0f || ownerEntity.Mass <= 0.0)
			{
				SolutionFailed = true;
				return false;
			}

			// Convert scene entities to value-only solver inputs. TrySolve itself has no
			// dependency on Entity, so it can also be called for hypothetical vehicles and
			// free-space waypoints before a navigation capability is attached.
			var vehicleInput = new RendezvousGuidanceSolver.VehicleInput(
				ownerEntity.Position,
				ownerEntity.Velocity,
				ownerEntity.Mass,
				ownerEntity.PhysicsBody.Index);

			var targetInput = new RendezvousGuidanceSolver.TargetInput(
				targetPos,
				targetVel,
				TargetOffset,
				targetEntity != null,
				targetEntity?.IsCelestial ?? false,
				targetEntity?.PhysicsBody.Index ?? -1);

			if (!RendezvousGuidanceSolver.TrySolve(
				vehicleInput,
				targetInput,
				_manager.MaxForce,
				ArrivalThreshold,
				VelocityThreshold,
				dt,
				timeOfFlight,
				out double2 commandedAcceleration,
				out float solvedTimeOfFlight,
				planMemory: _rendezvousPlanMemory))
			{
				SolutionFailed = true;
				return false;
			}

			timeOfFlight = solvedTimeOfFlight;
			double requestedAcceleration = math.length(commandedAcceleration);
			double maximumAcceleration = _manager.MaxForce / ownerEntity.Mass;
			if (!IsFinite(requestedAcceleration) || requestedAcceleration <= 1e-10 || maximumAcceleration <= 0.0)
			{
				// A zero-thrust first interval is a valid MPC decision (coast before a later burn).
				// Keep the current attitude; the next navigation tick replans from the new state.
				burnOrientation = ownerEntity.Orientation;
				burnThrottle = 0f;
				return true;
			}

			burnOrientation = math.atan2((float)commandedAcceleration.y, (float)commandedAcceleration.x);
			burnThrottle = math.clamp((float)(requestedAcceleration / maximumAcceleration), 0f, 1f);
			return true;
		}
	}
}