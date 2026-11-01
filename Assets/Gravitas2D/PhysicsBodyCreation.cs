using Unity.Mathematics;
using UnityEngine;

namespace Gravitas2D
{
	/// <summary>
	/// Encapsulates optional spawn parameters.
	/// </summary>
	public struct BodySpawnOptions
	{
		public double Delay;
		public bool IsCelestial;
		public bool IsCollisionTrigger;
		public bool IsCollidable;
		public PhysicsBody ReferenceBody;

		public static BodySpawnOptions Default => new BodySpawnOptions
		{
			Delay = 0f,
			IsCelestial = false,
			IsCollisionTrigger = true,
			IsCollidable = true,
			ReferenceBody = null
		};

		public static BodySpawnOptions Delayed(double delayTime, PhysicsBody referenceBody = null) => new BodySpawnOptions
		{
			Delay = delayTime,
			IsCelestial = false,
			IsCollisionTrigger = true,
			IsCollidable = true,
			ReferenceBody = referenceBody
		};

		public static BodySpawnOptions Celestial => new BodySpawnOptions
		{
			Delay = 0f,
			IsCelestial = true,
			IsCollisionTrigger = true,
			IsCollidable = true,
			ReferenceBody = null
		};
	}

	public static class PhysicsBodyCreation
	{
		/// <summary>
		/// Standardizes state validation to prevent redundant initialization and boundary checks.
		/// </summary>
		private static bool ValidateCreation(double2 absolutePosition)
		{
			if (PhysicsManager2D.Instance == null)
			{
				Debug.LogError($"[{nameof(PhysicsBodyCreation)}] PhysicsManager2D is uninitialized.");
				return false;
			}

			if (!PhysicsManager2D.Instance.SimulationBounds.IsWithinBounds(absolutePosition))
			{
				Debug.LogError($"[{nameof(PhysicsBodyCreation)}] Initial position is out of simulation bounds.");
				return false;
			}

			return true;
		}

		/// <summary>
		/// Injects a fully defined body into the simulation using absolute world coordinates.
		/// </summary>
		public static bool CreateBody(string name, VehicleModel model, double2 absolutePosition, double2 absoluteVelocity, float orientation, float angularVelocity, int compositionIndex, double warheadEnergy, out PhysicsBody physicsBody, BodySpawnOptions options = default)
		{
			if (!ValidateCreation(absolutePosition))
			{
				physicsBody = null;
				return false;
			}

			StateVector state = new StateVector(absolutePosition, absoluteVelocity, orientation, angularVelocity);
			physicsBody = PhysicsManager2D.Instance.AddBody(state, model, compositionIndex, options.IsCelestial, name, options.IsCollisionTrigger, options.IsCollidable, warheadEnergy, options.Delay, options.ReferenceBody);

			return true;
		}

		/// <summary>
		/// Injects a body into the simulation using velocity relative to its dominant gravitational parent.
		/// </summary>
		public static bool CreateBodyRelative(string name, VehicleModel model, double2 absolutePosition, double2 relativeVelocity, float orientation, float angularVelocity, int compositionIndex, double warheadEnergy, out PhysicsBody physicsBody, BodySpawnOptions options = default)
		{
			if (!ValidateCreation(absolutePosition))
			{
				physicsBody = null;
				return false;
			}

			PhysicsBody dominantParent = OrbitalMath.GetDominantSoI(absolutePosition);

			// Frame transformation: Shift relative orbital velocity to absolute simulation velocity
			double2 absoluteVelocity = relativeVelocity;
			if (dominantParent != null)
			{
				absoluteVelocity += dominantParent.Velocity;
			}
			else
			{
				Debug.LogWarning($"[{nameof(PhysicsBodyCreation)}] No dominant parent found at position. Falling back to absolute velocity.");
			}

			StateVector state = new StateVector(absolutePosition, absoluteVelocity, orientation, angularVelocity);
			physicsBody = PhysicsManager2D.Instance.AddBody(state, model, compositionIndex, options.IsCelestial, name, options.IsCollisionTrigger, options.IsCollidable, warheadEnergy, options.Delay, options.ReferenceBody);

			return true;
		}

		/// <summary>
		/// Solves Lambert's problem to spawn a body that will precisely intersect a target coordinate after a given time of flight.
		/// </summary>
		public static bool CreateTargetedBody(string name, VehicleModel model, double2 initialPos, double2 targetPos, double timeToTarget, float orientation, float angularVelocity, int compositionIndex, double warheadEnergy, out PhysicsBody physicsBody, bool shortWay = true, BodySpawnOptions options = default)
		{
			if (timeToTarget <= 0.0)
			{
				Debug.LogError($"[{nameof(PhysicsBodyCreation)}] Time to target must be greater than zero.");
				physicsBody = null;
				return false;
			}

			if (!ValidateCreation(initialPos))
			{
				physicsBody = null;
				return false;
			}

			PhysicsBody dominantParent = OrbitalMath.GetDominantSoI(initialPos);
			
			if (dominantParent == null)
			{
				Debug.LogError($"[{nameof(PhysicsBodyCreation)}] No dominant gravity source found to define the orbital reference frame.");
				physicsBody = null;
				return false;
			}

			// Shift coordinates into the local frame of the dominant gravitational body
			double2 relPos1 = initialPos - dominantParent.Position;
			double2 relPos2 = targetPos - dominantParent.Position;
			double mu = OrbitalMath.GetStandardGravitationalParameter(dominantParent.Mass, model.Mass);

			if (!OrbitalMath.SolveVelocityToHitTarget(relPos1, relPos2, timeToTarget, mu, out double2 relInitialVelocity, shortWay))
			{
				Debug.LogError($"[{nameof(PhysicsBodyCreation)}] Lambert solver failed to converge for the given boundary conditions.");
				physicsBody = null;
				return false;
			}

			// Transform resolved relative velocity back to world space
			double2 absoluteVelocity = relInitialVelocity + dominantParent.Velocity;

			StateVector newState = new StateVector(initialPos, absoluteVelocity, orientation, angularVelocity);
			physicsBody = PhysicsManager2D.Instance.AddBody(newState, model, compositionIndex, options.IsCelestial, name, options.IsCollisionTrigger, options.IsCollidable, warheadEnergy, options.Delay, options.ReferenceBody);
			
			return true;
		}
	}
}