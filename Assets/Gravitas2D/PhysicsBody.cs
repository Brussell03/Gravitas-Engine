using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace Gravitas2D
{
	public class PhysicsBody
	{
		private string _name;
		private bool _isCelestial;
		private int _bodyIndex;
		private StateVector _currentStateVector;
		private VehicleModel _vehicleModel;
		private int _compositionIndex;
		private PhysicsBody _parentSoI;
		private HashSet<PhysicsBody> _childCelestials = new HashSet<PhysicsBody>();
		private bool _isTracked = false;
		private bool _isFracturable = false;
		private double _gravParam;

		private static int NUM_BODIES_ADDED = 0;

		// Deferred execution queue for state mutations
		private Queue<Action> _pendingOperations = new Queue<Action>();

		public string Name => _name;
		public double Radius => _vehicleModel.Radius;
		public bool IsCelestial => _isCelestial;
		public double Mass => _vehicleModel.Mass;
		public double Inertia => _vehicleModel.InertiaTensor;
		public int Index => _bodyIndex;
		public double GravParam => _gravParam;
		public double2 Position => _currentStateVector.Position;
		public double2 Velocity => _currentStateVector.Velocity;
		public float Orientation => _currentStateVector.Orientation;
		public float AngularVelocity => _currentStateVector.AngularVelocity;
		public bool IsDestroyed => _bodyIndex <= -1;
		public bool IsPending => _bodyIndex < 0 && _bodyIndex != -1;
		public CompositionProfile Composition => PhysicsManager2D.Instance.CompositionProfiles[_compositionIndex];
		public int CompositionIndex => _compositionIndex;
		public PhysicsBody ParentSoI => _parentSoI;
		public HashSet<PhysicsBody> ChildCelestials => _childCelestials;
		public bool HasPendingOperations => _pendingOperations.Count > 0;
		public bool IsFracturable => _isFracturable;
		public bool IsTracked => _isTracked;
		public VehicleModel VehicleModel => _vehicleModel;

		public event Action<double2> OnTrackedPositionUpdated;
		public event Action<PhysicsBody> OnSoIChanged;
		public event Action OnDestroyed;

		public PhysicsBody(int bodyIndex, StateVector stateVector, VehicleModel vehicleModel, int compositionIndex, string name, bool isCelestial = false)
		{
			_name = name == null ? $"physicsBody_{NUM_BODIES_ADDED++}" : name;
			_isCelestial = isCelestial;
			_bodyIndex = bodyIndex;
			_currentStateVector = stateVector;
			_vehicleModel = vehicleModel;
			_compositionIndex = compositionIndex;

			_isFracturable = PhysicsManager2D.Instance.IsBodyFracturable(this);
		}

		public void Remove()
		{
			PhysicsManager2D.Instance.RemoveBody(_bodyIndex);
		}

		public void PostRemove()
		{
			if (_isCelestial) _parentSoI?.DeregisterCelestialAsChild(this);
			_bodyIndex = -1;
			_isTracked = false;
			OnDestroyed?.Invoke();
		}

		public void DecrementIndex() => _bodyIndex--;

		internal void SetIndex(int index) => _bodyIndex = index;

		public void UpdateSoI()
		{
			// 1. Full SoI determination if prior SoI isn't already known
			if (_parentSoI == null || _parentSoI.IsDestroyed)
			{
				var soi = OrbitalMath.GetDominantSoI(this);
				if (soi != _parentSoI)
				{
					_parentSoI = soi;
					_gravParam = OrbitalMath.GetStandardGravitationalParameter(_parentSoI?.Mass ?? 0, _vehicleModel.Mass);
					OnSoIChanged?.Invoke(_parentSoI);
				}
				if (_isCelestial && _parentSoI != null) _parentSoI.RegisterCelestialAsChild(this);
				return;
			}
			
			// 2. Check if the body has entered any sub spheres of influence (evaluate entry before exit)
			PhysicsBody subSoI = OrbitalMath.CheckEnteredSubSoI(this, _parentSoI);
			if (subSoI != null)
			{
				if (_isCelestial) _parentSoI.DeregisterCelestialAsChild(this);
				_parentSoI = subSoI;
				_gravParam = OrbitalMath.GetStandardGravitationalParameter(_parentSoI.Mass, _vehicleModel.Mass);
				OnSoIChanged?.Invoke(_parentSoI);
				if (_isCelestial) _parentSoI.RegisterCelestialAsChild(this);
				return;
			}

			// 3. Check if the body has left the current sphere of influence
			if (OrbitalMath.HasExitedSoI(this, _parentSoI))
			{
				if (_isCelestial) _parentSoI.DeregisterCelestialAsChild(this);
				
				// Fall back to the parent's parent upon exiting the local SoI
				_parentSoI = _parentSoI.ParentSoI;
				_gravParam = OrbitalMath.GetStandardGravitationalParameter(_parentSoI.Mass, _vehicleModel.Mass);
				OnSoIChanged?.Invoke(_parentSoI);
				
				if (_isCelestial && _parentSoI != null) _parentSoI.RegisterCelestialAsChild(this);
			}
		}

		public bool RegisterCelestialAsChild(PhysicsBody childCelestial)
		{
			if (_childCelestials.Add(childCelestial))
			{
				return true;
			}
			return false;
		}

		public bool DeregisterCelestialAsChild(PhysicsBody childCelestial)
		{
			if (_childCelestials.Remove(childCelestial))
			{
				return true;
			}
			return false;
		}

		public void SyncStateFromManager(in StateVector stateVector)
		{
			if (_isTracked)
			{
				OnTrackedPositionUpdated?.Invoke(stateVector.Position - _currentStateVector.Position);
			}

			_currentStateVector = stateVector;
			// _vehicleModel = data.VehicleModel;

			FlushPendingOperations();

			UpdateSoI();
		}

		/// <summary>
		/// Sets the mass of the physics body directly without overwriting external forces, moments, or velocity vectors.
		/// </summary>
		public void SetMass(double newMass)
		{
			if (newMass <= 0.0) throw new ArgumentOutOfRangeException(nameof(newMass), "Mass must be strictly greater than zero.");

			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetVehicleModel(_bodyIndex, out VehicleModel model))
				{
					// Recompute vehicle model properties with the new mass
					VehicleModel updatedModel = new VehicleModel(newMass, model.Radius);
					_vehicleModel = updatedModel;

					PhysicsManager2D.Instance.SetBodyVehicleModel(_bodyIndex, updatedModel);
				}
			});
		}

		public void SetTracked(bool isTracked) => _isTracked = isTracked;

		/// <summary>
		/// Empties and evaluates the pending operations queue. Crucial for flushing forces 
		/// applied to bodies that skipped the physics integration tick.
		/// </summary>
		public void FlushPendingOperations()
		{
			while (_pendingOperations.Count > 0)
			{
				_pendingOperations.Dequeue().Invoke();
			}
		}

		public void Fracture()
		{
			PhysicsManager2D.Instance.FractureBody(_bodyIndex);
		}

		/// <summary>
		/// Returns the sphere of influence radius for this body.
		/// </summary>
		public double GetSoIRadius()
		{
			if (_parentSoI == null) return double.MaxValue;

			return OrbitalMath.CalculateSoIRadius(this);
		}

		private void ExecuteOrDefer(Action operation)
		{
			if (PhysicsManager2D.Instance.IsPhysicsStepRunning || IsPending)
			{
				_pendingOperations.Enqueue(operation);
			}
			else
			{
				operation.Invoke();
			}
		}

		#region Forcing and Torquing
		/// <summary>
		/// Applies an instantaneous acceleration (defined in world-space).
		/// </summary>
		public void AddWorldAcceleration(float2 worldAcceleration)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetBodyForce(_bodyIndex, out double2 currentForce))
				{
					double2 worldForce = Mass * (double2)worldAcceleration;
					PhysicsManager2D.Instance.SetBodyForce(_bodyIndex, currentForce + worldForce);
				}
			});
		}

		/// <summary>
		/// Applies an instantaneous acceleration (defined in body-space).
		/// </summary>
		public void AddRelativeAcceleration(float2 relativeAcceleration)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetStateVector(_bodyIndex, out StateVector state) &&
					PhysicsManager2D.Instance.TryGetVehicleModel(_bodyIndex, out VehicleModel model) &&
					PhysicsManager2D.Instance.TryGetBodyForce(_bodyIndex, out double2 currentForce))
				{
					double2 worldForce = RotateVector(model.Mass * (double2)relativeAcceleration, state.Orientation);
					PhysicsManager2D.Instance.SetBodyForce(_bodyIndex, currentForce + worldForce);
				}
			});
		}
		
		/// <summary>
		/// Applies a continuous force (defined in world-space).
		/// </summary>
		public void AddWorldForce(float2 worldForce)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetBodyForce(_bodyIndex, out double2 currentForce))
				{
					PhysicsManager2D.Instance.SetBodyForce(_bodyIndex, currentForce + worldForce);
				}
			});
		}
		
		/// <summary>
		/// Applies a continuous force (defined in world-space) at a specific world-space position.
		/// Modifies both force and moment
		/// </summary>
		public void AddWorldForceAtPosition(float2 worldForce, double2 worldPosition)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetStateVector(_bodyIndex, out StateVector state) &&
					PhysicsManager2D.Instance.TryGetBodyForce(_bodyIndex, out double2 currentForce) &&
					PhysicsManager2D.Instance.TryGetBodyMoment(_bodyIndex, out double currentMoment))
				{
					float2 r_world = (float2)(worldPosition - state.Position);
					float torque = Cross2D(r_world, worldForce);

					PhysicsManager2D.Instance.SetBodyForce(_bodyIndex, currentForce + worldForce);
					PhysicsManager2D.Instance.SetBodyMoment(_bodyIndex, currentMoment + torque);
				}
			});
		}

		/// <summary>
		/// Applies a continuous force (defined in body-space).
		/// </summary>
		public void AddRelativeForce(float2 relativeForce)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetStateVector(_bodyIndex, out StateVector state) &&
					PhysicsManager2D.Instance.TryGetBodyForce(_bodyIndex, out double2 currentForce))
				{
					double2 worldForce = RotateVector(relativeForce, state.Orientation);
					PhysicsManager2D.Instance.SetBodyForce(_bodyIndex, currentForce + worldForce);
				}
			});
		}
		
		/// <summary>
		/// Applies a continuous force (defined in body-space) at a specific body-space position.
		/// Modifies both force and moment
		/// </summary>
		public void AddRelativeForceAtPosition(float2 relativeForce, float2 relativePosition)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetStateVector(_bodyIndex, out StateVector state) &&
					PhysicsManager2D.Instance.TryGetBodyForce(_bodyIndex, out double2 currentForce) &&
					PhysicsManager2D.Instance.TryGetBodyMoment(_bodyIndex, out double currentMoment))
				{
					double2 worldForce = RotateVector(relativeForce, state.Orientation);
					float torque = Cross2D(relativePosition, relativeForce);
					
					PhysicsManager2D.Instance.SetBodyForce(_bodyIndex, currentForce + worldForce);
					PhysicsManager2D.Instance.SetBodyMoment(_bodyIndex, currentMoment + torque);
				}
			});
		}

		/// <summary>
		/// Applies an instantaneous impulse (defined in world-space).
		/// Modifies linear velocity.
		/// </summary>
		public void AddWorldImpulse(float2 worldImpulse)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetStateVector(_bodyIndex, out StateVector state) &&
					PhysicsManager2D.Instance.TryGetVehicleModel(_bodyIndex, out VehicleModel model))
				{
					double2 velocityChange = (double2)worldImpulse * model.InvMass;

					StateVector newState = new StateVector
					{
						Position = state.Position,
						Velocity = state.Velocity + velocityChange,
						Orientation = state.Orientation,
						AngularVelocity = state.AngularVelocity
					};

					PhysicsManager2D.Instance.SetBodyStateVector(_bodyIndex, newState);
				}
			});
		}

		/// <summary>
		/// Applies an instantaneous impulse at a specific world-space position.
		/// Modifies both linear and angular velocity.
		/// </summary>
		public void AddWorldImpulseAtPosition(float2 worldImpulse, double2 worldPosition)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetStateVector(_bodyIndex, out StateVector state) &&
					PhysicsManager2D.Instance.TryGetVehicleModel(_bodyIndex, out VehicleModel model))
				{
					double2 velocityChange = (double2)worldImpulse * model.InvMass;
					double2 r_world = worldPosition - state.Position;
					
					float angularImpulse = Cross2D((float2)r_world, worldImpulse);
					float angularVelChange = (float)((double)angularImpulse * model.InvInertiaTensor);

					StateVector newState = new StateVector
					{
						Position = state.Position,
						Velocity = state.Velocity + velocityChange,
						Orientation = state.Orientation,
						AngularVelocity = state.AngularVelocity + angularVelChange
					};

					PhysicsManager2D.Instance.SetBodyStateVector(_bodyIndex, newState);
				}
			});
		}

		/// <summary>
		/// Applies an instantaneous impulse (defined in body-space).
		/// Modifies linear velocity.
		/// </summary>
		public void AddRelativeImpulse(float2 relativeImpulse)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetStateVector(_bodyIndex, out StateVector state) &&
					PhysicsManager2D.Instance.TryGetVehicleModel(_bodyIndex, out VehicleModel model))
				{
					double2 velocityChangeRelative = (double2)relativeImpulse * model.InvMass;
					double2 velocityChangeWorld = RotateVector(velocityChangeRelative, state.Orientation);

					StateVector newState = new StateVector
					{
						Position = state.Position,
						Velocity = state.Velocity + velocityChangeWorld,
						Orientation = state.Orientation,
						AngularVelocity = state.AngularVelocity
					};

					PhysicsManager2D.Instance.SetBodyStateVector(_bodyIndex, newState);
				}
			});
		}

		/// <summary>
		/// Applies an instantaneous impulse (defined in body-space) at a specific body-space position.
		/// Modifies both linear and angular velocity.
		/// </summary>
		public void AddRelativeImpulseAtPosition(float2 relativeImpulse, float2 relativePosition)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetStateVector(_bodyIndex, out StateVector state) &&
					PhysicsManager2D.Instance.TryGetVehicleModel(_bodyIndex, out VehicleModel model))
				{
					double2 velocityChangeRelative = (double2)relativeImpulse * model.InvMass;
					double2 velocityChangeWorld = RotateVector(velocityChangeRelative, state.Orientation);
					
					float angularImpulse = Cross2D(relativePosition, relativeImpulse);
					float angularVelChange = (float)((double)angularImpulse * model.InvInertiaTensor);

					StateVector newState = new StateVector
					{
						Position = state.Position,
						Velocity = state.Velocity + velocityChangeWorld,
						Orientation = state.Orientation,
						AngularVelocity = state.AngularVelocity + angularVelChange
					};

					PhysicsManager2D.Instance.SetBodyStateVector(_bodyIndex, newState);
				}
			});
		}

		/// <summary>
		/// Applies an instantaneous velocity change (defined in world-space).
		/// Modifies linear velocity.
		/// </summary>
		public void AddWorldVelocity(float2 worldVelocity)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetStateVector(_bodyIndex, out StateVector state))
				{
					StateVector newState = new StateVector
					{
						Position = state.Position,
						Velocity = state.Velocity + worldVelocity,
						Orientation = state.Orientation,
						AngularVelocity = state.AngularVelocity
					};

					PhysicsManager2D.Instance.SetBodyStateVector(_bodyIndex, newState);
				}
			});
		}

		/// <summary>
		/// Applies an instantaneous velocity change (defined in body-space).
		/// Modifies linear velocity.
		/// </summary>
		public void AddRelativeVelocity(float2 relativeVelocity)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetStateVector(_bodyIndex, out StateVector state))
				{
					double2 velocityChangeWorld = RotateVector(relativeVelocity, state.Orientation);

					StateVector newState = new StateVector
					{
						Position = state.Position,
						Velocity = state.Velocity + velocityChangeWorld,
						Orientation = state.Orientation,
						AngularVelocity = state.AngularVelocity
					};

					PhysicsManager2D.Instance.SetBodyStateVector(_bodyIndex, newState);
				}
			});
		}
		
		/// <summary>
		/// Adds a continuous world-space torque to the body.
		/// </summary>
		public void AddTorque(float worldTorque)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetBodyMoment(_bodyIndex, out double currentMoment))
				{
					PhysicsManager2D.Instance.SetBodyMoment(_bodyIndex, currentMoment + worldTorque);
				}
			});
		}
		#endregion

		#region Positioning
		/// <summary>
		/// Sets physics body position (defined in world-space).
		/// </summary>
		public void MoveTo(float2 position)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetStateVector(_bodyIndex, out StateVector state))
				{
					StateVector newState = new StateVector
					{
						Position = position,
						Velocity = state.Velocity,
						Orientation = state.Orientation,
						AngularVelocity = state.AngularVelocity
					};

					PhysicsManager2D.Instance.SetBodyStateVector(_bodyIndex, newState);
				}
			});
		}

		/// <summary>
		/// Moves the physics body by deltaPosition.
		/// </summary>
		public void Move(float2 deltaPosition)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetStateVector(_bodyIndex, out StateVector state))
				{
					StateVector newState = new StateVector
					{
						Position = state.Position + deltaPosition,
						Velocity = state.Velocity,
						Orientation = state.Orientation,
						AngularVelocity = state.AngularVelocity
					};

					PhysicsManager2D.Instance.SetBodyStateVector(_bodyIndex, newState);
				}
			});
		}

		/// <summary>
		/// Sets the body orientation.
		/// </summary>
		public void Look(float orientation)
		{
			ExecuteOrDefer(() =>
			{
				if (PhysicsManager2D.Instance.TryGetStateVector(_bodyIndex, out StateVector state))
				{
					StateVector newState = new StateVector
					{
						Position = state.Position,
						Velocity = state.Velocity,
						Orientation = orientation,
						AngularVelocity = state.AngularVelocity
					};

					PhysicsManager2D.Instance.SetBodyStateVector(_bodyIndex, newState);
				}
			});
		}
		#endregion
	
		// Utility mathematical overrides for 2D vectors
		private float2 RotateVector(float2 vector, float angle)
		{
			float s = math.sin(angle);
			float c = math.cos(angle);
			return new float2(vector.x * c - vector.y * s, vector.x * s + vector.y * c);
		}

		// Utility mathematical overrides for 2D vectors
		private double2 RotateVector(double2 vector, float angle)
		{
			float s = math.sin(angle);
			float c = math.cos(angle);
			return new double2(vector.x * c - vector.y * s, vector.x * s + vector.y * c);
		}

		private float Cross2D(float2 a, float2 b)
		{
			return a.x * b.y - a.y * b.x;
		}
	}
}