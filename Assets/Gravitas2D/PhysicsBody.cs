using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace Gravitas2D
{
	public struct PhysicsBodyData
	{
		public StateVector StateVector;
		public VehicleModel VehicleModel;
		public double2 ExternalForce;
		public double ExternalMoment;

		public PhysicsBodyData(in StateVector stateVector, VehicleModel vehicleModel, double2 externalForce, double externalMoment)
		{
			StateVector = stateVector;
			VehicleModel = vehicleModel;
			ExternalForce = externalForce;
			ExternalMoment = externalMoment;
		}

		public static PhysicsBodyData Stripped(StateVector stateVector, VehicleModel vehicleModel)
		{
			return new PhysicsBodyData(stateVector, vehicleModel, double2.zero, 0);
		}
	}

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

		public void SyncStateFromManager(in PhysicsBodyData data)
		{
			if (_isTracked)
			{
				OnTrackedPositionUpdated?.Invoke(data.StateVector.Position - _currentStateVector.Position);
			}

			_currentStateVector = data.StateVector;
			_vehicleModel = data.VehicleModel;

			FlushPendingOperations();

			UpdateSoI();
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					double2 worldForce = data.VehicleModel.Mass * (double2)worldAcceleration;
					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(data.StateVector, data.VehicleModel, data.ExternalForce + worldForce, data.ExternalMoment));
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					double2 worldForce = RotateVector(data.VehicleModel.Mass * (double2)relativeAcceleration, data.StateVector.Orientation); 
					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(data.StateVector, data.VehicleModel, data.ExternalForce + worldForce, data.ExternalMoment));
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(data.StateVector, data.VehicleModel, data.ExternalForce + worldForce, data.ExternalMoment));
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					float2 r_world = (float2)(worldPosition - data.StateVector.Position);
					float torque = Cross2D(r_world, worldForce);

					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(data.StateVector, data.VehicleModel, data.ExternalForce + worldForce, data.ExternalMoment + torque));
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					float2 worldForce = RotateVector(relativeForce, data.StateVector.Orientation);
					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(data.StateVector, data.VehicleModel, data.ExternalForce + worldForce, data.ExternalMoment));
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					float2 worldForce = RotateVector(relativeForce, data.StateVector.Orientation);
					float torque = Cross2D(relativePosition, relativeForce);
					
					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(data.StateVector, data.VehicleModel, data.ExternalForce + worldForce, data.ExternalMoment + torque));
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					double2 velocityChange = (double2)worldImpulse * data.VehicleModel.InvMass;

					StateVector newState = new StateVector
					{
						Position = data.StateVector.Position,
						Velocity = data.StateVector.Velocity + velocityChange,
						Orientation = data.StateVector.Orientation,
						AngularVelocity = data.StateVector.AngularVelocity
					};

					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(newState, data.VehicleModel, data.ExternalForce, data.ExternalMoment));
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					double2 velocityChange = (double2)worldImpulse * data.VehicleModel.InvMass;
					double2 r_world = worldPosition - data.StateVector.Position;
					
					float angularImpulse = Cross2D((float2)r_world, worldImpulse);
					float angularVelChange = (float)((double)angularImpulse * data.VehicleModel.InvInertiaTensor);

					StateVector newState = new StateVector
					{
						Position = data.StateVector.Position,
						Velocity = data.StateVector.Velocity + velocityChange,
						Orientation = data.StateVector.Orientation,
						AngularVelocity = data.StateVector.AngularVelocity + angularVelChange
					};

					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(newState, data.VehicleModel, data.ExternalForce, data.ExternalMoment));
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					double2 velocityChangeRelative = (double2)relativeImpulse * data.VehicleModel.InvMass;
					double2 velocityChangeWorld = RotateVector(velocityChangeRelative, data.StateVector.Orientation);

					StateVector newState = new StateVector
					{
						Position = data.StateVector.Position,
						Velocity = data.StateVector.Velocity + velocityChangeWorld,
						Orientation = data.StateVector.Orientation,
						AngularVelocity = data.StateVector.AngularVelocity
					};

					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(newState, data.VehicleModel, data.ExternalForce, data.ExternalMoment));
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					double2 velocityChangeRelative = (double2)relativeImpulse * data.VehicleModel.InvMass;
					double2 velocityChangeWorld = RotateVector(velocityChangeRelative, data.StateVector.Orientation);
					
					float angularImpulse = Cross2D(relativePosition, relativeImpulse);
					float angularVelChange = (float)((double)angularImpulse * data.VehicleModel.InvInertiaTensor);

					StateVector newState = new StateVector
					{
						Position = data.StateVector.Position,
						Velocity = data.StateVector.Velocity + velocityChangeWorld,
						Orientation = data.StateVector.Orientation,
						AngularVelocity = data.StateVector.AngularVelocity + angularVelChange
					};

					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(newState, data.VehicleModel, data.ExternalForce, data.ExternalMoment));
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					StateVector newState = new StateVector
					{
						Position = data.StateVector.Position,
						Velocity = data.StateVector.Velocity + worldVelocity,
						Orientation = data.StateVector.Orientation,
						AngularVelocity = data.StateVector.AngularVelocity
					};

					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(newState, data.VehicleModel, data.ExternalForce, data.ExternalMoment));
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					double2 velocityChangeWorld = RotateVector(relativeVelocity, data.StateVector.Orientation);

					StateVector newState = new StateVector
					{
						Position = data.StateVector.Position,
						Velocity = data.StateVector.Velocity + velocityChangeWorld,
						Orientation = data.StateVector.Orientation,
						AngularVelocity = data.StateVector.AngularVelocity
					};

					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(newState, data.VehicleModel, data.ExternalForce, data.ExternalMoment));
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(data.StateVector, data.VehicleModel, data.ExternalForce, data.ExternalMoment + worldTorque));
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					var currentState = data.StateVector;

					StateVector newState = new StateVector
					{
						Position = position,
						Velocity = currentState.Velocity,
						Orientation = currentState.Orientation,
						AngularVelocity = currentState.AngularVelocity
					};

					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(newState, data.VehicleModel, data.ExternalForce, data.ExternalMoment));
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					var currentState = data.StateVector;

					StateVector newState = new StateVector
					{
						Position = currentState.Position + deltaPosition,
						Velocity = currentState.Velocity,
						Orientation = currentState.Orientation,
						AngularVelocity = currentState.AngularVelocity
					};

					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(newState, data.VehicleModel, data.ExternalForce, data.ExternalMoment));
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
				if (PhysicsManager2D.Instance.TryGetBodyData(_bodyIndex, out PhysicsBodyData data))
				{
					var currentState = data.StateVector;

					StateVector newState = new StateVector
					{
						Position = currentState.Position,
						Velocity = currentState.Velocity,
						Orientation = orientation,
						AngularVelocity = currentState.AngularVelocity
					};

					PhysicsManager2D.Instance.UpdateBodyData(_bodyIndex, new PhysicsBodyData(newState, data.VehicleModel, data.ExternalForce, data.ExternalMoment));
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