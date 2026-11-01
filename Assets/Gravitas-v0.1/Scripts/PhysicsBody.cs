using System;
using System.Runtime.InteropServices;
using Gravitas;
using Unity.Burst;
using Unity.Mathematics;
using UnityEngine;

namespace Gravitas
{
	[StructLayout(LayoutKind.Sequential)]
	public struct PhysicsBodyData
	{
		public StateVector stateVector { get; private set; }
		public VehicleModel vmod { get; private set; }
		public float3 externalForce { get; private set; }
		public float3 externalMoment { get; private set; }

		public PhysicsBodyData(in StateVector stateVector, VehicleModel vmod, float3 externalForce, float3 externalMoment)
		{
			this.stateVector = stateVector;

			this.vmod = vmod;

			this.externalForce = externalForce;
			this.externalMoment = externalMoment;

			vmod.Initialize();
		}
	}
	
	public class PhysicsBody
	{
		Rigidbody rb;
		public string name { get; private set; }
		public bool isCelestial { get; private set; }
		public bool isKinematic { get; private set; }
		// public double radius { get; private set; }

		int physicsBodyIndex;

		bool isStateDirty = false;

		static int NUM_ADDED = 0;

		public PhysicsBody(in StateVector startState, VehicleModel vmod, bool isKinematic, bool isCelestial, string name = null, Rigidbody rb = null)
		{
			this.name = name == null ? $"physicsBody_{NUM_ADDED++}" : name;
			this.isCelestial = isCelestial;
			this.isKinematic = isKinematic;
			// this.radius = radius;

			// if (rb == null)
			// {
			// 	if (isCelestial)
			// 	{
			// 		rb = CreateRigidbodyPrimitive(this.name, (float3)startState.position, PrimitiveType.Sphere, (float)radius * 2);
			// 	}
			// 	else
			// 	{
			// 		rb = CreateRigidbodyPrimitive(this.name, (float3)startState.position, PrimitiveType.Cube);
			// 	}
			// }

			this.rb = rb;

			if (isKinematic)
			{
				physicsBodyIndex = PhysicsManager.AddKinematicBody(this, startState, vmod);
			}
			else
			{
				physicsBodyIndex = PhysicsManager.AddBody(this, startState, vmod);
			}
			
			UpdateScenePosition();
			
			PhysicsManager.OnPhysicsUpdated += UpdateScenePosition;
			//PhysicsManager.OnOffsetBodies += OffsetPosition;
		}

		[BurstCompile]
		public void Remove()
		{
			if (isKinematic)
			{
				PhysicsManager.RemoveKinematicBody(physicsBodyIndex);
			}
			else
			{
				PhysicsManager.RemoveBody(physicsBodyIndex);
			}

			PhysicsManager.OnPhysicsUpdated -= UpdateScenePosition;
			//PhysicsManager.OnOffsetBodies -= OffsetPosition;
			physicsBodyIndex = -1;
		}

		[BurstCompile]
		public void ConvertToKinematic()
		{
			PhysicsManager.ConvertPhysicsBodyToKinematicBody(physicsBodyIndex);
		}

		[BurstCompile]
		public void ConvertToPhysics()
		{
			PhysicsManager.ConvertKinematicBodyToPhysicsBody(physicsBodyIndex);
		}

		#region Forcing and Torquing
		/// <summary>
		/// Applies an instantaneous acceleration (defined in world-space).
		/// </summary>
		[BurstCompile]
		public void AddWorldAcceleration(float3 worldAcceleation)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);
			var currentState = currentBodyData.stateVector;

			float3 worldForce = currentBodyData.vmod.mass * worldAcceleation; // Convert acceleration vector into equivalent force vector
			float3 bodyForce = math.rotate(math.inverse(currentState.orientation), worldForce);
			float3 newForce = currentBodyData.externalForce + bodyForce;

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(currentState, currentBodyData.vmod, newForce, currentBodyData.externalMoment), isKinematic);
		}

		/// <summary>
		/// Applies an instantaneous acceleration (defined in body-space).
		/// </summary>
		[BurstCompile]
		public void AddRelativeAcceleration(float3 relativeAcceleation)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);

			float3 relativeForce = currentBodyData.vmod.mass * relativeAcceleation; // Convert acceleration vector into equivalent force vector
			float3 newForce = currentBodyData.externalForce + relativeForce;

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(currentBodyData.stateVector, currentBodyData.vmod, newForce, currentBodyData.externalMoment), isKinematic);
		}
		
		/// <summary>
		/// Applies a continuous force (defined in world-space).
		/// </summary>
		[BurstCompile]
		public void AddWorldForce(float3 worldForce)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);
			var currentState = currentBodyData.stateVector;

			float3 bodyForce = math.rotate(math.inverse(currentState.orientation), worldForce);
			float3 newForce = currentBodyData.externalForce + bodyForce;

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(currentState, currentBodyData.vmod, newForce, currentBodyData.externalMoment), isKinematic);
		}
		
		/// <summary>
		/// Applies a continuous force (defined in world-space) at a specific body-space position.
		/// Modifies both force and moment
		/// </summary>
		[BurstCompile]
		public void AddWorldForceAtPosition(float3 worldForce, float3 worldPosition)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);
			var currentState = currentBodyData.stateVector;

			// Calculate force
			float3 bodyForce = math.rotate(math.inverse(currentState.orientation), worldForce);
			float3 newForce = currentBodyData.externalForce + bodyForce;

			// Calculate moment
			float3 r_world = (float3)((double3)worldPosition - currentState.position);
			float3 r_body = math.rotate(math.inverse(currentState.orientation), r_world);

			float3 torque_body = math.cross(r_body, bodyForce);
			float3 newMoment = currentBodyData.externalMoment + torque_body;

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(currentState, currentBodyData.vmod, newForce, newMoment), isKinematic);
		}

		/// <summary>
		/// Applies a continuous force (defined in body-space).
		/// </summary>
		[BurstCompile]
		public void AddRelativeForce(float3 relativeForce)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);

			float3 newForce = currentBodyData.externalForce + relativeForce;

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(currentBodyData.stateVector, currentBodyData.vmod, newForce, currentBodyData.externalMoment), isKinematic);
		}
		
		/// <summary>
		/// Applies a continuous force (defined in body-space) at a specific body-space position.
		/// Modifies both force and moment
		/// </summary>
		[BurstCompile]
		public void AddRelativeForceAtPosition(float3 relativeForce, float3 relativePosition)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);
			var currentState = currentBodyData.stateVector;

			// Calculate force
			float3 newForce = currentBodyData.externalForce + relativeForce;

			// Calculate moment
			float3 torque_body = math.cross(relativePosition, relativeForce);
			float3 newMoment = currentBodyData.externalMoment + torque_body;

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(currentState, currentBodyData.vmod, newForce, newMoment), isKinematic);
		}

		/// <summary>
		/// Applies an instantaneous impulse (defined in world-space).
		/// Modifies linear velocity.
		/// </summary>
		[BurstCompile]
		public void AddWorldImpulse(float3 worldImpulse)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);
			var currentState = currentBodyData.stateVector;

			double3 velocityChange = worldImpulse / currentBodyData.vmod.mass;

			StateVector newState = new StateVector
			{
				position = currentState.position,
				velocity = currentState.velocity + velocityChange,
				orientation = currentState.orientation,
				angularVelocity = currentState.angularVelocity
			};

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(newState, currentBodyData.vmod, currentBodyData.externalForce, currentBodyData.externalMoment), isKinematic);
		}

		/// <summary>
		/// Applies an instantaneous impulse at a specific world-space position.
		/// Modifies both linear and angular velocity.
		/// </summary>
		[BurstCompile]
		public void AddWorldImpulseAtPosition(float3 worldImpulse, float3 worldPosition)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);
			var currentState = currentBodyData.stateVector;
			var vmod = currentBodyData.vmod;

			// Calculate new velocity
			float3 velocityChange = worldImpulse / vmod.mass;
			double3 newVelocity = currentState.velocity + velocityChange;

			// Calculate new angular velocity
			float3 r_world = (float3)((double3)worldPosition - currentState.position);
			float3 angularImpulse_world = math.cross(r_world, worldImpulse);
			float3 angularImpulse_body = math.rotate(math.inverse(currentState.orientation), angularImpulse_world);
			float3 angularVelChange_body = math.mul(vmod.InvInertiaTensor, angularImpulse_body);
			float3 newAngularVelocity = currentState.angularVelocity + angularVelChange_body;

			StateVector newState = new StateVector
			{
				position = currentState.position,
				velocity = newVelocity,
				orientation = currentState.orientation,
				angularVelocity = newAngularVelocity
			};

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(newState, vmod, currentBodyData.externalForce, currentBodyData.externalMoment), isKinematic);
		}

		/// <summary>
		/// Applies an instantaneous impulse (defined in body-space).
		/// Modifies linear velocity.
		/// </summary>
		[BurstCompile]
		public void AddRelativeImpulse(float3 relativeImpulse)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);
			var currentState = currentBodyData.stateVector;

			float3 velocityChangeRelative = relativeImpulse / currentBodyData.vmod.mass;
			double3 velocityChangeWorld = math.rotate(currentBodyData.stateVector.orientation, velocityChangeRelative);

			StateVector newState = new StateVector
			{
				position = currentState.position,
				velocity = currentState.velocity + velocityChangeWorld,
				orientation = currentState.orientation,
				angularVelocity = currentState.angularVelocity
			};

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(newState, currentBodyData.vmod, currentBodyData.externalForce, currentBodyData.externalMoment), isKinematic);
		}

		/// <summary>
		/// Applies an instantaneous impulse (defined in body-space) at a specific body-space position.
		/// Modifies both linear and angular velocity.
		/// </summary>
		[BurstCompile]
		public void AddRelativeImpulseAtPosition(float3 relativeImpulse, float3 relativePosition)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);
			var currentState = currentBodyData.stateVector;
			var vmod = currentBodyData.vmod;

			// Calcualte new velocity
			float3 velocityChangeRelative = relativeImpulse / vmod.mass;
			double3 velocityChangeWorld = math.rotate(currentBodyData.stateVector.orientation, velocityChangeRelative);
			double3 newVelocity = currentState.velocity + velocityChangeWorld;

			// Calculate new angular velocity
			float3 angularImpulse_body = math.cross(relativePosition, relativeImpulse);
			float3 angularVelChange_body = math.mul(vmod.InvInertiaTensor, angularImpulse_body);
			float3 newAngularVelocity = currentState.angularVelocity + angularVelChange_body;

			StateVector newState = new StateVector
			{
				position = currentState.position,
				velocity = newVelocity,
				orientation = currentState.orientation,
				angularVelocity = newAngularVelocity
			};

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(newState, vmod, currentBodyData.externalForce, currentBodyData.externalMoment), isKinematic);
		}

		/// <summary>
		/// Applies an instantaneous velocity change (defined in world-space).
		/// Modifies linear velocity.
		/// </summary>
		[BurstCompile]
		public void AddWorldVelocity(float3 worldVelocity)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);
			var currentState = currentBodyData.stateVector;

			StateVector newState = new StateVector
			{
				position = currentState.position,
				velocity = currentState.velocity + worldVelocity,
				orientation = currentState.orientation,
				angularVelocity = currentState.angularVelocity
			};

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(newState, currentBodyData.vmod, currentBodyData.externalForce, currentBodyData.externalMoment), isKinematic);
		}

		/// <summary>
		/// Applies an instantaneous velocity change (defined in body-space).
		/// Modifies linear velocity.
		/// </summary>
		[BurstCompile]
		public void AddRelativeVelocity(float3 relativeVelocity)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);
			var currentState = currentBodyData.stateVector;
			double3 velocityChangeWorld = math.rotate(currentState.orientation, relativeVelocity);

			StateVector newState = new StateVector
			{
				position = currentState.position,
				velocity = currentState.velocity + velocityChangeWorld,
				orientation = currentState.orientation,
				angularVelocity = currentState.angularVelocity
			};

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(newState, currentBodyData.vmod, currentBodyData.externalForce, currentBodyData.externalMoment), isKinematic);
		}
		
		/// <summary>
		/// Adds a continuous world-space torque to the body.
		/// </summary>
		[BurstCompile]
		public void AddWorldTorque(float3 worldTorque)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);
			var currentState = currentBodyData.stateVector;

			// Rotate world torque into body-space
			float3 torque_body = math.rotate(math.inverse(currentState.orientation), worldTorque);
			float3 newMoment = currentBodyData.externalMoment + torque_body;
			
			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(currentState, currentBodyData.vmod, currentBodyData.externalForce, newMoment), isKinematic);
		}

		/// <summary>
		/// Adds a continuous body-space (relative) torque to the body.
		/// </summary>
		[BurstCompile]
		public void AddRelativeTorque(float3 relativeTorque)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);

			float3 newMoment = currentBodyData.externalMoment + relativeTorque;

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(currentBodyData.stateVector, currentBodyData.vmod, currentBodyData.externalForce, newMoment), isKinematic);
		}
		#endregion

		#region Positioning
		[BurstCompile]
		private void UpdateScenePosition()
		{
			if (physicsBodyIndex < 0) return;

			// rb?.MovePosition(FloatingOrigin.GetFloatingPosition((float3)(position - PhysicsManager.playerMovementVector)));
			rb?.MovePosition((float3)PhysicsManager.GetScenePosition(position));
			rb?.MoveRotation(orientation);
		}

		/// <summary>
		/// Sets physics body position (defined in world-space).
		/// </summary>
		[BurstCompile]
		public void MoveTo(float3 position)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);
			var currentState = currentBodyData.stateVector;

			StateVector newState = new StateVector
			{
				position = position,
				velocity = currentState.velocity,
				orientation = currentState.orientation,
				angularVelocity = currentState.angularVelocity
			};

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(newState, currentBodyData.vmod, currentBodyData.externalForce, currentBodyData.externalMoment), isKinematic);
		}

		/// <summary>
		/// Moves the physics body by deltaPosition.
		/// </summary>
		[BurstCompile]
		public void Move(float3 deltaPosition)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);
			var currentState = currentBodyData.stateVector;

			StateVector newState = new StateVector
			{
				position = currentState.position + deltaPosition,
				velocity = currentState.velocity,
				orientation = currentState.orientation,
				angularVelocity = currentState.angularVelocity
			};

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(newState, currentBodyData.vmod, currentBodyData.externalForce, currentBodyData.externalMoment), isKinematic);
		}

		/// <summary>
		/// Sets the body orientation.
		/// </summary>
		[BurstCompile]
		public void Look(Quaternion rotation)
		{
			var currentBodyData = PhysicsManager.GetBodyData(physicsBodyIndex, isKinematic);
			var currentState = currentBodyData.stateVector;

			StateVector newState = new StateVector
			{
				position = currentState.position,
				velocity = currentState.velocity,
				orientation = rotation,
				angularVelocity = currentState.angularVelocity
			};

			PhysicsManager.UpdateBodyData(physicsBodyIndex, new PhysicsBodyData(newState, currentBodyData.vmod, currentBodyData.externalForce, currentBodyData.externalMoment), isKinematic);
		}
		#endregion

		[BurstCompile]
		public void DeclarePlayerBody()
		{
			PhysicsManager.SetPlayerBody(this);
			Debug.Log($"{name} is now the player physics body.");
		}

		[BurstCompile]
		public void RegisterCollision(in Collision collision, in PhysicsBody otherBody, float penetration)
		{
			int otherBodyIndex = otherBody.physicsBodyIndex;
			// PhysicsManager.RegisterCollision(physicsBodyIndex, otherBodyIndex, collision, penetration);
			if (physicsBodyIndex < otherBodyIndex)
			{
				PhysicsManager.RegisterCollision(physicsBodyIndex, otherBodyIndex, collision, penetration);
			}
		}

		[BurstCompile]
		public void RegisterCollision(in Collision collision, in Collider collider, in Transform transform)
		{
			PhysicsBody otherBody = PhysicsManager.GetBodyFromCollider(collision.collider);

			int otherBodyIndex = otherBody.physicsBodyIndex;
			if (physicsBodyIndex < otherBodyIndex) return;

			for (int i = 0; i < collision.contactCount; i++)
			{
				ContactPoint contact = collision.GetContact(i);

				Physics.ComputePenetration(collider, transform.position, transform.rotation, collision.collider, collision.transform.position, collision.transform.rotation, out _, out float penetrationDepth);
				PhysicsManager.RegisterContact(physicsBodyIndex, otherBodyIndex, contact.point, contact.normal, penetrationDepth);

				// const float raycastStartOffset = 5.0f;
				// const float maxPenetration = 10.0f;

				// Vector3 rayStart = contact.point + contact.normal * raycastStartOffset;
				// Ray ray = new Ray(rayStart, -contact.normal);

				// var context = new RaycastContext
				// {
				// 	bodyIndexA = physicsBodyIndex,
				// 	bodyIndexB = otherBodyIndex,
				// 	contactPoint = contact.point,
				// 	contactNormal = contact.normal,
				// 	raycastStartOffset = raycastStartOffset,
				// 	distance = raycastStartOffset + maxPenetration
				// };

				// PhysicsManager.QueueRaycast(context);
				// continue;


				// if (collision.collider.Raycast(ray, out RaycastHit hit, raycastStartOffset + maxPenetration))
				// {
				// 	float penetrationDepth = raycastStartOffset - hit.distance;

				// 	if (penetrationDepth > 0.0001f)
				// 	{
				// 		PhysicsManager.RegisterContact(physicsBodyIndex, otherBodyIndex, contact.point, contact.normal, penetrationDepth);
				// 	}
				// }
			}
		}

		[BurstCompile]
		public void DecrementIndex()
		{
			physicsBodyIndex--;
		}

		[BurstCompile]
		public void MarkDirty()
		{
			isStateDirty = true;
		}

		public double3 position => PhysicsManager.GetBodyState(physicsBodyIndex, isKinematic).position;
		public double3 velocity => PhysicsManager.GetBodyState(physicsBodyIndex, isKinematic).velocity;
		public quaternion orientation => PhysicsManager.GetBodyState(physicsBodyIndex, isKinematic).orientation;
		public float3 angularVelocity => PhysicsManager.GetBodyState(physicsBodyIndex, isKinematic).angularVelocity;

		public Rigidbody CreateRigidbodyPrimitive(string name, Vector3 position, PrimitiveType type, float scale = 1.0f)
		{
			// 1. Create the basic GameObject
			GameObject capsuleGO = GameObject.CreatePrimitive(type);
			capsuleGO.transform.position = position;
			capsuleGO.name = name;
			capsuleGO.transform.localScale = new Vector3(scale, scale, scale);

			Rigidbody rb = capsuleGO.AddComponent<Rigidbody>();
			rb.mass = 1.0f;
			rb.useGravity = false;
			rb.isKinematic = true;

			// Collider collider = capsuleGO.GetComponent<CapsuleCollider>();
			// GameObject.Destroy(collider);

			// TrailRenderer trailRenderer = capsuleGO.AddComponent<TrailRenderer>();
			// trailRenderer.startWidth = 0.1f;
			// trailRenderer.endWidth = 0.1f;
			// trailRenderer.time = 100f;
			// var defaultMaterial = Resources.Load<Material>("Default-Line");
			// if (defaultMaterial != null)
			// {
			// 	trailRenderer.material = defaultMaterial;
			// 	trailRenderer.material.color = Color.white; // Set a default color
			// }

			return rb;
		}
	}
}