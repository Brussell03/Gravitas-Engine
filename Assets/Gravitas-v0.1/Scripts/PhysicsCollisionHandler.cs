using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace Gravitas
{
	[RequireComponent(typeof(Collider))]
	public class PhysicsCollisionHandler : MonoBehaviour
	{
		public PhysicsBody physicsBody { get; private set; }
		Collider physicsCollider;

		void Awake()
		{
			// Cache the collider for performance
			physicsCollider = GetComponent<Collider>();
			if (physicsCollider == null)
			{
				Debug.LogError($"{name} has no Collider component!", this);
			}
		}

		public void SetPhysicsBody(PhysicsBody physicsBody)
		{
			this.physicsBody = physicsBody;

			if (!PhysicsManager.colliderBodyMap.ContainsKey(physicsCollider))
			{
				PhysicsManager.colliderBodyMap.Add(physicsCollider, physicsBody);
			}
		}

		void OnCollisionEnter(Collision collision)
		{
			//Debug.Log($"{name} collided with {collision.gameObject.name}.");
			// HandleCollision(collision);
			// HandleContacts(collision);
			physicsBody.RegisterCollision(collision, physicsCollider, transform);
		}

		void OnCollisionStay(Collision collision)
		{
			// HandleCollision(collision);
			// HandleContacts(collision);
			physicsBody.RegisterCollision(collision, physicsCollider, transform);
		}

		void HandleCollision(Collision collision)
		{
			if (physicsBody == null || physicsCollider == null) return;

			Physics.ComputePenetration(physicsCollider, transform.position, transform.rotation, collision.collider, collision.transform.position, collision.transform.rotation, out _, out float distance);

			PhysicsBody otherBody = PhysicsManager.GetBodyFromCollider(collision.collider);

			physicsBody.RegisterCollision(collision, otherBody, distance);
		}

		void OnDestroy()
		{
			if (physicsCollider != null && PhysicsManager.colliderBodyMap.ContainsKey(physicsCollider))
			{
				PhysicsManager.colliderBodyMap.Remove(physicsCollider);
			}
		}
	}

	public struct RaycastContext
	{
		public int bodyIndexA;
		public int bodyIndexB;
		public float3 contactPoint;
		public float3 contactNormal;
		public float raycastStartOffset;
		public float distance;
	}

	public struct PrepareRaycastCommandJob : IJobParallelFor
	{
		[WriteOnly] public NativeArray<RaycastCommand> RaycastCommands;
		[ReadOnly] public NativeArray<RaycastContext> RaycastContexts;

		public void Execute(int index)
		{
			Vector3 rayStart = RaycastContexts[index].contactPoint + RaycastContexts[index].contactNormal * RaycastContexts[index].raycastStartOffset;
			var queryParams = new QueryParameters(-1, false, QueryTriggerInteraction.UseGlobal, true);
			RaycastCommands[index] = new RaycastCommand(rayStart, -RaycastContexts[index].contactNormal, queryParams, RaycastContexts[index].distance);
		}
	}

	[BurstCompile]
	public struct ProcessRaycastHitsJob : IJobParallelFor
	{
		[ReadOnly] public NativeArray<RaycastHit> RaycastHits;
		[ReadOnly] public NativeArray<RaycastContext> RaycastContexts;
		[ReadOnly] public double3 playerPos;
		
		// public NativeList<DynamicContactInfo>.ParallelWriter ContactWriter;
		[WriteOnly] public NativeArray<DynamicContactInfo> Contacts;

		public void Execute(int index)
		{
			RaycastHit hit = RaycastHits[index];
            RaycastContext context = RaycastContexts[index];
			
			// Convert hit point from scene space to physics space
			double3 absoluteHitPoint = new double3(hit.point) + playerPos;

            // In a batch, a "miss" has no valid collider
            if (hit.normal == Vector3.zero) 
			{
				Contacts[index] = new DynamicContactInfo
				{
					bodyIndexA = context.bodyIndexB, // LOW index
					bodyIndexB = context.bodyIndexA, // HIGH index
					worldNormal = -context.contactNormal, // Already points LOW -> HIGH
					worldPoint = absoluteHitPoint,
					penetrationDepth = 1
				};
				return;
			}

			float penetrationDepth = context.raycastStartOffset - hit.distance;

			if (penetrationDepth > 0.0001f)
			{

				Contacts[index] = new DynamicContactInfo
				{
					bodyIndexA = context.bodyIndexB, // LOW index
					bodyIndexB = context.bodyIndexA, // HIGH index
					worldNormal = -context.contactNormal, // Already points LOW -> HIGH
					worldPoint = absoluteHitPoint,
					penetrationDepth = penetrationDepth
				};
			}
			else
			{
				// --- FIX 2: EXPLICITLY WRITE DEFAULT ON NO PENETRATION ---
				Contacts[index] = new DynamicContactInfo
				{
					bodyIndexA = context.bodyIndexB, // LOW index
					bodyIndexB = context.bodyIndexA, // HIGH index
					worldNormal = -context.contactNormal, // Already points LOW -> HIGH
					worldPoint = absoluteHitPoint,
					penetrationDepth = 1
				};
			}
		}
	}

	[BurstCompile]
	public struct ProcessStaticCollisionsJob : IJob
	{
		// Make Bodies Read/Write
		public NativeArray<PhysicsBodyData> Bodies;
		public NativeQueue<StaticContactInfo> CollisionQueue;

		[ReadOnly] public double restitution;
		[ReadOnly] public double staticFriction;
		[ReadOnly] public double dynamicFriction;
		[ReadOnly] public float timeStep; // Must pass in Time.fixedDeltaTime

		// Solver settings
		private const int solverIterations = 8;
		private const double penetrationSlop = 0.0;
		private const double positionCorrectionFactor = 0.1;
		private const double restingVelocityThreshold = 0.5;

		// A struct to hold pre-calculated contact data
		private struct ContactConstraint
		{
			public int bodyIndex;
			public double3 r_a; // Lever arm (world space)
			public double3 r_cross_n;
			public double3 worldNormal;
			public double penetration;

			// Pre-calculated "effective mass" for normal and tangent
			public double invMassNormal;
			public double invMassTangent1;
			public double invMassTangent2;
			public double invMassTwist;

			// Tangent directions
			public double3 tangent1;
			public double3 tangent2;

			// Total accumulated impulses (for clamping)
			public double accumulatedNormalImpulse;
			public double accumulatedFrictionImpulse1;
			public double accumulatedFrictionImpulse2;
			public double accumulatedTwistImpulse;
		}

		public void Execute()
		{
			// Dequeue all contacts and pre-calculate constraint data
			NativeList<ContactConstraint> constraints = new NativeList<ContactConstraint>(Allocator.Temp);
			while (CollisionQueue.TryDequeue(out StaticContactInfo collision))
			{
				PhysicsBodyData body = Bodies[collision.bodyIndex];
				StateVector state = body.stateVector;
				float3x3 I_inv_body = body.vmod.InvInertiaTensor;
				double inv_mass = body.vmod.invMass;

				// --- Calculate World-Space Inverse Inertia Tensor ---
				float3x3 R = new float3x3(state.orientation);
				float3x3 R_T = math.transpose(R);
				float3x3 I_inv_world = math.mul(math.mul(R, I_inv_body), R_T);

				double3 r_a = collision.worldPoint - state.position;
				double3 n = (double3)collision.worldNormal;

				// --- Create Tangent Vectors ---
				CreateBasis(n, out double3 t1, out double3 t2);

				// --- Pre-calculate Effective Mass Denominators ---
				double3 r_cross_n = math.cross(r_a, n);
				double3 angular_comp_n = math.cross(math.mul(I_inv_world, r_cross_n), r_a);
				double denom_n = inv_mass + math.dot(n, angular_comp_n);

				double3 r_cross_t1 = math.cross(r_a, t1);
				double3 angular_comp_t1 = math.cross(math.mul(I_inv_world, r_cross_t1), r_a);
				double denom_t1 = inv_mass + math.dot(t1, angular_comp_t1);

				double3 r_cross_t2 = math.cross(r_a, t2);
				double3 angular_comp_t2 = math.cross(math.mul(I_inv_world, r_cross_t2), r_a);
				double denom_t2 = inv_mass + math.dot(t2, angular_comp_t2);

				double3 angular_comp_twist = math.mul(I_inv_world, n);
				double denom_twist = math.dot(n, angular_comp_twist);

				constraints.Add(new ContactConstraint
				{
					bodyIndex = collision.bodyIndex,
					r_a = r_a,
					r_cross_n = r_cross_n,
					worldNormal = n,
					penetration = collision.penetrationDepth,
					tangent1 = t1,
					tangent2 = t2,
					invMassNormal = (denom_n > 1e-9) ? 1.0 / denom_n : 0.0,
					invMassTangent1 = (denom_t1 > 1e-9) ? 1.0 / denom_t1 : 0.0,
					invMassTangent2 = (denom_t2 > 1e-9) ? 1.0 / denom_t2 : 0.0,
					invMassTwist = (denom_twist > 1e-9) ? 1.0 / denom_twist : 0.0,
					accumulatedNormalImpulse = 0.0,
					accumulatedFrictionImpulse1 = 0.0,
					accumulatedFrictionImpulse2 = 0.0,
					accumulatedTwistImpulse = 0.0
				});
			}

			if (constraints.Length == 0)
			{
				constraints.Dispose();
				return;
			}

			// --- Iterative Solver Loop ---
			for (int iter = 0; iter < solverIterations; iter++)
			{
				for (int i = 0; i < constraints.Length; i++)
				{
					ContactConstraint c = constraints[i]; // Get a copy of the constraint
					PhysicsBodyData body = Bodies[c.bodyIndex];
					StateVector state = body.stateVector;

					// --- Solve Velocity Impulses (Bounce & Friction) ---

					// Get current contact velocity
					double3 omega_w = math.rotate(state.orientation, state.angularVelocity);
					double3 v_contact = state.velocity + math.cross(omega_w, c.r_a);
					double v_rel_normal = math.dot(v_contact, c.worldNormal);

					// --- Solve Positional Error (Stops Sinking) ---
					// Calculate Baumgarte stabilization "impulse" to correct penetration
					double bias = (c.penetration - penetrationSlop) * (positionCorrectionFactor / timeStep);
					double j_positional = math.max(0.0, bias) * c.invMassNormal;

					// --- Solve Velocity Error (Bounce) ---
					double effectiveRestitution = (math.abs(v_rel_normal) > restingVelocityThreshold) ? restitution : 0.0;
					double j_velocity = -(1.0 + effectiveRestitution) * v_rel_normal * c.invMassNormal;

					// --- Solve Normal Impulse (Combine & Clamp) ---
					double j_n_old = c.accumulatedNormalImpulse;
					double j_n_new = math.max(0.0, j_n_old + j_positional + j_velocity); // Clamped impulse
					double delta_j_n = j_n_new - j_n_old; // The change to apply
					c.accumulatedNormalImpulse = j_n_new;

					double3 J_normal = delta_j_n * c.worldNormal;

					// --- Solve Linear Friction ---
					double maxStaticFriction = staticFriction * c.accumulatedNormalImpulse; // Static friction limit
					double maxDynamicFriction = dynamicFriction * c.accumulatedNormalImpulse; // Dynamic friction limit
					double3 J_friction = double3.zero;

					// Friction 1 (Tangent 1)
					double v_rel_tan1 = math.dot(v_contact, c.tangent1);
					double j_t1_unclamped = c.accumulatedFrictionImpulse1 - v_rel_tan1 * c.invMassTangent1;

					// Friction 2 (Tangent 2)
					double v_rel_tan2 = math.dot(v_contact, c.tangent2);
					double j_t2_unclamped = c.accumulatedFrictionImpulse2 - v_rel_tan2 * c.invMassTangent2;

					// Clamp to Friction Cone
					double j_t_magnitude_unclamped = math.sqrt(j_t1_unclamped * j_t1_unclamped + j_t2_unclamped * j_t2_unclamped);
					double j_t1_new = j_t1_unclamped;
					double j_t2_new = j_t2_unclamped;

					if (j_t_magnitude_unclamped > maxStaticFriction && j_t_magnitude_unclamped > 1e-9) // If we are not within the cone
					{
						// SLIDING
						// Static friction limit broken. Apply dynamic friction.
						// Scale the uncamped vector to the dynamic friction limit.
						double scale = maxDynamicFriction / j_t_magnitude_unclamped;
						j_t1_new = j_t1_unclamped * scale;
						j_t2_new = j_t2_unclamped * scale;
					}

					// Calculate the delta to apply
					double delta_j_t1 = j_t1_new - c.accumulatedFrictionImpulse1;
					double delta_j_t2 = j_t2_new - c.accumulatedFrictionImpulse2;

					// Store new accumulated impulse
					c.accumulatedFrictionImpulse1 = j_t1_new;
					c.accumulatedFrictionImpulse2 = j_t2_new;

					// Build the final friction impulse vector
					J_friction = delta_j_t1 * c.tangent1 + delta_j_t2 * c.tangent2;

					// --- Solve Torsional (Spin) Friction
					double v_rel_twist = math.dot(omega_w, c.worldNormal);
					double j_twist_old = c.accumulatedTwistImpulse;
					double j_twist_new_unclamped = c.accumulatedTwistImpulse - v_rel_twist * c.invMassTwist;
					double j_twist_new = j_twist_new_unclamped;

					if (math.abs(j_twist_new_unclamped) > maxStaticFriction)
					{
						// SPINNING
						// Static limit broken, apply dynamic limit
						j_twist_new = math.clamp(j_twist_new_unclamped, -maxDynamicFriction, maxDynamicFriction);
					}

					double delta_j_twist = j_twist_new - j_twist_old;
					c.accumulatedTwistImpulse = j_twist_new;

					// --- Apply Total Impulse ---
					double3 J_total = J_normal + J_friction;
					state.velocity += J_total * body.vmod.invMass;

					double3 L_world_linear = math.cross(c.r_a, J_total);
					double3 L_world_twist = delta_j_twist * c.worldNormal; // The twist impulse is an angular impulse directly around the normal
					double3 L_world_total = L_world_linear + L_world_twist;
					EquationsOfMotion.RotateDouble3(math.inverse(state.orientation), L_world_total, out double3 L_body);
					state.angularVelocity += (float3)math.mul(body.vmod.InvInertiaTensor, L_body);

					// --- Write Back State ---
					Bodies[c.bodyIndex] = new PhysicsBodyData(state, body.vmod, body.externalForce, body.externalMoment);
					constraints[i] = c;
				}
			}

			constraints.Dispose();
		}

		// Helper to create an orthonormal basis (tangents) from a normal
		private void CreateBasis(double3 n, out double3 t1, out double3 t2)
		{
			if (math.abs(n.x) >= 0.57735) // 1/sqrt(3)
				t1 = math.normalize(new double3(n.y, -n.x, 0.0));
			else
				t1 = math.normalize(new double3(0.0, n.z, -n.y));

			t2 = math.normalize(math.cross(n, t1));
		}
	}

	[BurstCompile]
	public struct ProcessDynamicCollisionsJob : IJob
	{
		public NativeArray<PhysicsBodyData> Bodies;
		[ReadOnly] public NativeArray<DynamicContactInfo> Contacts;

		[ReadOnly] public double restitution;
		[ReadOnly] public double staticFriction;
		[ReadOnly] public double dynamicFriction;
		[ReadOnly] public float timeStep;

		// Solver settings
		private const int solverIterations = 8;
		private const double penetrationSlop = 0.0;
		private const double positionCorrectionFactor = 0.1;
		private const double restingVelocityThreshold = 0.5;

		// A struct to hold pre-calculated constraint data for TWO bodies
		private struct ContactConstraint
		{
			public int bodyIndexA;
			public int bodyIndexB;
			public double3 r_a; // Lever arm for A (world space)
			public double3 r_b; // Lever arm for B (world space)

			public double3 worldNormal;
			public double penetration;

			// Pre-calculated "effective mass" for normal and tangent
			public double invMassNormal;
			public double invMassTangent1;
			public double invMassTangent2;
			public double invMassTwist;

			// Tangent directions
			public double3 tangent1;
			public double3 tangent2;

			// Total accumulated impulses (for clamping)
			public double accumulatedNormalImpulse;
			public double accumulatedFrictionImpulse1;
			public double accumulatedFrictionImpulse2;
			public double accumulatedTwistImpulse;
		}

		public void Execute()
		{
			// 1. Dequeue all contacts and pre-calculate constraint data
			NativeList<ContactConstraint> constraints = new NativeList<ContactConstraint>(Allocator.Temp);
			for (int i = 0; i < Contacts.Length; i++)
			{
				DynamicContactInfo contact = Contacts[i];

				// if (contact.bodyIndexA == 0 && contact.bodyIndexB == 0)
				// {
				// 	continue;
				// }

				PhysicsBodyData bodyA = Bodies[contact.bodyIndexA];
				PhysicsBodyData bodyB = Bodies[contact.bodyIndexB];
				StateVector stateA = bodyA.stateVector;
				StateVector stateB = bodyB.stateVector;

				double inv_mass_A = bodyA.vmod.invMass;
				double inv_mass_B = bodyB.vmod.invMass;

				// --- Calculate World-Space Inverse Inertia Tensors ---
				float3x3 R_A = new float3x3(stateA.orientation);
				float3x3 R_T_A = math.transpose(R_A);
				float3x3 I_inv_world_A = math.mul(math.mul(R_A, bodyA.vmod.InvInertiaTensor), R_T_A);

				float3x3 R_B = new float3x3(stateB.orientation);
				float3x3 R_T_B = math.transpose(R_B);
				float3x3 I_inv_world_B = math.mul(math.mul(R_B, bodyB.vmod.InvInertiaTensor), R_T_B);

				// Lever arms from center of mass to contact point
				double3 r_a = contact.worldPoint - stateA.position;
				double3 r_b = contact.worldPoint - stateB.position;
				double3 n = (double3)contact.worldNormal;

				CreateBasis(n, out double3 t1, out double3 t2);

				// --- Pre-calculate Effective Mass Denominators (for TWO bodies) ---
				// A simpler, more stable way to calculate angular component: dot(K, I_inv * K)

				// Normal
				double3 K_A_n = math.cross(r_a, n);
				double3 K_B_n = math.cross(r_b, n);
				double ang_comp_A_n = math.dot(math.mul(I_inv_world_A, K_A_n), K_A_n);
				double ang_comp_B_n = math.dot(math.mul(I_inv_world_B, K_B_n), K_B_n);
				double denom_n = inv_mass_A + inv_mass_B + ang_comp_A_n + ang_comp_B_n;

				// Tangent 1
				double3 K_A_t1 = math.cross(r_a, t1);
				double3 K_B_t1 = math.cross(r_b, t1);
				double ang_comp_A_t1 = math.dot(math.mul(I_inv_world_A, K_A_t1), K_A_t1);
				double ang_comp_B_t1 = math.dot(math.mul(I_inv_world_B, K_B_t1), K_B_t1);
				double denom_t1 = inv_mass_A + inv_mass_B + ang_comp_A_t1 + ang_comp_B_t1;

				// Tangent 2
				double3 K_A_t2 = math.cross(r_a, t2);
				double3 K_B_t2 = math.cross(r_b, t2);
				double ang_comp_A_t2 = math.dot(math.mul(I_inv_world_A, K_A_t2), K_A_t2);
				double ang_comp_B_t2 = math.dot(math.mul(I_inv_world_B, K_B_t2), K_B_t2);
				double denom_t2 = inv_mass_A + inv_mass_B + ang_comp_A_t2 + ang_comp_B_t2;

				// Twist (Normal)
				double3 ang_comp_A_twist = math.mul(I_inv_world_A, n);
				double3 ang_comp_B_twist = math.mul(I_inv_world_B, n);
				double denom_twist = math.dot(n, ang_comp_A_twist) + math.dot(n, ang_comp_B_twist);

				constraints.Add(new ContactConstraint
				{
					bodyIndexA = contact.bodyIndexA,
					bodyIndexB = contact.bodyIndexB,
					r_a = r_a,
					r_b = r_b,
					worldNormal = n,
					penetration = contact.penetrationDepth,
					tangent1 = t1,
					tangent2 = t2,
					invMassNormal = (denom_n > 1e-9) ? 1.0 / denom_n : 0.0,
					invMassTangent1 = (denom_t1 > 1e-9) ? 1.0 / denom_t1 : 0.0,
					invMassTangent2 = (denom_t2 > 1e-9) ? 1.0 / denom_t2 : 0.0,
					invMassTwist = (denom_twist > 1e-9) ? 1.0 / denom_twist : 0.0,
					accumulatedNormalImpulse = 0.0,
					accumulatedFrictionImpulse1 = 0.0,
					accumulatedFrictionImpulse2 = 0.0,
					accumulatedTwistImpulse = 0.0
				});
			}

			if (constraints.Length == 0)
			{
				constraints.Dispose();
				return;
			}

			// --- 2. Iterative Solver Loop ---
			for (int iter = 0; iter < solverIterations; iter++)
			{
				for (int i = 0; i < constraints.Length; i++)
				{
					ContactConstraint c = constraints[i];
					PhysicsBodyData bodyA = Bodies[c.bodyIndexA];
					PhysicsBodyData bodyB = Bodies[c.bodyIndexB];
					StateVector stateA = bodyA.stateVector;
					StateVector stateB = bodyB.stateVector;

					// --- Get current RELATIVE contact velocity ---
					double3 omega_w_A = math.rotate(stateA.orientation, stateA.angularVelocity);
					double3 omega_w_B = math.rotate(stateB.orientation, stateB.angularVelocity);

					double3 v_contact_A = stateA.velocity + math.cross(omega_w_A, c.r_a);
					double3 v_contact_B = stateB.velocity + math.cross(omega_w_B, c.r_b);

					// Relative velocity of A with respect to B
					double3 v_rel_contact = v_contact_A - v_contact_B;
					double v_rel_normal = math.dot(v_rel_contact, c.worldNormal);

					// --- A. Solve Positional Error (Stops Sinking) ---
					double bias = (c.penetration - penetrationSlop) * (positionCorrectionFactor / timeStep);
					double j_positional = math.max(0.0, bias) * c.invMassNormal;

					// --- B. Solve Velocity Error (Bounce) ---
					double effectiveRestitution = (math.abs(v_rel_normal) > restingVelocityThreshold) ? restitution : 0.0;
					// We use v_rel_normal here
					double j_velocity = -(1.0 + effectiveRestitution) * v_rel_normal * c.invMassNormal;

					// --- C. Solve Normal Impulse (Combine & Clamp) ---
					double j_n_old = c.accumulatedNormalImpulse;
					double j_n_new = math.max(0.0, j_n_old + j_positional + j_velocity);
					double delta_j_n = j_n_new - j_n_old;
					c.accumulatedNormalImpulse = j_n_new;

					double3 J_normal = delta_j_n * c.worldNormal;

					// --- D. Solve Linear Friction ---
					double maxStaticFriction = staticFriction * c.accumulatedNormalImpulse;
					double maxDynamicFriction = dynamicFriction * c.accumulatedNormalImpulse;

					// Relative sliding velocities
					double v_rel_tan1 = math.dot(v_rel_contact, c.tangent1);
					double v_rel_tan2 = math.dot(v_rel_contact, c.tangent2);

					double j_t1_unclamped = c.accumulatedFrictionImpulse1 - v_rel_tan1 * c.invMassTangent1;
					double j_t2_unclamped = c.accumulatedFrictionImpulse2 - v_rel_tan2 * c.invMassTangent2;

					// Clamp to Friction Cone
					double j_t_magnitude_unclamped = math.sqrt(j_t1_unclamped * j_t1_unclamped + j_t2_unclamped * j_t2_unclamped);
					double j_t1_new = j_t1_unclamped;
					double j_t2_new = j_t2_unclamped;

					if (j_t_magnitude_unclamped > maxStaticFriction && j_t_magnitude_unclamped > 1e-9) // If we are not within the cone
					{
						// SLIDING
						// Static friction limit broken. Apply dynamic friction.
						// Scale the uncamped vector to the dynamic friction limit.
						double scale = maxDynamicFriction / j_t_magnitude_unclamped;
						j_t1_new = j_t1_unclamped * scale;
						j_t2_new = j_t2_unclamped * scale;
					}

					// Calculate the delta to apply
					double delta_j_t1 = j_t1_new - c.accumulatedFrictionImpulse1;
					double delta_j_t2 = j_t2_new - c.accumulatedFrictionImpulse2;

					// Store new accumulated impulse
					c.accumulatedFrictionImpulse1 = j_t1_new;
					c.accumulatedFrictionImpulse2 = j_t2_new;

					// Build the final friction impulse vector
					double3 J_friction = delta_j_t1 * c.tangent1 + delta_j_t2 * c.tangent2;

					// --- D2. Solve Torsional (Spin) Friction ---
					double3 omega_rel_w = omega_w_A - omega_w_B; // Relative angular velocity
					double v_rel_twist = math.dot(omega_rel_w, c.worldNormal);
					double j_twist_old = c.accumulatedTwistImpulse;
					double j_twist_new_unclamped = c.accumulatedTwistImpulse - v_rel_twist * c.invMassTwist;
					double j_twist_new = j_twist_new_unclamped;

					if (math.abs(j_twist_new_unclamped) > maxStaticFriction)
					{
						// SPINNING
						// Static limit broken, apply dynamic limit
						j_twist_new = math.clamp(j_twist_new_unclamped, -maxDynamicFriction, maxDynamicFriction);
					}

					double delta_j_twist = j_twist_new - j_twist_old;
					c.accumulatedTwistImpulse = j_twist_new;

					// --- E. Apply Total Impulse to BOTH bodies ---
					// Impulse is applied to A, and subtracted from B

					// Linear
					double3 J_total_linear = J_normal + J_friction;
					stateA.velocity += J_total_linear * bodyA.vmod.invMass;
					stateB.velocity -= J_total_linear * bodyB.vmod.invMass;

					// Angular
					// Torque on A
					double3 L_world_linear_A = math.cross(c.r_a, J_total_linear);
					double3 L_world_twist_A = delta_j_twist * c.worldNormal;
					double3 L_world_total_A = L_world_linear_A + L_world_twist_A;

					// Torque on B (equal and opposite)
					double3 L_world_linear_B = math.cross(c.r_b, -J_total_linear); // r_b x (-J)
					double3 L_world_twist_B = -delta_j_twist * c.worldNormal; // -J_twist
					double3 L_world_total_B = L_world_linear_B + L_world_twist_B;

					// Apply to A
					EquationsOfMotion.RotateDouble3(math.inverse(stateA.orientation), L_world_total_A, out double3 L_body_A);
					stateA.angularVelocity += (float3)math.mul(bodyA.vmod.InvInertiaTensor, L_body_A);

					// Apply to B
					EquationsOfMotion.RotateDouble3(math.inverse(stateB.orientation), L_world_total_B, out double3 L_body_B);
					stateB.angularVelocity += (float3)math.mul(bodyB.vmod.InvInertiaTensor, L_body_B);

					// --- F. Write Back State for BOTH bodies ---
					Bodies[c.bodyIndexA] = new PhysicsBodyData(stateA, bodyA.vmod, bodyA.externalForce, bodyA.externalMoment);
					Bodies[c.bodyIndexB] = new PhysicsBodyData(stateB, bodyB.vmod, bodyB.externalForce, bodyB.externalMoment);
					constraints[i] = c; // Write back accumulated impulses
				}
			}

			constraints.Dispose();
		}

		// Helper to create an orthonormal basis (tangents) from a normal
		private void CreateBasis(double3 n, out double3 t1, out double3 t2)
		{
			if (math.abs(n.x) >= 0.57735) // 1/sqrt(3)
				t1 = math.normalize(new double3(n.y, -n.x, 0.0));
			else
				t1 = math.normalize(new double3(0.0, n.z, -n.y));

			t2 = math.normalize(math.cross(n, t1));
		}
	}

	// [BurstCompile]
	// public struct PopulateContactsJob : IJobParallelFor
	// {
	// 	// IN: All contacts found this frame (with duplicates)
	// 	[ReadOnly] public NativeArray<DynamicContactInfo> AllContacts;
		
	// 	// OUT: A map where Key = packed pair, Value = ContactInfo
	// 	public NativeParallelMultiHashMap<long, DynamicContactInfo>.ParallelWriter ContactMapWriter;

	// 	public void Execute(int index)
	// 	{
	// 		DynamicContactInfo rawContact = AllContacts[index];
	// 		int indexA = rawContact.bodyIndexA;
	// 		int indexB = rawContact.bodyIndexB;

	// 		// 1. Normalize the pair to ensure (A,B) is the same as (B,A)
	// 		int low, high;
	// 		if (indexA < indexB)
	// 		{
	// 			low = indexA;
	// 			high = indexB;
	// 			rawContact.worldNormal *= -1f;
	// 		}
	// 		else
	// 		{
	// 			// If B < A, we must also flip the contact normal
	// 			low = indexB;
	// 			high = indexA;
	// 		}

	// 		// 2. Pack into a 64-bit long
	// 		long pairKey = ((long)low << 32) | (long)high;

	// 		// 3. Add to the map. This map allows multiple contacts per pair.
	// 		ContactMapWriter.Add(pairKey, rawContact);
	// 	}
	// }

	// [BurstCompile]
	// public struct FilterUniqueContactsJob : IJob
	// {
	// 	// IN: The map populated by Job 1
	// 	[ReadOnly] public NativeParallelMultiHashMap<long, DynamicContactInfo> AllContactsMap;

	// 	// OUT: The final, clean queue for your solver
	// 	public NativeQueue<DynamicContactInfo> FinalUniqueContactsQueue;

	// 	// We'll consider points closer than 1mm "the same"
	// 	// Using squared distance for performance
	// 	private const double EpsilonSq = 0.000001; // (0.001 * 0.001)

	// 	public void Execute()
	// 	{
	// 		// A temp list to hold contacts *per-pair*
	// 		var tempContactList = new NativeList<DynamicContactInfo>(64, Allocator.Temp);

	// 		// A temp list to hold the filtered contacts
	// 		var filteredList = new NativeList<DynamicContactInfo>(64, Allocator.Temp);

	// 		// Get all unique pairs (keys) from the map
	// 		(NativeArray<long> uniqueKeys, int uniqueKeyCount) = AllContactsMap.GetUniqueKeyArray(Allocator.Temp);

	// 		for (int i = 0; i < uniqueKeyCount; i++)
	// 		{
	// 			long pairKey = uniqueKeys[i];

	// 			// Clear lists for this new pair
	// 			tempContactList.Clear();
	// 			filteredList.Clear();

	// 			// Get all contacts for this specific pair
	// 			if (AllContactsMap.TryGetFirstValue(pairKey, out DynamicContactInfo contact, out var it))
	// 			{
	// 				do
	// 				{
	// 					tempContactList.Add(contact);
	// 				}
	// 				while (AllContactsMap.TryGetNextValue(out contact, ref it));
	// 			}

	// 			if (tempContactList.Length == 0)
	// 			{
	// 				continue;
	// 			}

	// 			// --- Filter this pair's contact list ---

	// 			// Add the first contact automatically
	// 			filteredList.Add(tempContactList[0]);

	// 			// Start from the second contact
	// 			for (int j = 1; j < tempContactList.Length; j++)
	// 			{
	// 				DynamicContactInfo newContact = tempContactList[j];
	// 				bool isUnique = true;

	// 				// Compare against the contacts we've already added to the filtered list
	// 				for (int k = 0; k < filteredList.Length; k++)
	// 				{
	// 					DynamicContactInfo addedContact = filteredList[k];

	// 					// Check if the points are too close
	// 					if (math.distancesq(newContact.worldPoint, addedContact.worldPoint) < EpsilonSq)
	// 					{
	// 						isUnique = false;
	// 						break;
	// 					}
	// 				}

	// 				if (isUnique)
	// 				{
	// 					filteredList.Add(newContact);
	// 				}
	// 			}

	// 			// --- Enqueue the unique contacts ---
	// 			// Now add the unique, filtered contacts to the final solver queue
	// 			for (int j = 0; j < filteredList.Length; j++)
	// 			{
	// 				FinalUniqueContactsQueue.Enqueue(filteredList[j]);
	// 			}
	// 		}

	// 		// Clean up temp allocations
	// 		uniqueKeys.Dispose();
	// 		tempContactList.Dispose();
	// 		filteredList.Dispose();
	// 	}
	// }
}