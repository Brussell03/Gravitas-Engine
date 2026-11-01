using Gravitas;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.Pool;

namespace Gravitas
{
	[RequireComponent(typeof(Rigidbody))]
	public class PhysicsBodyMono : MonoBehaviour
	{
		// public StateVector initialState;
		// public VehicleModel vmod;
		// public bool isKinematic;

		Rigidbody rb;
		PhysicsBody physicsBody;
		IObjectPool<PhysicsBodyMono> objectPool;

		// public property to give the projectile a reference to its ObjectPool
        public IObjectPool<PhysicsBodyMono> ObjectPool { set => objectPool = value; }

		void Awake()
		{
			rb = GetComponent<Rigidbody>();
			rb.useGravity = false;
			rb.isKinematic = true;
		}

		public void InitializeBody(in StateVector startState, VehicleModel vmod, bool isKinematic, bool isCelestial, double radius = 1.0, string name = null)
		{
			//transform.position = (float3)PhysicsManager.GetScenePosition(startState.position);
			physicsBody = new PhysicsBody(startState, vmod, isKinematic, isCelestial, name, rb);
			this.name = physicsBody.name;
			SetMesh(isCelestial, (float)radius);

			var collisionHandler = transform.AddComponent<PhysicsCollisionHandler>();
			collisionHandler.SetPhysicsBody(physicsBody);
		}

		public void DeinitializeBody()
		{
			physicsBody.Remove();
		}

		public void Deactivate()
		{
			objectPool.Release(this);
		}

		void SetMesh(bool isSphere, float radius)
		{
			MeshFilter meshFilter = GetComponent<MeshFilter>();

			Mesh mesh;

			if (isSphere)
			{
				GameObject tempSphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
				mesh = tempSphere.GetComponent<MeshFilter>().sharedMesh;
				Destroy(tempSphere);
				DestroyImmediate(GetComponent<BoxCollider>());
				transform.AddComponent<SphereCollider>();
			}
			else
			{
				GameObject tempCube = GameObject.CreatePrimitive(PrimitiveType.Cube);
				mesh = tempCube.GetComponent<MeshFilter>().sharedMesh;
				Destroy(tempCube);
			}

			meshFilter.mesh = mesh;

			var meshRenderer = transform.GetComponent<MeshRenderer>();
			meshRenderer.material = Resources.Load<Material>("TestMaterial");

			float scale = (float)radius * 2;
			transform.localScale = new Vector3(scale, scale, scale);
		}
	}
}
