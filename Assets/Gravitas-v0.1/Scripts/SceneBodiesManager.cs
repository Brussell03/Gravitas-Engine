using UnityEngine;
using UnityEngine.Pool;

namespace Gravitas
{
	public class SceneBodiesManager
	{
		IObjectPool<PhysicsBodyMono> sceneBodiesPool;
		PhysicsBodyMono bodyPrefab;

		// throw an exception if we try to return an existing item, already in the pool
		private bool collectionCheck = false;

		// extra options to control the pool capacity and maximum size
		private int defaultCapacity = 1000;
		private int maxSize = 50000;

		public SceneBodiesManager(PhysicsBodyMono bodyPrefab)
		{
			this.bodyPrefab = bodyPrefab;
			sceneBodiesPool = new ObjectPool<PhysicsBodyMono>(CreateBody, OnGetFromPool, OnReleaseToPool, OnDestroyPooledObject, collectionCheck, defaultCapacity, maxSize);
		}

		public PhysicsBodyMono GetBody(in StateVector startState, VehicleModel vmod, bool isKinematic, bool isCelestial, double radius = 1.0, string name = null)
		{
			var body = sceneBodiesPool.Get();
			body.InitializeBody(startState, vmod, isKinematic, isCelestial, radius, name);

			return body;
		}

		PhysicsBodyMono CreateBody()
		{
			PhysicsBodyMono bodyInstance = PhysicsManager.InstantiateBody(bodyPrefab);
			bodyInstance.ObjectPool = sceneBodiesPool;

			return bodyInstance;
		}

		void OnGetFromPool(PhysicsBodyMono pooledObject)
		{
			pooledObject.gameObject.SetActive(true);
		}

		void OnReleaseToPool(PhysicsBodyMono pooledObject)
		{
			pooledObject.DeinitializeBody();
			pooledObject.gameObject.SetActive(false);
		}

		void OnDestroyPooledObject(PhysicsBodyMono pooledObject)
		{
			pooledObject.DeinitializeBody();
			PhysicsManager.DestroyBody(pooledObject.gameObject);
		}
	}
}