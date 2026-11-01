using UnityEngine;
using Unity.Mathematics;
using System.Runtime.InteropServices;

namespace Gravitas
{
	[System.Serializable, StructLayout(LayoutKind.Sequential)]
	public struct StateVector
	{
		public double3 position;
		public double3 velocity;
		public quaternion orientation;	// Orientation represented as a quaternion
		public float3 angularVelocity;  // Angular velocity vector (rad/s)

		public StateVector(double3 position, double3 velocity, quaternion orientation, float3 angularVelocity)
		{
			this.position = position;
			this.velocity = velocity;
			this.orientation = orientation;
			this.angularVelocity = angularVelocity;
		}
		
		public StateVector(Transform transform)
		{
			position = new double3(transform.position);
			velocity = double3.zero;
			orientation = transform.rotation;
			angularVelocity = float3.zero;
		}
		
		public override string ToString()
		{
			return $"Position: {position}, Velocity: {velocity}, Orientation: {orientation}, Angular Velocity: {angularVelocity}";
		}
	}

	// This class holds the derivative vectors
	[StructLayout(LayoutKind.Sequential)]
	public struct StateDerivative
	{
		public double3 velocity;			// d(position)/dt
		public double3 acceleration;		// d(velocity)/dt
		public float3 angularVelocity;		// d(orientation)/dt (represented differently for quaternions)
		public float3 angularAcceleration;	// d(angularVelocity)/dt
	}

	[System.Serializable, StructLayout(LayoutKind.Sequential)]
	public struct VehicleModel
	{
		public float mass;
		public float3x3 InertiaTensor;
		public float3 linearDamping;
		public float3 angularDamping;
		[HideInInspector] public float3x3 InvInertiaTensor;
		[HideInInspector] public float invMass;

		public void Initialize()
		{
			InvInertiaTensor = math.inverse(InertiaTensor);
			invMass = 1.0f / mass;
		}
	}
	
	// [System.Serializable, StructLayout(LayoutKind.Sequential)]
	// public struct AtmosphericModel
	// {
	// 	public double planetRadius;
	// 	public double mu;
	// 	public float density;
	// }
	

	[StructLayout(LayoutKind.Sequential)]
	public struct StaticContactInfo
	{
		public int bodyIndex;
		public float3 worldNormal;
		public float3 worldPoint;
		public float penetrationDepth;
	}

	[StructLayout(LayoutKind.Sequential)]
	public struct DynamicContactInfo
	{
		public int bodyIndexA;
		public int bodyIndexB;
		public float3 worldNormal;
		public double3 worldPoint;
		public float penetrationDepth;
	}
}
