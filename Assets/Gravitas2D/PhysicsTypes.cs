using UnityEngine;
using Unity.Mathematics;
using System;
using System.Runtime.InteropServices;
using Unity.Burst;

namespace Gravitas2D
{
	[Serializable, StructLayout(LayoutKind.Sequential)]
	public struct StateVector
	{
		public double2 Position;
		public double2 Velocity;
		public float Orientation;
		public float AngularVelocity;

		public StateVector(double2 position, double2 velocity, float orientation, float angularVelocity)
		{
			Position = position;
			Velocity = velocity;
			Orientation = orientation;
			AngularVelocity = angularVelocity;
		}

		public StateVector(Transform transform)
		{
			Position = new double2(transform.position.x, transform.position.y);
			Velocity = double2.zero;
			Orientation = transform.rotation.eulerAngles.z * math.TORADIANS;
			AngularVelocity = 0f;
		}

		public override string ToString()
		{
			return $"Position: {Position}, Velocity: {Velocity}, Orientation: {Orientation}, Angular Velocity: {AngularVelocity}";
		}
	}

	[StructLayout(LayoutKind.Sequential)]
	public struct StateDerivative
	{
		public double2 Velocity;			// d(position)/dt
		public double2 Acceleration;		// d(velocity)/dt
		public float AngularVelocity;		// d(orientation)/dt
		public float AngularAcceleration;	// d(angularVelocity)/dt
	}

	[Serializable, StructLayout(LayoutKind.Sequential)]
	public struct VehicleModel
	{
		public double Mass;
		public double Radius;
		public double InertiaTensor;
		public float2 LinearDamping;
		public float AngularDamping;
		
		[HideInInspector] public double InvInertiaTensor;
		[HideInInspector] public double InvMass;

		public VehicleModel(double mass, double radius, double inertiaTensor = 0, float2 linearDamping = default, float angularDamping = 0)
		{
			Mass = mass;
			Radius = radius;
			InertiaTensor = inertiaTensor == 0 ? 0.4 * mass * radius * radius : inertiaTensor;
			LinearDamping = linearDamping;
			AngularDamping = angularDamping;
			
			// Input validation to prevent divide-by-zero errors in physics calculations
			InvInertiaTensor = InertiaTensor <= 0 ? 0 : 1.0 / InertiaTensor;
			InvMass = Mass <= 0 ? 0 : 1.0 / Mass;
		}

		public static VehicleModel CreateFromMass(double radius, double mass)
		{
			return new VehicleModel(mass, radius);
		}

		public static VehicleModel CreateFromComposition(double radius, int compositionIndex)
		{
			double volume = 4.0 / 3.0 * math.PI * math.pow(radius, 3);
			double density = PhysicsManager2D.Instance.CompositionProfiles[compositionIndex].Density;
			double mass = volume * density;
			return new VehicleModel(mass, radius);
		}
	}

	[Serializable, StructLayout(LayoutKind.Sequential)]
	[BurstCompile]
	public struct Bounds2D
	{
		public double MinX;
		public double MaxX;
		public double MinY;
		public double MaxY;

		public double2 Size => new double2(MaxX - MinX, MaxY - MinY);
		public double2 Center => new double2((MaxX - MinX) * 0.5 + MinX, (MaxY - MinY) * 0.5 + MinY);

		/// <summary>
		/// Initializes a new 2D bounding area.
		/// </summary>
		public Bounds2D(double minX, double maxX, double minY, double maxY)
		{
			MinX = minX;
			MaxX = maxX;
			MinY = minY;
			MaxY = maxY;
		}

		public Bounds2D(double2 center, double halfWidth, double halfHeight)
		{
			MinX = center.x - halfWidth;
			MaxX = center.x + halfWidth;
			MinY = center.y - halfHeight;
			MaxY = center.y + halfHeight;
		}

		public Bounds2D(double2 center, double radius)
		{
			MinX = center.x - radius;
			MaxX = center.x + radius;
			MinY = center.y - radius;
			MaxY = center.y + radius;
		}

		/// <summary>
		/// Evaluates whether a circular body centered at 'position' with 'radius' intersects or falls inside the bounds.
		/// </summary>
		[BurstCompile]
		public bool IsWithinBounds(double2 position, double radius = 0.0)
		{
			return (position.x + radius) >= MinX && (position.x - radius) <= MaxX &&
			       (position.y + radius) >= MinY && (position.y - radius) <= MaxY;
		}

		/// <summary>
		/// Translates the bounding box by physical units.
		/// </summary>
		public void Translate(double deltaX, double deltaY)
		{
			MinX += deltaX;
			MaxX += deltaX;
			MinY += deltaY;
			MaxY += deltaY;
		}

		/// <summary>
		/// Moves the center of the bounds to another position.
		/// </summary>
		public void MoveTo(double2 pos)
		{
			double2 delta = pos - Center;

			MinX += delta.x;
			MaxX += delta.x;
			MinY += delta.y;
			MaxY += delta.y;
		}

		/// <summary>
		/// Scales the bounding box around its center point.
		/// </summary>
		public void Scale(double factor, double minDimensionLimit)
		{
			double2 center = Center;
			
			double newWidth = math.max(minDimensionLimit, Size.x * factor);
			double newHeight = math.max(minDimensionLimit, Size.y * factor);

			MinX = center.x - (newWidth * 0.5);
			MaxX = center.x + (newWidth * 0.5);
			MinY = center.y - (newHeight * 0.5);
			MaxY = center.y + (newHeight * 0.5);
		}
	}

	[Serializable, StructLayout(LayoutKind.Sequential)]
	public struct PhysicsRankData
	{
		public double MinGravity;
		public int TickInterval;
	}
}