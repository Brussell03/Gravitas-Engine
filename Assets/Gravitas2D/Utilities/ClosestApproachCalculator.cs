using Unity.Mathematics;
using UnityEngine;

namespace Gravitas2D
{
	public struct CloseApproachData
	{
		public bool IsValid;
		public double TimeOfClosestApproach;
		public double Separation;
		public double2 Body1Position;
		public double2 Body1Velocity;
		public double2 Body2Position;
		public double2 Body2Velocity;
	}

	public static class CloseApproachCalculator
	{
		// Pre-computed Golden Ratio constants for search optimization
		private static readonly double Phi = (1.0 + math.sqrt(5.0)) / 2.0;
		private static readonly double ResPhi = 2.0 - Phi;

		/// <summary>
		/// Calculates the minimum separation between two bodies within the same SoI.
		/// Returns IsValid = false if the closest approach exceeds the defined threshold.
		/// </summary>
		public static CloseApproachData Calculate(double2 b1Pos, double2 b1Vel, double b1Mass, double2 b2Pos, double2 b2Vel, double b2Mass, PhysicsBody parent, double minSeparationThreshold = double.MaxValue, int coarseSteps = 200)
		{
			double2 parentPos = double2.zero;
			double2 parentVel = double2.zero;
			double parentMass = 0;
			if (parent != null)
			{
				parentPos = parent.Position;
				parentVel = parent.Velocity;
				parentMass = parent.Mass;
			}

			// 1. Establish foundational orbital metrics to determine evaluation timeframe
			double mu1 = OrbitalMath.GetStandardGravitationalParameter(parentMass, b1Mass);
			double mu2 = OrbitalMath.GetStandardGravitationalParameter(parentMass, b2Mass);

			double r1 = math.distance(b1Pos, parentPos);
			double r2 = math.distance(b2Pos, parentPos);
			
			double vSq1 = math.lengthsq(b1Vel - parentVel);
			double vSq2 = math.lengthsq(b2Vel - parentVel);

			double a1 = OrbitalMath.CalculateSemiMajorAxis(r1, vSq1, mu1);
			double a2 = OrbitalMath.CalculateSemiMajorAxis(r2, vSq2, mu2);

			double period1 = OrbitalMath.CalculateOrbitalPeriod(a1, mu1);
			double period2 = OrbitalMath.CalculateOrbitalPeriod(a2, mu2);

			// 1. Relative motion metrics for hypervelocity and open/hyperbolic trajectory handling
			double2 relPos = b2Pos - b1Pos;
			double2 relVel = b2Vel - b1Vel;
			double relSpeedSq = math.lengthsq(relVel);
			
			double linearClosestTime = 0.0;
			if (relSpeedSq > 1e-9)
			{
				linearClosestTime = -math.dot(relPos, relVel) / relSpeedSq;
			}

			// 2. Adaptive time horizon calculation for closed vs open/hypervelocity orbits
			double searchHorizon = 0.0;
			if (!double.IsInfinity(period1) && period1 > 0) searchHorizon = math.max(searchHorizon, period1);
			if (!double.IsInfinity(period2) && period2 > 0) searchHorizon = math.max(searchHorizon, period2);

			double dist = math.length(relPos);
			double relSpeed = math.sqrt(relSpeedSq);
			double dynamicHorizon = relSpeed > 1e-6 ? (dist / relSpeed) * 4.0 : 1000.0;
			if (linearClosestTime > 0)
			{
				dynamicHorizon = math.max(dynamicHorizon, linearClosestTime * 2.0);
			}

			if (searchHorizon <= 0 || double.IsInfinity(searchHorizon))
			{
				searchHorizon = math.max(1000.0, dynamicHorizon);
			}
			else
			{
				searchHorizon = math.max(searchHorizon, dynamicHorizon);
			}

			// 3. Coarse Search incorporating linear intercept estimation
			double timeStep = searchHorizon / coarseSteps;
			double bestT = 0;
			double minSeparation = double.MaxValue;

			for (int i = 0; i <= coarseSteps; i++)
			{
				double t = i * timeStep;
				double sep = OrbitalMath.GetSeparationAtTime(b1Pos, b1Vel, b2Pos, b2Vel, parent, t, mu1, mu2);
				if (sep < minSeparation)
				{
					minSeparation = sep;
					bestT = t;
				}
			}

			// Explicitly sample linear closest time if within bounds to prevent coarse-step skipping
			if (linearClosestTime > 0 && linearClosestTime <= searchHorizon)
			{
				double sepLin = OrbitalMath.GetSeparationAtTime(b1Pos, b1Vel, b2Pos, b2Vel, parent, linearClosestTime, mu1, mu2);
				if (sepLin < minSeparation)
				{
					minSeparation = sepLin;
					bestT = linearClosestTime;
				}
			}

			// 4. Fine Optimization via Golden Section Search
			double bracketWidth = math.max(timeStep, searchHorizon * 0.05);
			double tMin = math.max(0, bestT - bracketWidth);
			double tMax = math.min(searchHorizon, bestT + bracketWidth);
			double tolerance = 1e-5;

			double a = tMin;
			double b = tMax;
			if (a >= b)
			{
				b = math.min(searchHorizon, a + 1.0);
			}

			double c = a + ResPhi * (b - a);
			double d = b - ResPhi * (b - a);
			double fc = OrbitalMath.GetSeparationAtTime(b1Pos, b1Vel, b2Pos, b2Vel, parent, c, mu1, mu2);
			double fd = OrbitalMath.GetSeparationAtTime(b1Pos, b1Vel, b2Pos, b2Vel, parent, d, mu1, mu2);

			int maxIterations = 50;
			int iter = 0;
			while (math.abs(c - d) > tolerance && iter < maxIterations)
			{
				if (fc < fd)
				{
					b = d;
					d = c;
					fd = fc;
					c = a + ResPhi * (b - a);
					fc = OrbitalMath.GetSeparationAtTime(b1Pos, b1Vel, b2Pos, b2Vel, parent, c, mu1, mu2);
				}
				else
				{
					a = c;
					c = d;
					fc = fd;
					d = b - ResPhi * (b - a);
					fd = OrbitalMath.GetSeparationAtTime(b1Pos, b1Vel, b2Pos, b2Vel, parent, d, mu1, mu2);
				}
				iter++;
			}

			double finalTime = (a + b) / 2.0;
			
			// 5. Final State Extraction
			OrbitalMath.GetLocalStateAtTime(b1Pos, b1Vel, parent, finalTime, mu1, out double2 p1, out double2 v1);
			OrbitalMath.GetLocalStateAtTime(b2Pos, b2Vel, parent, finalTime, mu2, out double2 p2, out double2 v2);
			
			double finalSeparation = math.distance(p1, p2);

			if (finalSeparation > minSeparationThreshold)
			{
				return new CloseApproachData { IsValid = false };
			}

			return new CloseApproachData
			{
				IsValid = true,
				TimeOfClosestApproach = finalTime,
				Separation = finalSeparation,
				Body1Position = p1 + parent.Position,
				Body1Velocity = v1 + parent.Velocity,
				Body2Position = p2 + parent.Position,
				Body2Velocity = v2 + parent.Velocity
			};
		}

		/// <summary>
		/// Calculates the minimum separation between two bodies within the same SoI.
		/// Returns IsValid = false if the closest approach exceeds the defined threshold.
		/// </summary>
		public static CloseApproachData Calculate(PhysicsBody b1, PhysicsBody b2, double minSeparationThreshold = double.MaxValue, int coarseSteps = 100)
		{
			// Input Validation and System integrity checks
			if (b1 == null || b2 == null || b1.IsDestroyed || b2.IsDestroyed) return new CloseApproachData { IsValid = false };
			if (b1.ParentSoI != b2.ParentSoI) return new CloseApproachData { IsValid = false };

			PhysicsBody parent = b1.ParentSoI;

			return Calculate(b1.Position, b1.Velocity, b1.Mass, b2.Position, b2.Velocity, b2.Mass, parent, minSeparationThreshold, coarseSteps);
		}
	}
}