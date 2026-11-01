using Unity.Mathematics;
using UnityEngine;

namespace Gravitas2D
{
	public static class OrbitalMath
	{
		#region Sphere of Influence Calculations

		/// <summary>
		/// Returns the lowest-level celestial PhysicsBody whose Sphere of Influence (SoI) currently contains the target position.
		/// </summary>
		public static PhysicsBody GetDominantSoI(double2 position, int index = -1)
		{
			if (!PhysicsManager2D.Instance.IsInitialized) return null;

			PhysicsBody currentSoIBody = null;
			double smallestSoIRadius = double.MaxValue;

			var celestialIndices = GravitySources.GravitationalBodyIndices;

			for (int i = 0; i < celestialIndices.Length; i++)
			{
				int celestialIndex = celestialIndices[i];
				
				// A celestial cannot be in its own SoI
				if (celestialIndex == index) continue;

				PhysicsBody celestialBody = PhysicsManager2D.Instance.PhysicsBodies[celestialIndex];

				// Ignore bodies that are flagged for cleanup or improperly initialized
				if (celestialBody == null || celestialBody.IsDestroyed) continue;

				int parentIndex = -1;
				double highestInfluence = 0;

				// 1. Identify this celestial's parent body to calculate its SoI boundary
				for (int j = 0; j < celestialIndices.Length; j++)
				{
					int potentialParentIndex = celestialIndices[j];
					if (potentialParentIndex == celestialIndex) continue;

					PhysicsBody parentBody = PhysicsManager2D.Instance.PhysicsBodies[potentialParentIndex];

					// Skip destroyed potential parents
					if (parentBody == null || parentBody.IsDestroyed) continue;

					// Parent must be significantly more massive to define an orbital hierarchy
					if (parentBody.Mass > celestialBody.Mass * 2.0)
					{
						double distSq = math.distancesq(celestialBody.Position, parentBody.Position);
						if (distSq <= 0) continue;

						// Gravitational influence proxy: M / r^2
						double influence = parentBody.Mass / distSq;
						if (influence > highestInfluence)
						{
							highestInfluence = influence;
							parentIndex = potentialParentIndex;
						}
					}
				}

				double soiRadius = double.MaxValue; // Defaults to infinite for the system's primary star

				// 2. Calculate the Laplace Sphere of Influence if a parent exists
				if (parentIndex != -1)
				{
					PhysicsBody parentBody = PhysicsManager2D.Instance.PhysicsBodies[parentIndex];
					
					// Calculate theta: the angle between the Parent->Celestial vector and Celestial->Target vector
					double2 relPosParent = celestialBody.Position - parentBody.Position;
					double2 relPosTarget = position - celestialBody.Position;

					double theta = 0.0;
					
					// Validate vectors to prevent math.atan2(0,0) generating NaN
					if (math.lengthsq(relPosParent) > 1e-10 && math.lengthsq(relPosTarget) > 1e-10)
					{
						double baseAngle = math.atan2(relPosParent.y, relPosParent.x);
						double targetAngle = math.atan2(relPosTarget.y, relPosTarget.x);
						theta = targetAngle - baseAngle;
					}

					soiRadius = CalculateSoIRadius(celestialBody, parentBody);
				}

				// 3. Check if the target body falls within this calculated SoI
				double distanceToTarget = math.distance(position, celestialBody.Position);

				if (distanceToTarget <= soiRadius)
				{
					// 4. Prioritize the smallest SoI to guarantee the lowest hierarchical level.
					// The null check guarantees the primary star (infinite SoI) isn't rejected by double.MaxValue < double.MaxValue.
					if (currentSoIBody == null || soiRadius < smallestSoIRadius)
					{
						smallestSoIRadius = soiRadius;
						currentSoIBody = celestialBody;
					}
				}
			}

			return currentSoIBody;
		}

		/// <summary>
		/// Returns the lowest-level celestial PhysicsBody whose Sphere of Influence (SoI) currently contains the target body.
		/// </summary>
		public static PhysicsBody GetDominantSoI(PhysicsBody targetBody)
		{
			if (targetBody == null || !PhysicsManager2D.Instance.IsInitialized || targetBody.Index <= -1) return null;
			
			return GetDominantSoI(targetBody.Position, targetBody.Index);
		}

		/// <summary>
		/// Returns the lowest-level celestial PhysicsBody whose Sphere of Influence (SoI) currently contains the target index.
		/// </summary>
		public static PhysicsBody GetDominantSoI(int targetBodyIndex)
		{
			if (!PhysicsManager2D.Instance.IsInitialized) return null;
			if (targetBodyIndex <= -1 || targetBodyIndex >= PhysicsManager2D.Instance.PhysicsBodies.Count) return null;

			return GetDominantSoI(PhysicsManager2D.Instance.PhysicsBodies[targetBodyIndex]);
		}

		/// <summary>
		/// Checks if a target position has exceeded the Laplace sphere boundary of its current SoI.
		/// </summary>
		public static bool HasExitedSoI(double2 position, PhysicsBody currentSoI, double2 soiPos)
		{
			// A body cannot exit the primary system body's SoI (e.g., the central star)
			if (currentSoI == null || currentSoI.ParentSoI == null) return false;

			double soiRadius = currentSoI.GetSoIRadius();
			double distanceToSoICenter = math.distance(position, soiPos);

			return distanceToSoICenter > soiRadius;
		}

		/// <summary>
		/// Checks if a target body has exceeded the Laplace sphere boundary of its current SoI.
		/// </summary>
		public static bool HasExitedSoI(PhysicsBody target, PhysicsBody currentSoI)
		{
			return HasExitedSoI(target.Position, currentSoI, currentSoI.Position);
		}

		/// <summary>
		/// Checks if a target position has entered the Laplace sphere of any immediate child celestials of its current SoI.
		/// </summary>
		public static bool CheckEnteredSubSoIAtTime(double2 position, PhysicsBody currentSoI, double time, out PhysicsBody enteredSoI)
		{
			enteredSoI = null;
			if (currentSoI == null) return false;

			// Iterate exclusively through immediate child bodies to avoid O(N) global checks
			foreach (PhysicsBody childCelestial in currentSoI.ChildCelestials)
			{
				GetLocalPosAtTime(childCelestial, currentSoI, time, childCelestial.GravParam, out double2 celestialPositionAtTime);
				celestialPositionAtTime += currentSoI.Position;

				double soiRadius = childCelestial.GetSoIRadius();
				double2 relPosTarget = position - celestialPositionAtTime;
				double distanceToChild = math.length(relPosTarget);

				if (distanceToChild <= soiRadius)
				{
					enteredSoI = childCelestial;
					return true;
				}
			}

			return false;
		}

		/// <summary>
		/// Checks if a target body has entered the Laplace sphere of any immediate child celestials of its current SoI.
		/// </summary>
		public static PhysicsBody CheckEnteredSubSoI(PhysicsBody target, PhysicsBody currentSoI)
		{
			if (target == null || currentSoI == null) return null;

			// Iterate exclusively through immediate child bodies to avoid O(N) global checks
			foreach (PhysicsBody childCelestial in currentSoI.ChildCelestials)
			{
				if (childCelestial.Index == target.Index) continue;

				// Derive directional angle for accurate entry checking
				double2 relPosParent = childCelestial.Position - currentSoI.Position;
				double2 relPosTarget = target.Position - childCelestial.Position;
				
				double theta = 0.0;
				if (math.lengthsq(relPosParent) > 1e-10 && math.lengthsq(relPosTarget) > 1e-10)
				{
					double baseAngle = math.atan2(relPosParent.y, relPosParent.x);
					double targetAngle = math.atan2(relPosTarget.y, relPosTarget.x);
					theta = targetAngle - baseAngle;
				}

				double soiRadius = childCelestial.GetSoIRadius();
				double distanceToChild = math.length(relPosTarget);

				if (distanceToChild <= soiRadius)
				{
					return childCelestial;
				}
			}

			return null;
		}

		/// <summary>
		/// Calculates the Laplace Sphere of Influence radius for a body relative to a parent.
		/// </summary>
		public static double CalculateSoIRadius(PhysicsBody body, PhysicsBody parentBody)
		{
			if (body == null || parentBody == null || parentBody.Mass <= 0.0) return double.MaxValue;

			double2 relPos = body.Position - parentBody.Position;
			double2 relVel = body.Velocity - parentBody.Velocity;
			double dist = math.length(relPos);
			double speedSq = math.lengthsq(relVel);
			double mu = GetStandardGravitationalParameter(parentBody.Mass, body.Mass);

			double semiMajorAxis = CalculateSemiMajorAxis(dist, speedSq, mu);
			
			// Fall back to current distance if trajectory is open or hyperbolic
			if (double.IsNaN(semiMajorAxis) || double.IsInfinity(semiMajorAxis) || semiMajorAxis <= 0.0)
			{
				semiMajorAxis = dist;
			}

			double massRatio = body.Mass / parentBody.Mass;
			return semiMajorAxis * math.pow(massRatio, 0.4); // (m / M)^(2/5)
		}

		/// <summary>
		/// Calculates the Laplace Sphere of Influence radius for a body relative to its parent.
		/// </summary>
		public static double CalculateSoIRadius(PhysicsBody body)
		{
			var parentBody = body.ParentSoI;
			if (body == null || parentBody == null || parentBody.Mass <= 0.0) return double.MaxValue;

			double2 relPos = body.Position - parentBody.Position;
			double2 relVel = body.Velocity - parentBody.Velocity;
			double dist = math.length(relPos);
			double speedSq = math.lengthsq(relVel);
			double mu = body.GravParam;

			double semiMajorAxis = CalculateSemiMajorAxis(dist, speedSq, mu);
			
			// Fall back to current distance if trajectory is open or hyperbolic
			if (double.IsNaN(semiMajorAxis) || double.IsInfinity(semiMajorAxis) || semiMajorAxis <= 0.0)
			{
				semiMajorAxis = dist;
			}

			double massRatio = body.Mass / parentBody.Mass;
			return semiMajorAxis * math.pow(massRatio, 0.4); // (m / M)^(2/5)
		}

		#endregion

		#region Keplerian Orbital Mechanics
		
		/// <summary>
		/// Standard gravitational parameter (mu).
		/// </summary>
		public static double GetStandardGravitationalParameter(double parentMass, double bodyMass)
		{
			if (parentMass <= 0) return 0;
			return GravitySources.GravitationalConstant * (parentMass + bodyMass);
		}

		/// <summary>
		/// Specific relative angular momentum (h) for a 2D orbit.
		/// Calculated as the 2D cross product of relative position and relative velocity.
		/// </summary>
		public static double CalculateSpecificAngularMomentum(double2 relativePosition, double2 relativeVelocity)
		{
			return (relativePosition.x * relativeVelocity.y) - (relativePosition.y * relativeVelocity.x);
		}

		/// <summary>
		/// Calculates the semi-major axis (a) using the Vis-Viva equation.
		/// Negative result indicates a hyperbolic trajectory.
		/// </summary>
		public static double CalculateSemiMajorAxis(double relativeDist, double velocitySq, double mu)
		{
			if (relativeDist <= 0 || mu <= 0) return double.NaN;
			
			double specificEnergy = (velocitySq / 2.0) - (mu / relativeDist);
			
			if (math.abs(specificEnergy) < 1e-10) return double.PositiveInfinity; // Parabolic escape
			
			return -mu / (2.0 * specificEnergy);
		}

		/// <summary>
		/// Calculates the eccentricity vector (e) which points toward periapsis.
		/// Magnitude represents the orbital eccentricity.
		/// </summary>
		public static double2 CalculateEccentricityVector(double2 relativePosition, double2 relativeVelocity, double mu)
		{
			if (mu <= 0) return double2.zero;

			double radius = math.length(relativePosition);
			if (radius <= 0) return double2.zero;

			double vSq = math.lengthsq(relativeVelocity);
			double radialVel = math.dot(relativePosition, relativeVelocity);

			// e = ((v^2 - mu/r) * r - (r dot v) * v) / mu
			double2 term1 = relativePosition * (vSq - (mu / radius));
			double2 term2 = relativeVelocity * radialVel;

			return (term1 - term2) / mu;
		}

		/// <summary>
		/// Derives Apoapsis and Periapsis distances from semi-major axis and eccentricity magnitude.
		/// </summary>
		public static void CalculateApsides(double semiMajorAxis, double eccentricity, out double periapsis, out double apoapsis)
		{
			if (semiMajorAxis <= 0) 
			{
				// Hyperbolic or Parabolic
				periapsis = math.abs(semiMajorAxis) * (eccentricity - 1.0);
				apoapsis = double.PositiveInfinity;
				return;
			}

			periapsis = semiMajorAxis * (1.0 - eccentricity);
			apoapsis = semiMajorAxis * (1.0 + eccentricity);
		}

		/// <summary>
		/// Calculates the time required to complete one full orbit.
		/// Returns infinity if the orbit is open (parabolic/hyperbolic).
		/// </summary>
		public static double CalculateOrbitalPeriod(double semiMajorAxis, double mu)
		{
			if (mu <= 0 || semiMajorAxis <= 0) return double.PositiveInfinity;

			return 2.0 * math.PI * math.sqrt(math.pow(semiMajorAxis, 3.0) / mu);
		}

		/// <summary>
		/// Calculates the velocity required to achieve a parabolic escape trajectory from a given distance.
		/// </summary>
		public static double CalculateEscapeVelocity(double radius, double mu)
		{
			if (radius <= 0 || mu <= 0) return 0;
			return math.sqrt(2.0 * mu / radius);
		}

		/// <summary>
		/// Calculates the velocity required to maintain a perfect circular orbit at a given distance.
		/// </summary>
		public static double CalculateCircularOrbitVelocity(double radius, double mu)
		{
			if (radius <= 0 || mu <= 0) return 0;
			return math.sqrt(mu / radius);
		}

		public static double GetSeparationAtTime(PhysicsBody b1, PhysicsBody b2, PhysicsBody parent, double t, double mu1, double mu2)
		{
			if (b1 == null || b1.IsDestroyed || b2 == null || b2.IsDestroyed || parent == null || parent.IsDestroyed) return double.MaxValue;
			GetLocalPosAtTime(b1.Position, b1.Velocity, parent.Position, parent.Velocity, t, mu1, out double2 p1);
			GetLocalPosAtTime(b2.Position, b2.Velocity, parent.Position, parent.Velocity, t, mu2, out double2 p2);
			return math.distance(p1, p2);
		}

		public static double GetSeparationAtTime(double2 b1Pos, double2 b1Vel, double2 b2Pos, double2 b2Vel, PhysicsBody parent, double t, double mu1, double mu2)
		{
			if (parent == null || parent.IsDestroyed) return double.MaxValue;
			GetLocalPosAtTime(b1Pos, b1Vel, parent.Position, parent.Velocity, t, mu1, out double2 p1);
			GetLocalPosAtTime(b2Pos, b2Vel, parent.Position, parent.Velocity, t, mu2, out double2 p2);
			return math.distance(p1, p2);
		}

		/// <summary>
		/// Universal Keplerian propagator. Derives exact cartesian state vectors analytically 
		/// at time t across all conic sections using Universal Variables.
		/// </summary>
		public static void GetLocalStateAtTime(PhysicsBody body, PhysicsBody parent, double t, double mu, out double2 pos, out double2 vel)
		{
			GetLocalStateAtTime(body.Position, body.Velocity, parent, t, mu, out pos, out vel);
		}

		/// <summary>
		/// Universal Keplerian propagator. Derives exact cartesian state vectors analytically
		/// at time t across all conic sections (elliptical, parabolic, hyperbolic) using Universal Variables.
		/// </summary>
		public static void GetLocalStateAtTime(double2 bodyPos, double2 bodyVel, PhysicsBody parent, double t, double mu, out double2 pos, out double2 vel)
		{
			double2 parentPos = parent != null ? parent.Position : double2.zero;
			double2 parentVel = parent != null ? parent.Velocity : double2.zero;
			GetLocalStateAtTime(bodyPos - parentPos, bodyVel - parentVel, t, mu, out pos, out vel);
		}

		/// <summary>
		/// Universal Keplerian propagator. Derives exact cartesian state vectors analytically
		/// at time t across all conic sections (elliptical, parabolic, hyperbolic) using Universal Variables.
		/// </summary>
		public static void GetLocalStateAtTime(double2 relPos, double2 relVel, double t, double mu, out double2 pos, out double2 vel)
		{
			double r0 = math.length(relPos);
			double v0Sq = math.lengthsq(relVel);

			// Guard against physical singularities (zero radius or zero gravity)
			if (r0 <= 1e-10 || mu <= 1e-10 || math.abs(t) < 1e-12)
			{
				pos = relPos;
				vel = relVel;
				return;
			}

			// Reciprocal semi-major axis alpha (1/a). Alpha > 0 (ellipse), Alpha = 0 (parabola), Alpha < 0 (hyperbola)
			double alpha = (2.0 / r0) - (v0Sq / mu);
			double sqrtMu = math.sqrt(mu);
			double r0DotV0 = math.dot(relPos, relVel);

			// Initial estimate for universal anomaly variable chi
			double chi = math.abs(alpha) > 1e-6 ? sqrtMu * t * alpha : sqrtMu * t / r0;
			double z = 0.0, c2 = 0.0, c3 = 0.0, r = r0;

			// Newton-Raphson iteration to solve Universal Kepler Equation for chi
			for (int i = 0; i < 20; i++)
			{
				z = alpha * chi * chi;
				c2 = StumpffC(z);
				c3 = StumpffS(z);

				// Universal Kepler Equation evaluation: F(chi)
				double f_chi = (r0DotV0 / sqrtMu) * chi * chi * c2 + (1.0 - alpha * r0) * math.pow(chi, 3.0) * c3 + r0 * chi - sqrtMu * t;

				// Derivative dF/dchi = current radial distance r
				r = (r0DotV0 / sqrtMu) * chi * (1.0 - z * c3) + (1.0 - alpha * r0) * chi * chi * c2 + r0;

				if (math.abs(r) < 1e-12) break;
				double deltaChi = f_chi / r;
				chi -= deltaChi;
				if (math.abs(deltaChi) < 1e-9) break;
			}

			// Derive Lagrange f and g coefficients
			double f = 1.0 - (chi * chi / r0) * c2;
			double g = t - (math.pow(chi, 3.0) / sqrtMu) * c3;
			
			// Derive time derivatives f_dot and g_dot
			double fDot = (sqrtMu / (r * r0)) * chi * (z * c3 - 1.0);
			double gDot = 1.0 - (chi * chi / r) * c2;

			pos = (f * relPos) + (g * relVel);
			vel = (fDot * relPos) + (gDot * relVel);
		}

		/// <summary>
		/// Universal Keplerian propagator. Derives exact cartesian position vector analytically
		/// at time t across all conic sections using Universal Variables.
		/// </summary>
		public static void GetLocalPosAtTime(PhysicsBody body, PhysicsBody parent, double t, double mu, out double2 pos)
		{
			GetLocalPosAtTime(body.Position, body.Velocity, parent, t, mu, out pos);
		}

		/// <summary>
		/// Universal Keplerian propagator. Derives exact cartesian position vector analytically
		/// at time t across all conic sections (elliptical, parabolic, hyperbolic) using Universal Variables.
		/// </summary>
		public static void GetLocalPosAtTime(double2 bodyPos, double2 bodyVel, PhysicsBody parent, double t, double mu, out double2 pos)
		{
			double2 parentPos = parent != null ? parent.Position : double2.zero;
			double2 parentVel = parent != null ? parent.Velocity : double2.zero;
			GetLocalPosAtTime(bodyPos, bodyVel, parentPos, parentVel, t, mu, out pos);
		}

		/// <summary>
		/// Universal Keplerian propagator. Derives exact cartesian position vector analytically
		/// at time t across all conic sections (elliptical, parabolic, hyperbolic) using Universal Variables.
		/// </summary>
		public static void GetLocalPosAtTime(double2 bodyPos, double2 bodyVel, double2 parentPos, double2 parentVel, double t, double mu, out double2 pos)
		{
			double r0 = math.length(bodyPos - parentPos);
			double2 r0_vec = bodyPos - parentPos;
			double2 v0_vec = bodyVel - parentVel;
			double v0Sq = math.lengthsq(v0_vec);

			// Guard against physical singularities (zero radius or zero gravity)
			if (r0 <= 1e-10 || mu <= 1e-10 || math.abs(t) < 1e-12)
			{
				pos = bodyPos;
				return;
			}

			// Reciprocal semi-major axis alpha (1/a). Alpha > 0 (ellipse), Alpha = 0 (parabola), Alpha < 0 (hyperbola)
			double alpha = (2.0 / r0) - (v0Sq / mu);
			double sqrtMu = math.sqrt(mu);
			double r0DotV0 = math.dot(r0_vec, v0_vec);

			// Initial estimate for universal anomaly variable chi
			double chi = math.abs(alpha) > 1e-6 ? sqrtMu * t * alpha : sqrtMu * t / r0;
			double z = 0.0, c2 = 0.0, c3 = 0.0, r = r0;

			// Newton-Raphson iteration to solve Universal Kepler Equation for chi
			for (int i = 0; i < 20; i++)
			{
				z = alpha * chi * chi;
				c2 = StumpffC(z);
				c3 = StumpffS(z);

				// Universal Kepler Equation evaluation: F(chi)
				double f_chi = (r0DotV0 / sqrtMu) * chi * chi * c2 + (1.0 - alpha * r0) * math.pow(chi, 3.0) * c3 + r0 * chi - sqrtMu * t;
				
				// Derivative dF/dchi = current radial distance r
				r = (r0DotV0 / sqrtMu) * chi * (1.0 - z * c3) + (1.0 - alpha * r0) * chi * chi * c2 + r0;

				if (math.abs(r) < 1e-12) break;
				double deltaChi = f_chi / r;
				chi -= deltaChi;
				if (math.abs(deltaChi) < 1e-9) break;
			}

			// Derive Lagrange f and g coefficients
			double f = 1.0 - (chi * chi / r0) * c2;
			double g = t - (math.pow(chi, 3.0) / sqrtMu) * c3;

			pos = (f * r0_vec) + (g * v0_vec);
		}

		/// <summary>
		/// Computes the exact analytical SoI exit state entirely in coordinates relative to the dominant body.
		/// Returns relative exit position, relative exit velocity, and the exact time of flight to the boundary.
		/// </summary>
		public static bool TryFindSoIExitState(double2 relPos, double2 relVel, double mu, PhysicsBody dominantBody, out double2 relExitPos, out double2 relExitVel, out double exitTime)
		{
			relExitPos = double2.zero;
			relExitVel = double2.zero;
			exitTime = 0.0;

			if (dominantBody == null || dominantBody.ParentSoI == null) return false;

			double r0 = math.length(relPos);
			double v0Sq = math.lengthsq(relVel);
			double h = CalculateSpecificAngularMomentum(relPos, relVel);

			if (math.abs(h) < 1e-10 || mu <= 0.0) return false;

			double2 eVec = CalculateEccentricityVector(relPos, relVel, mu);
			double e = math.length(eVec);
			double a = CalculateSemiMajorAxis(r0, v0Sq, mu);
			double p = (h * h) / mu;
			double rSoi = dominantBody.GetSoIRadius();

			if (r0 >= rSoi) return false;

			if (e < 1.0 && a > 0.0)
			{
				double apoapsis = a * (1.0 + e);
				if (apoapsis < rSoi) return false; // Ellipse never reaches boundary
			}
			
			if (e < 1e-6) return false;

			// Solve cos(nu) = (p / R_SoI - 1) / e
			double cosNu = (p / rSoi - 1.0) / e;
			cosNu = math.clamp(cosNu, -1.0, 1.0);

			double nuSoi = math.acos(cosNu);

			// Determine the correct exit true anomaly based on orbital direction (h)
			// For prograde (h > 0), outbound exit is at +nuSoi. For retrograde (h < 0), it is at 2PI - nuSoi.
			double targetNu = (h > 0.0) ? nuSoi : (2.0 * math.PI - nuSoi);

			double omega = math.atan2(eVec.y, eVec.x);
			double currentAngle = math.atan2(relPos.y, relPos.x);
			double nuStart = currentAngle - omega;

			while (nuStart < 0) nuStart += 2.0 * math.PI;
			while (nuStart >= 2.0 * math.PI) nuStart -= 2.0 * math.PI;

			// Calculate exact time of flight from nuStart to targetNu
			double candTof = 0.0;
			if (e < 1.0)
			{
				double E0 = 2.0 * math.atan2(math.sqrt(1.0 - e) * math.sin(nuStart / 2.0), math.sqrt(1.0 + e) * math.cos(nuStart / 2.0));
				double E1 = 2.0 * math.atan2(math.sqrt(1.0 - e) * math.sin(targetNu / 2.0), math.sqrt(1.0 + e) * math.cos(targetNu / 2.0));

				double M0 = E0 - e * math.sin(E0);
				double M1 = E1 - e * math.sin(E1);

				double n = math.sqrt(mu / math.pow(a, 3.0));
				
				if (h > 0.0)
				{
					candTof = (M1 - M0) / n;
					if (candTof < 0.0) candTof += 2.0 * math.PI / n;
				}
				else
				{
					candTof = (M0 - M1) / n;
					if (candTof < 0.0) candTof += 2.0 * math.PI / n;
				}
			}
			else
			{
				double tanHalfNu0 = math.tan(nuStart / 2.0);
				double tanHalfNu1 = math.tan(targetNu / 2.0);

				double sqrtE = math.sqrt((e - 1.0) / (e + 1.0));
				double x0 = sqrtE * tanHalfNu0;
				double x1 = sqrtE * tanHalfNu1;

				double H0 = 0.5 * math.log((1.0 + x0) / math.max(1e-12, 1.0 - x0));
				double H1 = 0.5 * math.log((1.0 + x1) / math.max(1e-12, 1.0 - x1));

				double M0 = e * math.sinh(H0) - H0;
				double M1 = e * math.sinh(H1) - H1;

				double n = math.sqrt(mu / math.pow(math.abs(a), 3.0));
				candTof = math.abs((M1 - M0) / n);
			}

			if (candTof < 0.0 || double.IsNaN(candTof) || double.IsInfinity(candTof)) return false;

			// 1. Exact Exit Position in Relative Coordinates
			double exitAngle = omega + targetNu;
			relExitPos = new double2(math.cos(exitAngle), math.sin(exitAngle)) * rSoi;

			// 2. Exact Exit Velocity in Relative Coordinates
			double exitVr = (mu / h) * e * math.sin(targetNu);
			double vTheta = h / rSoi;

			double2 uR = new double2(math.cos(exitAngle), math.sin(exitAngle));
			double2 uTheta = new double2(-uR.y, uR.x);
			relExitVel = (uR * exitVr) + (uTheta * vTheta);

			exitTime = candTof;
			return true;
		}
		
		/// <summary>
		/// Snaps a world position perfectly onto a continuous two-body orbit using radial true anomaly projection.
		/// </summary>
		public static bool GetSnappedOrbitalState(PhysicsBody target, PhysicsBody parent, double2 worldTargetPos, out double2 snapPos, out double2 snapVel, out double snapTime)
		{
			if (target == null || parent == null)
			{
				snapPos = target != null ? target.Position : worldTargetPos;
				snapVel = target != null ? target.Velocity : double2.zero;
				snapTime = 0.0;
				return false;
			}

			double mu = target.GravParam;
			double2 r = target.Position - parent.Position;
			double2 v = target.Velocity - parent.Velocity;
			
			double h = CalculateSpecificAngularMomentum(r, v);
			
			if (math.abs(h) < 1e-10)
			{
				snapPos = target.Position;
				snapVel = target.Velocity;
				snapTime = 0.0;
				return false;
			}

			double2 eVec = CalculateEccentricityVector(r, v, mu);
			double e = math.length(eVec);
			
			double2 cursorRel = worldTargetPos - parent.Position;
			
			// Validate vectors to prevent atan2(0,0) generating NaN
			if (math.lengthsq(cursorRel) < 1e-10)
			{
				snapPos = target.Position;
				snapVel = target.Velocity;
				snapTime = 0.0;
				return false;
			}

			// Radially project the cursor position to determine the true anomaly (nu)
			double omega = math.atan2(eVec.y, eVec.x);
			double cursorAngle = math.atan2(cursorRel.y, cursorRel.x);
			double nu = cursorAngle - omega;

			// Determine current true anomaly (nu0) to establish a baseline for time of flight
			double currentAngle = math.atan2(r.y, r.x);
			double nu0 = currentAngle - omega;

			// Normalize true anomalies to [-PI, PI] for boundary evaluation
			while (nu > math.PI) nu -= 2.0 * math.PI;
			while (nu <= -math.PI) nu += 2.0 * math.PI;
			
			while (nu0 > math.PI) nu0 -= 2.0 * math.PI;
			while (nu0 <= -math.PI) nu0 += 2.0 * math.PI;

			// Restrict hyperbolic snapping to prevent evaluating positions on the physical asymptote or mirror branch
			if (e >= 1.0)
			{
				double maxNu = math.acos(-1.0 / e);
				if (math.abs(nu) >= maxNu * 0.99)
				{
					snapPos = target.Position;
					snapVel = target.Velocity;
					snapTime = 0.0;
					return false;
				}
			}

			// Solve polar equation for radius strictly on the continuous analytical trajectory
			double p = (h * h) / mu;
			double rMag = p / (1.0 + e * math.cos(nu));

			double2 r_new = new double2(math.cos(cursorAngle), math.sin(cursorAngle)) * rMag;
			snapPos = parent.Position + r_new;

			// Derive exact instantaneous velocity using the orbital invariants
			double v_x_rel = (-mu / h) * (eVec.y + (r_new.y / rMag));
			double v_y_rel = (mu / h) * (eVec.x + (r_new.x / rMag));

			snapVel = parent.Velocity + new double2(v_x_rel, v_y_rel);

			// --- Time of Flight Calculation ---
			double a = CalculateSemiMajorAxis(math.length(r), math.lengthsq(v), mu);
			
			if (e < 1.0)
			{
				// Elliptical Time of Flight via Eccentric Anomaly
				double E0 = 2.0 * math.atan(math.sqrt((1.0 - e) / (1.0 + e)) * math.tan(nu0 / 2.0));
				double E1 = 2.0 * math.atan(math.sqrt((1.0 - e) / (1.0 + e)) * math.tan(nu / 2.0));
				
				double M0 = E0 - e * math.sin(E0);
				double M1 = E1 - e * math.sin(E1);
				
				double n = math.sqrt(mu / math.pow(a, 3.0));
				snapTime = (M1 - M0) / n;
				
				// Ensure forward time of flight. If negative, snap point requires wrapping around the orbit once.
				if (snapTime < 0)
				{
					snapTime += CalculateOrbitalPeriod(a, mu);
				}
			}
			else
			{
				// Clamp eccentricity to bypass degenerate parabolic division by zero
				e = math.max(e, 1.0001);

				// Hyperbolic Time of Flight via Hyperbolic Anomaly
				double tanHalfNu0 = math.tan(nu0 / 2.0);
				double tanHalfNu1 = math.tan(nu / 2.0);
				
				double sqrtE = math.sqrt((e - 1.0) / (e + 1.0));
				
				double x0 = sqrtE * tanHalfNu0;
				double x1 = sqrtE * tanHalfNu1;
				
				// atanh(x) = 0.5 * ln((1+x)/(1-x))
				double H0 = 0.5 * math.log((1.0 + x0) / (1.0 - x0));
				double H1 = 0.5 * math.log((1.0 + x1) / (1.0 - x1));
				
				double M0 = e * math.sinh(H0) - H0;
				double M1 = e * math.sinh(H1) - H1;
				
				double n = math.sqrt(mu / math.pow(math.abs(a), 3.0));
				snapTime = (M1 - M0) / n;
			}

			return true;
		}

		/// <summary>
		/// Solves Lambert's boundary value problem using Universal Variables and Bisection.
		/// </summary>
		public static bool SolveVelocityToHitTarget(double2 pos1, double2 pos2, double tof, double mu, out double2 v1, bool shortWay = true)
		{
			v1 = double2.zero;

			double r1 = math.length(pos1);
			double r2 = math.length(pos2);

			// Input validation guard: Prevent division by zero from exact-center targeting
			if (r1 < 1e-10)
			{
				pos1 = new double2(1e-10, 0.0);
				r1 = 1e-10;
			}
			if (r2 < 1e-10)
			{
				pos2 = new double2(1e-10, 0.0);
				r2 = 1e-10;
			}
			
			// Clamp dot product to [-1, 1] to prevent NaN errors from floating point imprecision
			double dotProduct = math.dot(pos1, pos2) / (r1 * r2);
			dotProduct = math.clamp(dotProduct, -1.0, 1.0);
			double theta = math.acos(dotProduct);
			
			// Adjust theta for the retrograde/long-way path
			if (!shortWay)
			{
				theta = (2.0 * math.PI) - theta;
			}

			// 'A' is a geometric constant for the transfer path
			double A = math.sin(theta) * math.sqrt((r1 * r2) / (1.0 - math.cos(theta)));
			
			// Degenerate orbit case (180-degree transfer).
			// In a generic 3D engine this requires defining a normal plane. In 2D it's physically ambiguous without a sign injection.
			if (math.abs(A) < 1e-8)
			{
				Debug.LogWarning($"[{nameof(OrbitalMath)}] Transfer angle is approximately 180 or 0 degrees. Geometry is degenerate.");
				return false;
			}

			// Bisection bounds for the universal variable 'z'
			// Negative z corresponds to hyperbolas, positive z corresponds to ellipses
			double zLow = -4.0 * math.PI * math.PI;
			double zHigh = 4.0 * math.PI * math.PI;
			double z = 0.0;
			
			double tolerance = 1e-5;
			bool converged = false;
			
			for (int i = 0; i < 100; i++)
			{
				z = (zLow + zHigh) * 0.5;
				
				double c2 = StumpffC(z);
				double c3 = StumpffS(z);
				
				double y = r1 + r2 + A * (z * c3 - 1.0) / math.sqrt(c2);
				
				// If y is negative, the current z yields an imaginary x.
				// This usually occurs when z is too low for paths where A < 0.
				if (y < 0.0)
				{
					zLow = z;
					continue;
				}
				
				double x = math.sqrt(y / c2);
				double calculatedTof = (math.pow(x, 3.0) * c3 + A * math.sqrt(y)) / math.sqrt(mu);
				
				if (math.abs(calculatedTof - tof) < tolerance)
				{
					converged = true;
					break;
				}
				
				// Time of flight strictly increases with z. Adjust bounds accordingly.
				if (calculatedTof <= tof)
				{
					zLow = z;
				}
				else
				{
					zHigh = z;
				}
			}
			
			if (!converged) return false;
			
			// Calculate final f and g coefficients for velocity extraction
			double finalC2 = StumpffC(z);
			double finalY = r1 + r2 + A * (z * StumpffS(z) - 1.0) / math.sqrt(finalC2);
			
			double f = 1.0 - finalY / r1;
			double g = A * math.sqrt(finalY / mu);
			
			v1 = (pos2 - (f * pos1)) / g;
			return true;
		}

		private static double StumpffC(double z)
		{
			if (z > 1e-6) return (1.0 - math.cos(math.sqrt(z))) / z;
			if (z < -1e-6) return (math.cosh(math.sqrt(-z)) - 1.0) / (-z);
			return 0.5 - (z / 24.0); // Taylor expansion fallback for limits near 0
		}

		private static double StumpffS(double z)
		{
			if (z > 1e-6) return (math.sqrt(z) - math.sin(math.sqrt(z))) / math.pow(z, 1.5);
			if (z < -1e-6) return (math.sinh(math.sqrt(-z)) - math.sqrt(-z)) / math.pow(-z, 1.5);
			return (1.0 / 6.0) - (z / 120.0); // Taylor expansion fallback for limits near 0
		}

		/// <summary>
		/// Solves for the exact orbital intercept velocity given a fixed delta-V magnitude constraint, 
		/// returning both the intercept velocity vector and the exact time of flight to interception.
		/// </summary>
		public static bool SolveInterceptionVelocity(
			double2 bodyPos, double2 bodyVel, double2 targetPos, double2 targetVel, double muTarget, PhysicsBody parent, double firingVelMag,
			out double2 interceptVelocity, out double interceptTime, double projectileMass = 0.0, double maxTime = 86400f, int coarseSteps = 100)
		{
			interceptVelocity = double2.zero;
			interceptTime = 0.0;

			// Input validation
			if (parent == null || firingVelMag <= 0.0 || maxTime <= 0.0 || coarseSteps < 1) return false;

			double muProject = GetStandardGravitationalParameter(parent.Mass, projectileMass);

			double2 parentPos = parent.Position;
			double2 parentVel = parent.Velocity;

			double2 relBodyPos = bodyPos - parentPos;

			double timeStep = maxTime / coarseSteps;

			double prevT = 1e-3;
			double prevError = double.MaxValue;
			bool hasPrev = false;

			GetLocalPosAtTime(targetPos, targetVel, parent, prevT, muTarget, out double2 initTargetPos);
			if (SolveVelocityToHitTarget(relBodyPos, initTargetPos, prevT, muProject, out double2 initTransferVelRel))
			{
				double2 initTransferVelWorld = initTransferVelRel + parentVel;
				prevError = math.length(initTransferVelWorld - bodyVel) - firingVelMag;
				hasPrev = true;
			}

			for (int i = 1; i <= coarseSteps; i++)
			{
				double currentT = i * timeStep;

				GetLocalPosAtTime(targetPos, targetVel, parent, currentT, muTarget, out double2 projectedTargetPos);

				if (SolveVelocityToHitTarget(relBodyPos, projectedTargetPos, currentT, muProject, out double2 transferVelRel))
				{
					double2 transferVelWorld = transferVelRel + parentVel;
					double currentError = math.length(transferVelWorld - bodyVel) - firingVelMag;

					if (hasPrev && (prevError * currentError <= 0.0))
					{
						double tLow = prevT;
						double tHigh = currentT;
						double errLow = prevError;

						double2 bestTransferVel = double2.zero;
						double bestT = currentT;
						bool bisectionSuccess = false;

						for (int j = 0; j < 25; j++)
						{
							double tMid = (tLow + tHigh) * 0.5;
							GetLocalPosAtTime(targetPos, targetVel, parent, tMid, muTarget, out double2 midTargetPos);

							if (!SolveVelocityToHitTarget(relBodyPos, midTargetPos, tMid, muProject, out double2 midVelRel))
							{
								break;
							}

							double2 midTransferVel = midVelRel + parentVel;
							double midError = math.length(midTransferVel - bodyVel) - firingVelMag;

							bestTransferVel = midTransferVel;
							bestT = tMid;
							bisectionSuccess = true;

							if (math.abs(midError) < 1e-4) break;

							if (errLow * midError <= 0.0)
							{
								tHigh = tMid;
							}
							else
							{
								tLow = tMid;
								errLow = midError;
							}
						}

						if (bisectionSuccess)
						{
							double2 requiredDeltaV = bestTransferVel - bodyVel;
							double firingAngle = math.atan2(requiredDeltaV.y, requiredDeltaV.x);
							double2 exactDeltaV = new double2(math.cos(firingAngle), math.sin(firingAngle)) * firingVelMag;

							interceptVelocity = bodyVel + exactDeltaV;
							interceptTime = bestT;
							return true;
						}
					}

					prevError = currentError;
					prevT = currentT;
					hasPrev = true;
				}
				else
				{
					hasPrev = false;
				}
			}

			return false;
		}

		public static bool SolveInterceptionVelocity(
			double2 bodyPos, double2 bodyVel, double2 targetPos, double2 targetVel, double muTarget, PhysicsBody parent, double firingVelMag,
			out double2 interceptVelocity, double projectileMass = 0.0, double maxTime = 86400f, int coarseSteps = 100)
		{
			return SolveInterceptionVelocity(bodyPos, bodyVel, targetPos, targetVel, muTarget, parent, firingVelMag, out interceptVelocity, out _, projectileMass, maxTime, coarseSteps);
		}

		public static bool SolveInterceptionVelocity(
			PhysicsBody firingBody, PhysicsBody targetBody, double firingVelMag,
			out double2 interceptVelocity, out double interceptTime, double projectileMass = 0.0, double maxTime = 86400f, int coarseSteps = 100)
		{
			interceptVelocity = double2.zero;
			interceptTime = 0.0;

			if (firingBody == null || firingBody.IsDestroyed) return false;
			if (targetBody == null || targetBody.IsDestroyed) return false;
			if (firingBody.ParentSoI != targetBody.ParentSoI || firingBody.ParentSoI == null) return false;

			return SolveInterceptionVelocity(firingBody.Position, firingBody.Velocity, targetBody.Position, targetBody.Velocity, targetBody.GravParam, firingBody.ParentSoI, firingVelMag, out interceptVelocity, out interceptTime, projectileMass, maxTime, coarseSteps);
		}

		public static bool SolveInterceptionVelocity(
			PhysicsBody firingBody, PhysicsBody targetBody, double firingVelMag,
			out double2 interceptVelocity, double projectileMass = 0.0, double maxTime = 86400f, int coarseSteps = 100)
		{
			return SolveInterceptionVelocity(firingBody, targetBody, firingVelMag, out interceptVelocity, out _, projectileMass, maxTime, coarseSteps);
		}

		#endregion

		#region Trajectories & Boundaries

		/// <summary>
		/// Generates the directional Sphere of Influence boundary contour for a celestial body in world-space coordinates.
		/// </summary>
		public static double2[] CalculateSoIBoundary(PhysicsBody body, int resolution)
		{
			if (body == null || body.ParentSoI == null || resolution < 3) return new double2[0];

			double2[] points = new double2[resolution];
			double2 relPos = body.Position - body.ParentSoI.Position;
			
			// Base angle aligning theta = 0 with the vector pointing from parent to body
			double baseAngle = math.atan2(relPos.y, relPos.x);

			for (int i = 0; i < resolution; i++)
			{
				double theta = (2.0 * math.PI * i) / (resolution - 1);
				double rSoi = body.GetSoIRadius();

				double worldAngle = baseAngle + theta;
				double2 offset = new double2(math.cos(worldAngle), math.sin(worldAngle)) * rSoi;

				points[i] = body.Position + offset;
			}

			return points;
		}

		/// <summary>
		/// Evaluates a sequence of points plotting out the trajectory of a two-body orbit.
		/// Handles both elliptical (closed) and hyperbolic (open) trajectories.
		/// </summary>
		public static double2[] CalculateTwoBodyOrbit(double2 relPos, double2 relVel, double2 parentPos, double mu, int resolution)
		{
			// Input validation
			if (resolution < 2) return new double2[0];

			double dist = math.length(relPos);
			double speedSq = math.lengthsq(relVel);

			// Utilize existing helpers to determine orbital invariants
			double h = CalculateSpecificAngularMomentum(relPos, relVel);
			double dir = h < 0.0 ? -1.0 : 1.0;

			double2 eVec = CalculateEccentricityVector(relPos, relVel, mu);
			double e = math.length(eVec);
			
			double a = CalculateSemiMajorAxis(dist, speedSq, mu);
			double omega = math.atan2(eVec.y, eVec.x);
			
			double2[] points = new double2[resolution];

			if (e < 1.0)
			{
				// Elliptical trajectory: Start from current true anomaly directly without tan(nu/2) singularities
				double currentAngle = math.atan2(relPos.y, relPos.x);
				double nuStart = currentAngle - omega;
				double EStart = 2.0 * math.atan2(math.sqrt(1.0 - e) * math.sin(nuStart / 2.0), math.sqrt(1.0 + e) * math.cos(nuStart / 2.0));

				double minorAxis = math.sqrt(1.0 - e * e);
				double cosOmega = math.cos(omega);
				double sinOmega = math.sin(omega);

				for (int i = 0; i < resolution; i++)
				{
					double fraction = (double)i / (resolution - 1);
					// Step forward cleanly in eccentric anomaly from current position
					double E = EStart + dir * fraction * 2.0 * math.PI;
					
					double xPrime = a * (math.cos(E) - e);
					double yPrime = a * minorAxis * math.sin(E);

					double rotX = xPrime * cosOmega - yPrime * sinOmega;
					double rotY = xPrime * sinOmega + yPrime * cosOmega;

					points[i] = new double2(parentPos.x + rotX, parentPos.y + rotY);
				}
			}
			else
			{
				// Hyperbolic trajectory forward from current true anomaly
				e = math.max(e, 1.00001);
				double p = (h * h) / mu;
				double cosOmega = math.cos(omega);
				double sinOmega = math.sin(omega);

				double currentAngle = math.atan2(relPos.y, relPos.x);
				double nuStart = currentAngle - omega;

				double maxNu = math.acos(-1.0 / e) * 0.99;
				nuStart = math.clamp(nuStart, -maxNu, maxNu);
				double targetNu = dir > 0.0 ? maxNu : -maxNu;

				for (int i = 0; i < resolution; i++)
				{
					double t = (double)i / (resolution - 1);
					double nu = math.lerp(nuStart, targetNu, t);

					double rMag = p / (1.0 + e * math.cos(nu));
					double xPrime = rMag * math.cos(nu);
					double yPrime = rMag * math.sin(nu);

					double rotX = xPrime * cosOmega - yPrime * sinOmega;
					double rotY = xPrime * sinOmega + yPrime * cosOmega;

					points[i] = new double2(parentPos.x + rotX, parentPos.y + rotY);
				}
			}

			return points;
		}

		/// <summary>
		/// Evaluates a sequence of points plotting out the trajectory of a two-body orbit restricted to a specific time duration.
		/// If pathDuration is greater than or equal to the orbital period, it returns a full orbit.
		/// </summary>
		public static double2[] CalculateTwoBodyOrbit(double2 relPos, double2 relVel, double2 parentPos, double mu, double pathDuration, int resolution)
		{
			if (resolution < 2) return new double2[0];

			double dist = math.length(relPos);
			double speedSq = math.lengthsq(relVel);

			double h = CalculateSpecificAngularMomentum(relPos, relVel);
			double dir = h < 0.0 ? -1.0 : 1.0;

			double2 eVec = CalculateEccentricityVector(relPos, relVel, mu);
			double e = math.length(eVec);
			
			double a = CalculateSemiMajorAxis(dist, speedSq, mu);
			double omega = math.atan2(eVec.y, eVec.x);
			
			double2[] points = new double2[resolution];

			if (e < 1.0 && a > 0.0)
			{
				double period = CalculateOrbitalPeriod(a, mu);
				double timeSpan = (double.IsInfinity(period) || period <= 0) ? pathDuration : math.min(pathDuration, period);
				double n = math.sqrt(mu / math.pow(a, 3.0));

				// Current eccentric anomaly (E0)
				double currentAngle = math.atan2(relPos.y, relPos.x);
				double nuStart = currentAngle - omega;
				double E0 = 2.0 * math.atan2(math.sqrt(1.0 - e) * math.sin(nuStart / 2.0), math.sqrt(1.0 + e) * math.cos(nuStart / 2.0));

				// Current mean anomaly and target mean anomaly
				double M0 = E0 - e * math.sin(E0);
				double M1 = M0 + dir * n * timeSpan;

				// Solve Kepler's equation for final eccentric anomaly (E1)
				double E1 = M1;
				for (int iter = 0; iter < 10; iter++)
				{
					double f = E1 - e * math.sin(E1) - M1;
					double df = 1.0 - e * math.cos(E1);
					if (math.abs(df) < 1e-12) break;
					double dE = f / df;
					E1 -= dE;
					if (math.abs(dE) < 1e-9) break;
				}

				double minorAxis = math.sqrt(1.0 - e * e);
				double cosOmega = math.cos(omega);
				double sinOmega = math.sin(omega);

				for (int i = 0; i < resolution; i++)
				{
					double fraction = (double)i / (resolution - 1);
					double E = math.lerp(E0, E1, fraction);
					
					double xPrime = a * (math.cos(E) - e);
					double yPrime = a * minorAxis * math.sin(E);

					double rotX = xPrime * cosOmega - yPrime * sinOmega;
					double rotY = xPrime * sinOmega + yPrime * cosOmega;

					points[i] = new double2(parentPos.x + rotX, parentPos.y + rotY);
				}
			}
			else
			{
				e = math.max(e, 1.00001);
				double p = (h * h) / mu;
				double n = math.sqrt(mu / math.pow(math.abs(a), 3.0));
				double cosOmega = math.cos(omega);
				double sinOmega = math.sin(omega);

				double currentAngle = math.atan2(relPos.y, relPos.x);
				double nuStart = currentAngle - omega;
				double maxNu = math.acos(-1.0 / e) * 0.99;
				nuStart = math.clamp(nuStart, -maxNu, maxNu);

				double tanHalfNu = math.tan(nuStart / 2.0);
				double sqrtE = math.sqrt((e - 1.0) / (e + 1.0));
				double x0 = sqrtE * tanHalfNu;
				double H0 = 0.5 * math.log((1.0 + x0) / math.max(1e-12, 1.0 - x0));

				double M0 = e * math.sinh(H0) - H0;
				double M1 = M0 + dir * n * pathDuration;

				// Solve hyperbolic Kepler's equation for final hyperbolic anomaly (H1)
				double H1 = M1 / e;
				for (int iter = 0; iter < 10; iter++)
				{
					double f = e * math.sinh(H1) - H1 - M1;
					double df = e * math.cosh(H1) - 1.0;
					if (math.abs(df) < 1e-12) break;
					double dH = f / df;
					H1 -= dH;
					if (math.abs(dH) < 1e-9) break;
				}

				for (int i = 0; i < resolution; i++)
				{
					double fraction = (double)i / (resolution - 1);
					double H = math.lerp(H0, H1, fraction);

					double tanHalfNuTarget = math.tanh(H / 2.0) / sqrtE;
					double nu = 2.0 * math.atan(tanHalfNuTarget);
					nu = math.clamp(nu, -maxNu, maxNu);

					double rMag = p / (1.0 + e * math.cos(nu));
					double xPrime = rMag * math.cos(nu);
					double yPrime = rMag * math.sin(nu);

					double rotX = xPrime * cosOmega - yPrime * sinOmega;
					double rotY = xPrime * sinOmega + yPrime * cosOmega;

					points[i] = new double2(parentPos.x + rotX, parentPos.y + rotY);
				}
			}

			return points;
		}

		#endregion
	}
}