using Gravitas2D;
using Unity.Mathematics;

namespace GravitasDemo
{
	public class OrbitMatchController
	{
		public bool IsEnabled { get; private set; }
		protected NavigationCapability _manager;


		// Orbit Match Reference Parameters
		// public Entity TargetOrbitEntity { get; set; }
		public double2 TargetOrbitPositionOffset { get; set; }
		public double2 TargetOrbitVelocity { get; set; }

		// Desired velocity correction for the active orbit-match guidance update.
		// Translational thrust is scaled from this error so it tapers smoothly at capture.
		public double RequiredOrbitMatchDeltaV { get; private set; }

		// Approximate velocity-error response time. Smaller values are more aggressive;
		// thrust still saturates at MaxForce for large corrections.
		public float OrbitMatchResponseTime { get; set; } = 1.0f;

		public OrbitMatchController(NavigationCapability manager)
		{
			_manager = manager;
		}

		public void Clear()
		{
			IsEnabled = false;
			TargetOrbitPositionOffset = double2.zero;
			TargetOrbitVelocity = double2.zero;
			RequiredOrbitMatchDeltaV = 0;
		}

		public bool CheckCompleted()
		{
			PhysicsBody parentSoI = _manager.TargetEntity != null && !_manager.TargetEntity.IsDestroyed && _manager.TargetEntity.PhysicsBody != null
				? _manager.TargetEntity.PhysicsBody.ParentSoI
				: _manager.OwnerEntity.PhysicsBody.ParentSoI;

			if (parentSoI != null)
			{
				double2 tPos = _manager.TargetEntity != null && !_manager.TargetEntity.IsDestroyed ? _manager.TargetEntity.Position : parentSoI.Position + TargetOrbitPositionOffset;
				double2 tVel = _manager.TargetEntity != null && !_manager.TargetEntity.IsDestroyed ? _manager.TargetEntity.Velocity : TargetOrbitVelocity;

				double mu = _manager.OwnerEntity.PhysicsBody.GravParam;
				double2 rT = tPos - parentSoI.Position;
				double2 vT = tVel - parentSoI.Velocity;
				double rTDist = math.length(rT);
				double2 rC = _manager.OwnerEntity.Position - parentSoI.Position;
				double2 vC = _manager.OwnerEntity.Velocity - parentSoI.Velocity;
				double rCDist = math.length(rC);

				if (mu > 0.0 && rTDist > 1.0 && rCDist > 1.0)
				{
					double targetEnergy = (math.lengthsq(vT) * 0.5) - (mu / rTDist);
					double targetH = (rT.x * vT.y) - (rT.y * vT.x);
					double currEnergy = (math.lengthsq(vC) * 0.5) - (mu / rCDist);
					double currH = (rC.x * vC.y) - (rC.y * vC.x);

					if (math.abs(targetEnergy) > 1e-12 && math.abs(currEnergy) > 1e-12 && math.abs(targetH) > 1e-9)
					{
						double targetA = -mu / (2.0 * targetEnergy);
						double currA = -mu / (2.0 * currEnergy);
						double2 targetEcc = new double2((vT.y * targetH / mu) - (rT.x / rTDist), (-vT.x * targetH / mu) - (rT.y / rTDist));
						double2 currEcc = new double2((vC.y * currH / mu) - (rC.x / rCDist), (-vC.x * currH / mu) - (rC.y / rCDist));

						bool aMatch = math.abs(currA - targetA) <= math.abs(targetA) * 0.002;
						bool hMatch = math.abs(currH - targetH) <= math.max(math.abs(targetH) * 0.002, 1e-6);
						bool eMatch = math.length(targetEcc) < 0.003
							? math.abs(math.length(currEcc) - math.length(targetEcc)) <= 0.0005
							: math.length(currEcc - targetEcc) <= math.max(1e-4, math.length(targetEcc) * 0.01);

						if (aMatch && hMatch && eMatch)
						{
							return true;
						}
					}
				}
			}

			return false;
		}
		
		/// <summary>
		/// Computes the trajectory to match a defined target orbit's energy and angular momentum invariants, 
		/// ensuring exact geometric orientation without rotational offset.
		/// </summary>
		public bool ComputeOrbitMatchTrajectory(float dt, out float burnOrientation, out float burnThrottle)
		{
			Entity ownerEntity = _manager.OwnerEntity;
			Entity targetEntity = _manager.TargetEntity;

			double mass = ownerEntity.Mass;
			double2 currentPos = ownerEntity.Position;
			double2 currentVel = ownerEntity.Velocity;

			burnOrientation = 0f;
			burnThrottle = 0f;

			RequiredOrbitMatchDeltaV = 0.0;
			if (mass <= 0.0) return false;

			PhysicsBody parentSoI = ownerEntity.DominantEntity?.PhysicsBody ?? null;
			if (parentSoI == null) return false;

			// Dynamically compute absolute target position from relative offset if manually placed
			double2 tPos = targetEntity != null && !targetEntity.IsDestroyed 
				? targetEntity.Position
				: parentSoI.Position + TargetOrbitPositionOffset;
			
			double2 tVel = targetEntity != null && !targetEntity.IsDestroyed 
				? targetEntity.Velocity
				: TargetOrbitVelocity;

			if (double.IsNaN(tPos.x) || double.IsNaN(tPos.y) || double.IsInfinity(tPos.x) || double.IsInfinity(tPos.y) ||
				double.IsNaN(tVel.x) || double.IsNaN(tVel.y) || double.IsInfinity(tVel.x) || double.IsInfinity(tVel.y))
				return false;

			double mu = ownerEntity.PhysicsBody.GravParam;
			if (mu <= 0.0 || double.IsNaN(mu) || double.IsInfinity(mu)) return false;

			double2 rTargetRel = tPos - parentSoI.Position;
			double2 vTargetRel = tVel - parentSoI.Velocity;
			double rTargetDist = math.length(rTargetRel);
			if (rTargetDist <= 1.0) return false;

			double targetEnergy = 0.5 * math.lengthsq(vTargetRel) - (mu / rTargetDist);
			double targetH = (rTargetRel.x * vTargetRel.y) - (rTargetRel.y * vTargetRel.x);
			if (math.abs(targetEnergy) < 1e-12 || math.abs(targetH) < 1e-9) return false;

			double targetSemiMajorAxis = -mu / (2.0 * targetEnergy);
			if (double.IsNaN(targetSemiMajorAxis) || double.IsInfinity(targetSemiMajorAxis)) return false;

			double2 targetEccVec = new double2(
				(vTargetRel.y * targetH / mu) - (rTargetRel.x / rTargetDist),
				(-vTargetRel.x * targetH / mu) - (rTargetRel.y / rTargetDist));

			double2 rCurrRel = currentPos - parentSoI.Position;
			double rCurrDist = math.length(rCurrRel);
			if (rCurrDist <= 1.0) return false;

			double2 uR = rCurrRel / rCurrDist;
			double2 uTheta = new double2(-uR.y, uR.x);

			double targetP = (targetH * targetH) / mu;
			double radiusDenominator = 1.0 + math.dot(targetEccVec, uR);
			if (radiusDenominator <= 1e-6) return false;
			double targetRadiusAtCurrentAngle = targetP / radiusDenominator;

			double targetRadialVelocity = -(mu / targetH) * math.dot(targetEccVec, uTheta);

			double maxAccel = _manager.MaxForce / mass;
			if (maxAccel <= 0.0 || double.IsNaN(maxAccel) || double.IsInfinity(maxAccel)) return false;
			double radiusError = targetRadiusAtCurrentAngle - rCurrDist;
			double radialCorrectionSpeed = math.sqrt(maxAccel * math.abs(radiusError));
			double localCircularSpeed = math.sqrt(mu / rCurrDist);
			radialCorrectionSpeed = math.min(radialCorrectionSpeed, localCircularSpeed * 0.1);
			double radialCorrectionSign = radiusError > 0.0 ? 1.0 : radiusError < 0.0 ? -1.0 : 0.0;
			double vrDesired = targetRadialVelocity + radialCorrectionSign * radialCorrectionSpeed;
			double vThetaDesired = targetH / rCurrDist;

			double2 vDesiredRel = (uR * vrDesired) + (uTheta * vThetaDesired);
			double2 vDesired = vDesiredRel + parentSoI.Velocity;
			double2 requiredVelChange = vDesired - currentVel;

			double responseTime = math.max(math.max(0.25, OrbitMatchResponseTime), 2.0 * math.max(0.0, dt));
			burnThrottle = (float)(maxAccel * responseTime > 0.0 ? math.clamp(RequiredOrbitMatchDeltaV / (maxAccel * responseTime), 0.0, 1.0) : 0.0);

			double len = math.length(requiredVelChange);
			if (len > 1e-8 && !double.IsNaN(len) && !double.IsInfinity(len))
			{
				RequiredOrbitMatchDeltaV = len;
				double2 burnDir = requiredVelChange / len;
				if (!double.IsNaN(burnDir.x) && !double.IsNaN(burnDir.y) && !double.IsInfinity(burnDir.x) && !double.IsInfinity(burnDir.y))
				{
					burnOrientation = math.atan2((float)burnDir.y, (float)burnDir.x);
					return true;
				}
			}

			burnThrottle = 0f;
			return false;
		}
	}
}