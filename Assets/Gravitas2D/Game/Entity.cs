using System;
using System.Collections.Generic;
using Gravitas2D;
using Unity.Mathematics;
using UnityEngine;

namespace GravitasDemo
{
	public interface IEntityCapability { }

	public enum SensorType { Optical = 0, Infrared = 1, Radar = 2 }

	public class SensorCapability : IEntityCapability
	{
		public SensorType Type { get; set; }
		public double Range { get; set; }
		public float FieldOfViewDegrees { get; set; } = 360f; // Default omnidirectional
		public float OrientationOffset { get; set; } = 0f;    // Offset relative to entity orientation
		public float Sensitivity { get; set; } = 1.0f;        // Tuning multiplier for detection strength
	}

	public class LaunchCapability : IEntityCapability
	{
		public int ProjectileCompositionIndex { get; set; }
		public float ProjectileRadius { get; set; }
		public float FireSpeed { get; set; }
		public float ReloadTime { get; set; }
		public int Ammo { get; set; }
	}

	public class MissileCapability : IEntityCapability
	{
		public int ProjectileCompositionIndex { get; set; }
		public float ProjectileRadius { get; set; }
		public float FireSpeed { get; set; }
		public float ReloadTime { get; set; }
		public int Ammo { get; set; }
	}

	public class WarheadCapability : IEntityCapability
	{
		public double WarheadEnergy { get; set; } // Joules

		public static WarheadCapability Missile => new WarheadCapability
		{
			WarheadEnergy = MissileEnergy
		};

		public static double MissileEnergy => 1.25e12;
	}

	public enum TaskStatus
	{
		Pending = 0,
		Running = 1,
		Completed = 2,
		Failed = 3,
		Aborted = 4
	}

	public interface IAutonomousTask
	{
		string TaskName { get; }
		TaskStatus Status { get; }
		void OnStart(Entity entity);
		TaskStatus Tick(Entity entity, float dt);
		void OnAbort(Entity entity);
	}

	/// <summary>
	/// Handles automated sequential execution of tasks.
	/// </summary>
	public class AutonomyController
	{
		private Entity _ownerEntity;
		private Queue<IAutonomousTask> _taskQueue = new Queue<IAutonomousTask>();
		private List<IAutonomousTask> _loopTemplateTasks = new List<IAutonomousTask>();

		private IAutonomousTask _currentTask;
		public bool IsRunning { get; private set; } = false;
		public bool IsLooping { get; set; } = false;

		public double CargoMass { get; set; } = 0.0;
		public double MaxCargoCapacity { get; set; } = 1000.0;

		public int RemainingTaskCount => _taskQueue.Count + (_currentTask != null ? 1 : 0);
		public IAutonomousTask CurrentTask => _currentTask;

		public event Action<IAutonomousTask> OnTaskCompleted;
		public event Action<IAutonomousTask, string> OnTaskFailed;
		public event Action OnAutonomySequenceCompleted;
		public event Action<Entity, bool> OnRunningChanged;

		public AutonomyController(Entity ownerEntity)
		{
			_ownerEntity = ownerEntity;
		}

		public void EnqueueTask(IAutonomousTask task, bool addToLoopTemplate = false)
		{
			if (task == null) throw new ArgumentNullException(nameof(task));
			_taskQueue.Enqueue(task);
			if (addToLoopTemplate)
			{
				_loopTemplateTasks.Add(task);
			}
		}

		public void ClearTasks()
		{
			if (_currentTask != null && _currentTask.Status == TaskStatus.Running)
			{
				_currentTask.OnAbort(_ownerEntity);
			}
			_currentTask = null;
			_taskQueue.Clear();
			_loopTemplateTasks.Clear();
			IsRunning = false;
			OnRunningChanged?.Invoke(_ownerEntity, false);
		}

		public void StartAutonomy()
		{
			if (_ownerEntity == null || _taskQueue.Count == 0) return;
			IsRunning = true;
			OnRunningChanged?.Invoke(_ownerEntity, true);
			AdvanceQueue();
		}

		public void StopAutonomy()
		{
			ClearTasks();
		}

		public void EvaluateAutonomy(float dt)
		{
			if (!IsRunning || _ownerEntity == null || _ownerEntity.IsDestroyed) return;

			if (_currentTask == null)
			{
				AdvanceQueue();
				if (_currentTask == null)
				{
					if (IsLooping && _loopTemplateTasks.Count > 0)
					{
						RequeueLoopTemplate();
						AdvanceQueue();
					}
					
					if (_currentTask == null)
					{
						IsRunning = false;
						OnAutonomySequenceCompleted?.Invoke();
						OnRunningChanged?.Invoke(_ownerEntity, false);
						return;
					}
				}
			}

			// Tick the active task
			TaskStatus status = _currentTask.Tick(_ownerEntity, dt);

			if (status == TaskStatus.Completed)
			{
				var completed = _currentTask;
				_currentTask = null;
				Debug.Log($"{_ownerEntity.Name} completed task {completed.TaskName}");
				OnTaskCompleted?.Invoke(completed);
				AdvanceQueue();
			}
			else if (status == TaskStatus.Failed)
			{
				var failed = _currentTask;
				string reason = "Task returned failure status.";
				_currentTask = null;
				IsRunning = false;
				Debug.Log($"{_ownerEntity.Name} failed task {failed.TaskName}");
				OnTaskFailed?.Invoke(failed, reason);
				OnRunningChanged?.Invoke(_ownerEntity, false);
			}
		}

		private void AdvanceQueue()
		{
			if (_taskQueue.Count > 0)
			{
				_currentTask = _taskQueue.Dequeue();
				_currentTask.OnStart(_ownerEntity);
				Debug.Log($"Starting {_currentTask.TaskName} for entity {_ownerEntity.Name}");
			}
			else
			{
				_currentTask = null;
			}
		}

		private void RequeueLoopTemplate()
		{
			foreach (var task in _loopTemplateTasks)
			{
				_taskQueue.Enqueue(task);
			}
		}
	}

	#region Tasks

	public class NavigateTask : IAutonomousTask
	{
		public string TaskName => "NavigateTask";
		public TaskStatus Status { get; private set; } = TaskStatus.Pending;

		private readonly Entity _targetEntity;
		private readonly double2 _targetPosition;
		private readonly bool _isRendezvous;
		private readonly float _arrivalThreshold;
		private readonly float _timeoutSeconds;
		private float _elapsedTime = 0f;

		public NavigateTask(Entity targetEntity, bool isRendezvous = false, float arrivalThreshold = 10f, float timeoutSeconds = float.PositiveInfinity)
		{
			_targetEntity = targetEntity ?? throw new ArgumentNullException(nameof(targetEntity));
			_isRendezvous = isRendezvous;
			_arrivalThreshold = arrivalThreshold;
			_timeoutSeconds = timeoutSeconds;
		}

		public NavigateTask(double2 targetPosition, bool isRendezvous = false, float arrivalThreshold = 10f, float timeoutSeconds = 300f)
		{
			_targetPosition = targetPosition;
			_isRendezvous = isRendezvous;
			_arrivalThreshold = arrivalThreshold;
			_timeoutSeconds = timeoutSeconds;
		}

		public void OnStart(Entity entity)
		{
			Status = TaskStatus.Running;
			_elapsedTime = 0f;

			var navCap = entity.GetCapability<NavigationCapability>();
			if (navCap == null)
			{
				Status = TaskStatus.Failed;
				return;
			}

			navCap.EndNavigation();
			navCap.ArrivalThreshold = _arrivalThreshold;
			navCap.IsRendezvousMode = _isRendezvous;
			navCap.IsStationKeepingMode = false;
			navCap.IsOrbitMatchMode = false;

			if (_targetEntity != null)
			{
				navCap.TargetEntity = _targetEntity;
			}
			else
			{
				navCap.TargetPosition = _targetPosition;
			}

			navCap.IsNavigating = true;
			navCap.IsEnabled = true;
		}

		public TaskStatus Tick(Entity entity, float dt)
		{
			if (Status != TaskStatus.Running) return Status;

			_elapsedTime += dt;
			if (_elapsedTime >= _timeoutSeconds)
			{
				Status = TaskStatus.Failed;
				entity.GetCapability<NavigationCapability>()?.EndNavigation();
				return Status;
			}

			if (_targetEntity != null && _targetEntity.IsDestroyed)
			{
				Status = TaskStatus.Failed;
				entity.GetCapability<NavigationCapability>()?.EndNavigation();
				return Status;
			}

			// Evaluate navigation step directly from the active task
			UpdateNavigation(entity, dt);

			var navCap = entity.GetCapability<NavigationCapability>();
			if (navCap == null || !navCap.IsNavigating)
			{
				// Navigation finished successfully
				Status = TaskStatus.Completed;
			}

			return Status;
		}

		public void OnAbort(Entity entity)
		{
			Status = TaskStatus.Aborted;
			entity.GetCapability<NavigationCapability>()?.EndNavigation();
		}
		public void UpdateNavigation(Entity entity, float dt)
		{
			var navCap = entity.GetCapability<NavigationCapability>();
			if (navCap == null || !navCap.IsNavigating) return;

			dt = navCap.PrepareAttitudeControl(dt);

			if (navCap.TargetEntity != null && navCap.TargetEntity.IsDestroyed)
			{
				navCap.EndNavigation();
				return;
			}

			bool trajectoryComputed = navCap.IsRendezvousMode
				? navCap.ComputeRendezvousTrajectory()
				: navCap.ComputeInterceptTrajectory();

			if (trajectoryComputed)
			{
				float burnAngle = navCap.BurnOrientation;
				navCap.UpdateTargetOrientation(burnAngle, dt);
				navCap.IsEnabled = true;

				float analogTorque = navCap.EvaluateAnalogTorque(dt);
				if (analogTorque != 0f)
				{
					entity.PhysicsBody.AddTorque(analogTorque);
				}

				float2 worldForce = new float2(math.cos(burnAngle), math.sin(burnAngle)) * navCap.MaxForce;
				entity.PhysicsBody.AddWorldForce(worldForce);

				double2 activeTargetPos = navCap.TargetEntity != null ? navCap.TargetEntity.Position : navCap.TargetPosition;
				double distanceToTarget = math.distance(entity.Position, activeTargetPos);
				double relativeSpeed = navCap.TargetEntity != null ? math.distance(entity.Velocity, navCap.TargetEntity.Velocity) : math.distance(entity.Position, navCap.TargetVelocity);

				bool arrived = navCap.IsRendezvousMode
					? (distanceToTarget <= navCap.ArrivalThreshold && relativeSpeed <= 1.0)
					: (distanceToTarget <= navCap.ArrivalThreshold);

				if (arrived)
				{
					navCap.EndNavigation();
				}
			}
			else
			{
				navCap.EndNavigation();
			}
		}
	}

	public class StationKeepTask : IAutonomousTask
	{
		public string TaskName => "StationKeepTask";
		public TaskStatus Status { get; private set; } = TaskStatus.Pending;

		private readonly Entity _targetEntity;
		private readonly double2 _targetPosition;
		private readonly float _timeoutSeconds;
		private float _elapsedTime = 0f;

		public StationKeepTask(Entity targetEntity, float timeoutSeconds = float.PositiveInfinity)
		{
			_targetEntity = targetEntity ?? throw new ArgumentNullException(nameof(targetEntity));
			_timeoutSeconds = timeoutSeconds;
		}

		public StationKeepTask(double2 targetPosition, float timeoutSeconds = float.PositiveInfinity)
		{
			_targetPosition = targetPosition;
			_timeoutSeconds = timeoutSeconds;
		}

		public void OnStart(Entity entity)
		{
			Status = TaskStatus.Running;
			_elapsedTime = 0f;

			var navCap = entity.GetCapability<NavigationCapability>();
			if (navCap == null)
			{
				Status = TaskStatus.Failed;
				return;
			}

			navCap.EndNavigation();
			navCap.IsStationKeepingMode = true;
			navCap.IsRendezvousMode = false;
			navCap.IsOrbitMatchMode = false;

			if (_targetEntity != null)
			{
				navCap.TargetEntity = _targetEntity;
			}
			else
			{
				navCap.TargetPosition = _targetPosition;
			}

			navCap.IsNavigating = true;
			navCap.IsEnabled = true;
		}

		public TaskStatus Tick(Entity entity, float dt)
		{
			if (Status != TaskStatus.Running) return Status;

			_elapsedTime += dt;
			if (_elapsedTime >= _timeoutSeconds)
			{
				Status = TaskStatus.Completed;
				entity.GetCapability<NavigationCapability>()?.EndNavigation();
				return Status;
			}

			if (_targetEntity != null && _targetEntity.IsDestroyed)
			{
				Status = TaskStatus.Failed;
				entity.GetCapability<NavigationCapability>()?.EndNavigation();
				return Status;
			}

			UpdateNavigation(entity, dt);
			return Status;
		}

		public void OnAbort(Entity entity)
		{
			Status = TaskStatus.Aborted;
			entity.GetCapability<NavigationCapability>()?.EndNavigation();
		}

		public void UpdateNavigation(Entity entity, float dt)
		{
			var navCap = entity.GetCapability<NavigationCapability>();
			if (navCap == null || !navCap.IsNavigating) return;

			dt = navCap.PrepareAttitudeControl(dt);

			if (navCap.TargetEntity != null && navCap.TargetEntity.IsDestroyed)
			{
				navCap.EndNavigation();
				return;
			}

			bool trajectoryComputed = navCap.ComputeStationKeepingTrajectory();
			if (trajectoryComputed)
			{
				float burnAngle = navCap.BurnOrientation;
				navCap.UpdateTargetOrientation(burnAngle, dt);
				navCap.IsEnabled = true;

				float analogTorque = navCap.EvaluateAnalogTorque(dt);
				if (analogTorque != 0f)
				{
					entity.PhysicsBody.AddTorque(analogTorque);
				}

				double2 targetPos = navCap.TargetEntity != null ? navCap.TargetEntity.Position : navCap.TargetPosition;
				double2 targetVel = navCap.TargetEntity != null ? navCap.TargetEntity.Velocity : navCap.TargetVelocity;

				double2 positionError = targetPos - entity.Position;
				double2 velocityError = entity.Velocity - targetVel;

				double2 desiredAccel = (positionError * 0.2) - (velocityError * 1.5);
				double desiredForce = math.length(desiredAccel) * entity.Mass;

				float throttle = (float)math.clamp(desiredForce / navCap.MaxForce, 0.0, 1.0);
				float2 vehicleForward = new float2(math.cos(entity.Orientation), math.sin(entity.Orientation));
				float2 burnDir = new float2(math.cos(burnAngle), math.sin(burnAngle));
				float alignment = math.max(0f, math.dot(vehicleForward, burnDir));

				float2 worldForce = vehicleForward * (navCap.MaxForce * throttle * alignment);
				if (!navCap.RequireAttitudeAlignmentForThrust || navCap.IsBurnAligned(dt))
				{
					entity.PhysicsBody.AddWorldForce(worldForce);
				}
			}
		}
	}

	public class OrbitMatchTask : IAutonomousTask
	{
		public string TaskName => "OrbitMatchTask";
		public TaskStatus Status { get; private set; } = TaskStatus.Pending;

		private readonly Entity _targetOrbitEntity;
		private readonly double2 _targetOrbitPosition;
		private readonly double2 _targetOrbitVelocity;
		private readonly bool _useEntity;
		private readonly float _timeoutSeconds;
		private float _elapsedTime = 0f;

		public OrbitMatchTask(Entity targetOrbitEntity, float timeoutSeconds = float.PositiveInfinity)
		{
			_targetOrbitEntity = targetOrbitEntity ?? throw new ArgumentNullException(nameof(targetOrbitEntity));
			_useEntity = true;
			_timeoutSeconds = timeoutSeconds;
		}

		public OrbitMatchTask(double2 targetPosition, double2 targetVelocity, float timeoutSeconds = float.PositiveInfinity)
		{
			_targetOrbitPosition = targetPosition;
			_targetOrbitVelocity = targetVelocity;
			_useEntity = false;
			_timeoutSeconds = timeoutSeconds;
		}

		public void OnStart(Entity entity)
		{
			Status = TaskStatus.Running;
			_elapsedTime = 0f;

			var navCap = entity.GetCapability<NavigationCapability>();
			if (navCap == null)
			{
				Status = TaskStatus.Failed;
				return;
			}

			navCap.EndNavigation();
			navCap.IsOrbitMatchMode = true;
			navCap.IsRendezvousMode = false;
			navCap.IsStationKeepingMode = false;

			if (_useEntity)
			{
				navCap.TargetOrbitEntity = _targetOrbitEntity;
			}
			else
			{
				PhysicsBody parentSoI = entity.PhysicsBody != null ? entity.PhysicsBody.ParentSoI : null;
				navCap.TargetOrbitPositionOffset = parentSoI != null ? _targetOrbitPosition - parentSoI.Position : _targetOrbitPosition;
				navCap.TargetOrbitVelocity = _targetOrbitVelocity;
			}

			navCap.IsNavigating = true;
			navCap.IsEnabled = true;
		}

		public TaskStatus Tick(Entity entity, float dt)
		{
			if (Status != TaskStatus.Running) return Status;

			_elapsedTime += dt;
			if (_elapsedTime >= _timeoutSeconds)
			{
				Status = TaskStatus.Failed;
				entity.GetCapability<NavigationCapability>()?.EndNavigation();
				return Status;
			}

			if (_useEntity && (_targetOrbitEntity == null || _targetOrbitEntity.IsDestroyed))
			{
				Status = TaskStatus.Failed;
				entity.GetCapability<NavigationCapability>()?.EndNavigation();
				return Status;
			}

			UpdateNavigation(entity, dt);

			var navCap = entity.GetCapability<NavigationCapability>();
			if (navCap == null || !navCap.IsNavigating)
			{
				Status = TaskStatus.Completed;
			}

			return Status;
		}

		public void OnAbort(Entity entity)
		{
			Status = TaskStatus.Aborted;
			entity.GetCapability<NavigationCapability>()?.EndNavigation();
		}

		public void UpdateNavigation(Entity entity, float dt)
		{
			var navCap = entity.GetCapability<NavigationCapability>();
			if (navCap == null || !navCap.IsNavigating) return;

			dt = navCap.PrepareAttitudeControl(dt);
			bool trajectoryComputed = navCap.ComputeOrbitMatchTrajectory();

			if (trajectoryComputed)
			{
				float burnAngle = navCap.BurnOrientation;
				navCap.UpdateTargetOrientation(burnAngle, dt);
				navCap.IsEnabled = true;

				float analogTorque = navCap.EvaluateAnalogTorque(dt);
				if (analogTorque != 0f)
				{
					entity.PhysicsBody.AddTorque(analogTorque);
				}

				PhysicsBody parentSoI = navCap.TargetOrbitEntity != null && !navCap.TargetOrbitEntity.IsDestroyed && navCap.TargetOrbitEntity.PhysicsBody != null
					? navCap.TargetOrbitEntity.PhysicsBody.ParentSoI
					: entity.PhysicsBody.ParentSoI;

				if (parentSoI != null)
				{
					double2 tPos = navCap.TargetOrbitEntity != null && !navCap.TargetOrbitEntity.IsDestroyed ? navCap.TargetOrbitEntity.Position : parentSoI.Position + navCap.TargetOrbitPositionOffset;
					double2 tVel = navCap.TargetOrbitEntity != null && !navCap.TargetOrbitEntity.IsDestroyed ? navCap.TargetOrbitEntity.Velocity : navCap.TargetOrbitVelocity;

					double mu = entity.PhysicsBody.GravParam;
					double2 rT = tPos - parentSoI.Position;
					double2 vT = tVel - parentSoI.Velocity;
					double rTDist = math.length(rT);
					double2 rC = entity.Position - parentSoI.Position;
					double2 vC = entity.Velocity - parentSoI.Velocity;
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
								navCap.EndNavigation();
							}
							else
							{
								double maxAccel = navCap.MaxForce / math.max(1e-12, entity.Mass);
								double responseTime = math.max(math.max(0.25, navCap.OrbitMatchResponseTime), 2.0 * math.max(0.0, dt));
								double throttle = maxAccel * responseTime > 0.0 ? math.clamp(navCap.RequiredOrbitMatchDeltaV / (maxAccel * responseTime), 0.0, 1.0) : 0.0;

								if (throttle > 1e-4 && (!navCap.RequireAttitudeAlignmentForThrust || navCap.IsBurnAligned(dt)))
								{
									float2 worldForce = new float2(math.cos(burnAngle), math.sin(burnAngle)) * navCap.MaxForce * (float)throttle;
									entity.PhysicsBody.AddWorldForce(worldForce);
								}
							}
						}
					}
				}
			}
		}
	}

	public class OrientationTask : IAutonomousTask
	{
		public string TaskName => "OrientationTask";
		public TaskStatus Status { get; private set; } = TaskStatus.Pending;

		private readonly float _targetOrientation;
		private readonly float _timeoutSeconds;
		private float _elapsedTime = 0f;

		public OrientationTask(float targetOrientation, float timeoutSeconds = float.PositiveInfinity)
		{
			_targetOrientation = targetOrientation;
			_timeoutSeconds = timeoutSeconds;
		}

		public void OnStart(Entity entity)
		{
			Status = TaskStatus.Running;
			_elapsedTime = 0f;

			var navCap = entity.GetCapability<NavigationCapability>();
			if (navCap == null)
			{
				Status = TaskStatus.Failed;
				return;
			}

			navCap.SetTargetOrientation(_targetOrientation);
		}

		public TaskStatus Tick(Entity entity, float dt)
		{
			if (Status != TaskStatus.Running) return Status;

			_elapsedTime += dt;
			if (_elapsedTime >= _timeoutSeconds)
			{
				Status = TaskStatus.Failed;
				entity.GetCapability<NavigationCapability>()?.EndNavigation();
				return Status;
			}

			UpdateNavigation(entity, dt);

			var navCap = entity.GetCapability<NavigationCapability>();
			if (navCap != null && navCap.IsBurnAligned(dt))
			{
				Status = TaskStatus.Completed;
			}

			return Status;
		}

		public void OnAbort(Entity entity)
		{
			Status = TaskStatus.Aborted;
			entity.GetCapability<NavigationCapability>()?.EndNavigation();
		}

		public void UpdateNavigation(Entity entity, float dt)
		{
			var navCap = entity.GetCapability<NavigationCapability>();
			if (navCap == null) return;

			dt = navCap.PrepareAttitudeControl(dt);
			navCap.UpdateTargetOrientation(navCap.TargetOrientation, dt);
			float analogTorque = navCap.EvaluateAnalogTorque(dt);
			if (analogTorque != 0f)
			{
				entity.PhysicsBody.AddTorque(analogTorque);
			}
		}
	}

	public class ExtractMassTask : IAutonomousTask
	{
		public string TaskName => "ExtractMassTask";
		public TaskStatus Status { get; private set; } = TaskStatus.Pending;

		private readonly AutonomyController _controller;
		private readonly Entity _targetEntity;
		private readonly double _targetExtractionAmount;
		private readonly double _extractionRatePerSecond;
		private double _extractedTotal = 0.0;
		private readonly float _maxRange;

		public ExtractMassTask(AutonomyController controller, Entity targetEntity, double extractionAmount, double ratePerSecond = 50.0, float maxRange = 5000.0f)
		{
			_controller = controller ?? throw new ArgumentNullException(nameof(controller));
			_targetEntity = targetEntity ?? throw new ArgumentNullException(nameof(targetEntity));
			_targetExtractionAmount = extractionAmount;
			_extractionRatePerSecond = ratePerSecond;
			_maxRange = maxRange;
		}

		public void OnStart(Entity entity)
		{
			Status = TaskStatus.Running;
			_extractedTotal = 0.0;
		}

		public TaskStatus Tick(Entity entity, float dt)
		{
			if (Status != TaskStatus.Running) return Status;

			if (_targetEntity == null || _targetEntity.IsDestroyed)
			{
				Status = TaskStatus.Failed;
				return Status;
			}

			// Validate proximity range
			double dist = math.distance(entity.Position, _targetEntity.Position);
			if (dist > _maxRange)
			{
				// Out of range failure condition
				Status = TaskStatus.Failed;
				return Status;
			}

			double amountToExtract = math.min(_extractionRatePerSecond * (double)dt, _targetExtractionAmount - _extractedTotal);
			if (_targetEntity.Mass <= amountToExtract || amountToExtract <= 0.0)
			{
				Status = TaskStatus.Completed;
				return Status;
			}

			// Perform mass transfer safely
			double newTargetMass = _targetEntity.Mass - amountToExtract;
			_targetEntity.PhysicsBody.SetMass(newTargetMass);

			if (_controller != null)
			{
				_controller.CargoMass = math.min(_controller.MaxCargoCapacity, _controller.CargoMass + amountToExtract);
			}

			_extractedTotal += amountToExtract;

			if (_extractedTotal >= _targetExtractionAmount || _controller?.CargoMass >= _controller?.MaxCargoCapacity)
			{
				Status = TaskStatus.Completed;
			}

			return Status;
		}

		public void OnAbort(Entity entity)
		{
			Status = TaskStatus.Aborted;
		}
	}

	public class DepositMassTask : IAutonomousTask
	{
		public string TaskName => "DepositMassTask";
		public TaskStatus Status { get; private set; } = TaskStatus.Pending;

		private readonly AutonomyController _controller;
		private readonly Entity _stationEntity;
		private readonly float _maxRange;

		public DepositMassTask(AutonomyController controller, Entity stationEntity, float maxRange = 5000.0f)
		{
			_controller = controller ?? throw new ArgumentNullException(nameof(controller));
			_stationEntity = stationEntity ?? throw new ArgumentNullException(nameof(stationEntity));
			_maxRange = maxRange;
		}

		public void OnStart(Entity entity)
		{
			Status = TaskStatus.Running;
		}

		public TaskStatus Tick(Entity entity, float dt)
		{
			if (Status != TaskStatus.Running) return Status;

			if (_stationEntity == null || _stationEntity.IsDestroyed)
			{
				Status = TaskStatus.Failed;
				return Status;
			}

			double dist = math.distance(entity.Position, _stationEntity.Position);
			if (dist > _maxRange)
			{
				Status = TaskStatus.Failed;
				return Status;
			}

			if (_controller == null || _controller.CargoMass <= 0.0)
			{
				Status = TaskStatus.Completed;
				return Status;
			}

			double depositAmount = _controller.CargoMass;
			double newStationMass = _stationEntity.Mass + depositAmount;

			_stationEntity.PhysicsBody.SetMass(newStationMass);
			_controller.CargoMass = 0.0;

			Status = TaskStatus.Completed;
			return Status;
		}

		public void OnAbort(Entity entity)
		{
			Status = TaskStatus.Aborted;
		}
	}

	#endregion

	public class NavigationCapability : IEntityCapability
	{
		public Entity OwnerEntity { get; set; }
		public Entity TargetEntity { get; set; }
		public double2 TargetPosition { get; set; }
		public double2 TargetVelocity { get; set; }
		public float MaxForce { get; set; } = 10000f; // Newtons
		public bool IsNavigating { get; set; } = false;
		public float ArrivalThreshold { get; set; } = 10.0f; // Meters
		public bool IsRendezvousMode { get; set; } = false; // Toggle between Intercept and Rendezvous
		public bool IsStationKeepingMode { get; set; } = false; // Toggle for relative position hold / station-keeping
		public bool IsOrbitMatchMode { get; set; } = false; // Toggle for matching a reference orbit

		// Attitude / Maneuvering Control Properties
		public float TargetOrientation { get; set; } = 0f; // Radians
		public float MaxTorque { get; set; } = 1000f; // Newton-meters
		public bool IsEnabled { get; set; } = true;

		// Predictive attitude-tracking state. Derivatives are filtered because the commanded
		// burn vector can move discontinuously between guidance updates.
		private float _targetAngularVelocity;
		private float _targetAngularAcceleration;
		private bool _hasTargetOrientationSample;

		// Nominal attitude-loop tuning. The controller adapts these values to the actual
		// scaled physics step and current vehicle inertia / torque authority at runtime.
		public float AttitudePredictionLeadTime { get; set; } = 0.04f;
		public float AttitudeMaxPredictionHorizon { get; set; } = 0.15f;
		public float AttitudeTargetRateFilterTime { get; set; } = 0.025f;
		public float AttitudeTargetAccelerationFilterTime { get; set; } = 0.05f;
		public float AttitudeTargetRateLimit { get; set; } = 100f;
		public float AttitudeTargetAccelerationLimit { get; set; } = 5000f;
		public float AttitudeCaptureAngle { get; set; } = 0.025f;
		public float AttitudeSwitchingAngle { get; set; } = 0.001f;

		// Base burn tolerances. The effective angular tolerance is allowed a very small
		// increase at coarse simulation steps, capped at 0.005 rad (0.286 degrees).
		public float BurnAlignmentToleranceRadians { get; set; } = 0.002f;
		public float BurnAlignmentRateTolerance { get; set; } = 0.02f;
		public bool RequireAttitudeAlignmentForThrust { get; set; } = true;

		// Recomputed from the live physics model and simulation timestep. These are kept
		// separate from the public nominal settings so time-scale changes do not compound.
		private float _controlStep = 0.02f;
		private float _effectivePredictionLeadTime = 0.04f;
		private float _effectivePredictionHorizon = 0.15f;
		private float _effectiveRateFilterTime = 0.025f;
		private float _effectiveAccelerationFilterTime = 0.05f;
		private float _effectiveTargetRateLimit = 100f;
		private float _effectiveTargetAccelerationLimit = 5000f;
		private float _effectiveCaptureAngle = 0.025f;
		private float _effectiveSwitchingAngle = 0.001f;
		private float _effectiveBurnAlignmentTolerance = 0.002f;
		private float _effectiveBurnRateTolerance = 0.02f;
		private float _effectiveMaxAngularAcceleration;
		private float _effectiveNaturalFrequency;

		// Read-only diagnostics for inspecting the automatic tuning at runtime.
		public float EffectiveAttitudeTimeStep => _controlStep;
		public float EffectiveMaxAngularAcceleration => _effectiveMaxAngularAcceleration;
		public float EffectiveAttitudeNaturalFrequency => _effectiveNaturalFrequency;
		public float EffectiveBurnAlignmentToleranceRadians => _effectiveBurnAlignmentTolerance;
		public float EffectiveBurnAlignmentRateTolerance => _effectiveBurnRateTolerance;

		// Orbit Match Reference Parameters
		public Entity TargetOrbitEntity { get; set; }
		public double2 TargetOrbitPositionOffset { get; set; }
		public double2 TargetOrbitVelocity { get; set; }

		public float BurnOrientation { get; private set; }

		// Desired velocity correction for the active orbit-match guidance update.
		// Translational thrust is scaled from this error so it tapers smoothly at capture.
		public double RequiredOrbitMatchDeltaV { get; private set; }

		// Approximate velocity-error response time. Smaller values are more aggressive;
		// thrust still saturates at MaxForce for large corrections.
		public float OrbitMatchResponseTime { get; set; } = 1.0f;

		// Low-pass the commanded thrust direction to suppress frame-to-frame sign flips.
		public float OrbitMatchDirectionResponseTime { get; set; } = 0.25f;
		private double2 _smoothedOrbitMatchBurnDirection;
		private bool _hasSmoothedOrbitMatchBurnDirection;

		// Rendezvous Clearance
		public float ClearanceDistance { get; set; } = 1000;

		private float _lastFlightTime = -1f;

		public float TimeOfFlight { get; private set; }

		public static NavigationCapability Missile => new NavigationCapability
		{
			MaxForce = MaxMissileForce, // 50g acceleration
			MaxTorque = 10000000
		};

		public static float MaxMissileForce => (float)2.5797e8; // 50g acceleration

		public void EndNavigation()
		{
			_lastFlightTime = -1f;
			TimeOfFlight = -1f;
			TargetEntity = null;
			TargetPosition = double2.zero;
			TargetVelocity = double2.zero;
			IsNavigating = false;
			IsRendezvousMode = false;
			IsStationKeepingMode = false;
			IsOrbitMatchMode = false;
			IsEnabled = false;
			TargetOrientation = OwnerEntity.Orientation;
			BurnOrientation = 0;
			_targetAngularVelocity = 0f;
			_targetAngularAcceleration = 0f;
			_hasTargetOrientationSample = false;
			TargetOrbitEntity = null;
			TargetOrbitPositionOffset = double2.zero;
			TargetOrbitVelocity = double2.zero;
			RequiredOrbitMatchDeltaV = 0;
		}

		/// <summary>
		/// Resolves the actual simulation-time step (including the time multiplier) and
		/// retunes the attitude loop from the current body inertia and configured torque.
		/// Call once at the start of each navigation evaluation.
		/// </summary>
		public float PrepareAttitudeControl(float fallbackDeltaTime)
		{
			float dt = fallbackDeltaTime;
			PhysicsManager2D manager = PhysicsManager2D.Instance;
			if (manager != null && manager.IsInitialized)
			{
				float scaledStep = manager.ScaledPhysicsTimeStep;
				if (IsFinite(scaledStep) && scaledStep > 0.00001f)
					dt = scaledStep;
			}

			if (!IsFinite(dt) || dt <= 0.00001f)
				dt = Mathf.Max(0.0001f, Time.fixedDeltaTime);

			_controlStep = dt;
			TuneAttitudeParameters(dt);
			return dt;
		}

		private void TuneAttitudeParameters(float dt)
		{
			dt = math.max(0.0001f, dt);

			double inertia = OwnerEntity != null && OwnerEntity.PhysicsBody != null
				? OwnerEntity.PhysicsBody.Inertia
				: 1.0;
			if (inertia <= 0.0 || double.IsNaN(inertia) || double.IsInfinity(inertia))
				inertia = 1.0;

			float torqueLimit = math.abs(MaxTorque);
			_effectiveMaxAngularAcceleration = torqueLimit > 0f
				? (float)(torqueLimit / inertia)
				: 0f;
			if (!IsFinite(_effectiveMaxAngularAcceleration))
				_effectiveMaxAngularAcceleration = 0f;

			// A prediction must cover at least one actual simulation step. At 10x speed,
			// for example, a 0.02 s fixed step may become a 0.2 s control interval.
			_effectivePredictionLeadTime = math.max(math.max(0f, AttitudePredictionLeadTime), dt);
			_effectivePredictionHorizon = math.max(
				math.max(0f, AttitudeMaxPredictionHorizon),
				_effectivePredictionLeadTime);

			// Derivative filters are never allowed to be shorter than the sampling interval.
			// This prevents noisy command-rate estimates when the time multiplier is increased.
			_effectiveRateFilterTime = math.max(math.max(0.001f, math.abs(AttitudeTargetRateFilterTime)), dt);
			_effectiveAccelerationFilterTime = math.max(
				math.max(0.001f, math.abs(AttitudeTargetAccelerationFilterTime)),
				1.5f * dt);

			// Limit the reference rate to a rate that the available angular acceleration can
			// reasonably brake over a half-turn, while retaining the user's setting as a cap.
			float authorityRate = _effectiveMaxAngularAcceleration > 0f
				? math.sqrt(2f * _effectiveMaxAngularAcceleration * Mathf.PI)
				: 0.1f;
			_effectiveTargetRateLimit = math.max(0.1f, math.min(
				math.max(0.1f, math.abs(AttitudeTargetRateLimit)),
				authorityRate));

			float authorityAccelerationLimit = math.max(1f, 4f * _effectiveMaxAngularAcceleration);
			_effectiveTargetAccelerationLimit = math.max(1f, math.min(
				math.max(1f, math.abs(AttitudeTargetAccelerationLimit)),
				authorityAccelerationLimit));

			_effectiveCaptureAngle = math.max(0.0001f, math.abs(AttitudeCaptureAngle));
			_effectiveSwitchingAngle = math.max(0.0001f, math.abs(AttitudeSwitchingAngle));

			// Discrete-time gain scheduling: the ideal continuous-time natural frequency is
			// based on available angular acceleration and capture angle, then limited so the
			// controller cannot demand an excessively fast response for this sampling interval.
			float authorityFrequency = _effectiveMaxAngularAcceleration > 0f
				? math.sqrt(2f * _effectiveMaxAngularAcceleration / _effectiveCaptureAngle)
				: 0f;
			float sampleLimitedFrequency = 0.65f / dt;
			_effectiveNaturalFrequency = math.min(authorityFrequency, sampleLimitedFrequency);
			if (!IsFinite(_effectiveNaturalFrequency))
				_effectiveNaturalFrequency = 0f;

			// Keep the requested precision at normal step sizes. Relax it only slightly when
			// the acceleration and the discrete step make sub-step angular resolution coarse.
			float stepAngularResolution = _effectiveMaxAngularAcceleration * dt * dt;
			float discretizationAllowance = math.min(0.005f, 0.02f * stepAngularResolution);
			_effectiveBurnAlignmentTolerance = math.max(
				math.max(0.00001f, math.abs(BurnAlignmentToleranceRadians)),
				discretizationAllowance);
			// The acceptable relative rate is linked to the vehicle's available braking
			// authority: at this rate, the ideal stopping distance is roughly the angular
			// alignment tolerance. Cap the model-derived allowance to avoid permitting a
			// visibly spinning vehicle to burn; the configured value remains a floor.
			float rateFromStoppingAngle = math.sqrt(
				2f * _effectiveMaxAngularAcceleration * _effectiveBurnAlignmentTolerance);
			float modelRateAllowance = math.min(0.25f, rateFromStoppingAngle);
			_effectiveBurnRateTolerance = math.max(
				math.max(0.00001f, math.abs(BurnAlignmentRateTolerance)),
				modelRateAllowance);
		}

		private static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);

		/// <summary>
		/// Updates target-angle kinematics. The command angle is not filtered; its rate and
		/// acceleration estimates are filtered using time constants adapted to the simulation step.
		/// </summary>
		public void UpdateTargetOrientation(float newTargetOrientation, float dt)
		{
			if (!IsFinite(newTargetOrientation)) return;
			dt = PrepareAttitudeControl(dt);

			if (dt <= 0.0001f)
			{
				TargetOrientation = newTargetOrientation;
				_targetAngularVelocity = 0f;
				_targetAngularAcceleration = 0f;
				_hasTargetOrientationSample = false;
				return;
			}

			float deltaAngle = WrapAngle(newTargetOrientation - TargetOrientation);
			TargetOrientation = newTargetOrientation;

			// The first sample establishes a reference. Do not interpret initialization as
			// instantaneous target motion, which would generate a false feedforward spike.
			if (!_hasTargetOrientationSample)
			{
				_targetAngularVelocity = 0f;
				_targetAngularAcceleration = 0f;
				_hasTargetOrientationSample = true;
				return;
			}

			float measuredOmega = math.clamp(deltaAngle / dt, -_effectiveTargetRateLimit, _effectiveTargetRateLimit);
			float previousOmega = _targetAngularVelocity;
			float rateBlend = 1f - math.exp(-dt / _effectiveRateFilterTime);
			_targetAngularVelocity = math.lerp(previousOmega, measuredOmega, rateBlend);

			float measuredAlpha = (_targetAngularVelocity - previousOmega) / dt;
			measuredAlpha = math.clamp(
				measuredAlpha,
				-_effectiveTargetAccelerationLimit,
				_effectiveTargetAccelerationLimit);
			float accelerationBlend = 1f - math.exp(-dt / _effectiveAccelerationFilterTime);
			_targetAngularAcceleration = math.lerp(_targetAngularAcceleration, measuredAlpha, accelerationBlend);
		}
		
		/// <summary>
		/// Predictive, rate-aware attitude control. Outside the capture region it uses a
		/// phase-plane braking law; close to the setpoint it transitions to damped PD
		/// control. Target acceleration is feedforward-compensated with torque headroom
		/// reserved for feedback.
		/// </summary>
		public float EvaluateAnalogTorque(float deltaTime)
		{
			if (!IsEnabled || OwnerEntity == null || OwnerEntity.PhysicsBody == null) return 0f;
			deltaTime = PrepareAttitudeControl(deltaTime);

			double inertia = OwnerEntity.PhysicsBody.Inertia;
			if (inertia <= 0.0 || double.IsNaN(inertia) || double.IsInfinity(inertia)) inertia = 1.0;
			float torqueLimit = math.abs(MaxTorque);
			if (torqueLimit <= 0f) return 0f;

			float alphaMax = (float)(torqueLimit / inertia);
			if (alphaMax <= 0f || !IsFinite(alphaMax)) return 0f;

			float predictionTime = math.min(
				math.max(_controlStep, _effectivePredictionLeadTime),
				_effectivePredictionHorizon);
			float bodyOmega = OwnerEntity.AngularVelocity;

			// Reserve at least half of the available angular acceleration for feedback/braking.
			float targetAlpha = math.clamp(_targetAngularAcceleration, -0.5f * alphaMax, 0.5f * alphaMax);
			float feedbackAlphaMax = math.max(0.5f * alphaMax, alphaMax - math.abs(targetAlpha));

			float predictedTargetAngle = TargetOrientation
				+ (_targetAngularVelocity * predictionTime)
				+ (0.5f * targetAlpha * predictionTime * predictionTime);
			float predictedBodyAngle = OwnerEntity.Orientation
				+ (bodyOmega * predictionTime)
				+ (0.5f * targetAlpha * predictionTime * predictionTime);
			float error = WrapAngle(predictedTargetAngle - predictedBodyAngle);
			float relativeOmega = bodyOmega - _targetAngularVelocity;

			// Phase-plane braking uses signed stopping angle, so braking begins before the
			// commanded orientation is crossed, including when the target itself is moving.
			// The switching layer grows with alpha * dt^2 (not dt alone) so its discrete-time
			// correction remains proportional to a fraction of the error at coarse time scales.
			float brakeAngle = (relativeOmega * math.abs(relativeOmega)) / (2f * feedbackAlphaMax);
			float phaseError = error - brakeAngle;
			float switchingWidth = math.max(
				_effectiveSwitchingAngle,
				feedbackAlphaMax * _controlStep * _controlStep);
			float phaseAlpha = feedbackAlphaMax * math.clamp(phaseError / switchingWidth, -1f, 1f);

			// Model- and step-aware terminal PD gains. The natural frequency is capped in
			// TuneAttitudeParameters so large time multipliers do not create discrete-time
			// overshoot that prevents the burn-alignment gate from ever being satisfied.
			float naturalFrequency = _effectiveNaturalFrequency;
			float kp = naturalFrequency * naturalFrequency;
			float kd = 2f * naturalFrequency; // critical damping
			float pdAlpha = math.clamp(kp * error - kd * relativeOmega, -feedbackAlphaMax, feedbackAlphaMax);

			float blend = math.clamp((math.abs(error) - _effectiveCaptureAngle) / _effectiveCaptureAngle, 0f, 1f);
			blend = blend * blend * (3f - 2f * blend);
			float feedbackAlpha = math.lerp(pdAlpha, phaseAlpha, blend);
			float commandedAlpha = targetAlpha + feedbackAlpha;
			return math.clamp((float)(inertia * commandedAlpha), -torqueLimit, torqueLimit);
		}

		/// <summary>Returns true when the force direction for the current physics step is aligned.</summary>
		public bool IsBurnAligned(float deltaTime)
		{
			if (OwnerEntity == null || OwnerEntity.PhysicsBody == null) return false;
			PrepareAttitudeControl(deltaTime);

			float error = WrapAngle(TargetOrientation - OwnerEntity.Orientation);
			float relativeOmega = OwnerEntity.AngularVelocity - _targetAngularVelocity;

			return math.abs(error) <= _effectiveBurnAlignmentTolerance
				&& math.abs(relativeOmega) <= _effectiveBurnRateTolerance;
		}

		private static float WrapAngle(float angle)
		{
			while (angle > Mathf.PI) angle -= 2f * Mathf.PI;
			while (angle < -Mathf.PI) angle += 2f * Mathf.PI;
			return angle;
		}

		public void SetTargetOrientation(float targetOrientation)
		{
			TargetOrientation = targetOrientation;
			_targetAngularVelocity = 0f;
			_targetAngularAcceleration = 0f;
			_hasTargetOrientationSample = false;
			IsNavigating = false; // Disable active transit so it acts purely as an attitude hold
			IsRendezvousMode = false;
			IsStationKeepingMode = false;
			IsOrbitMatchMode = false;
			IsEnabled = true;
		}

		/// <summary>
		/// Updates target position and velocity.
		/// </summary>
		private void UpdateTargetState()
		{
			if (TargetEntity == null || TargetEntity.IsDestroyed) return;

			if (IsRendezvousMode)
			{
				double targetRadius = TargetEntity.Radius;
				double targetClearance = TargetEntity.IsCelestial ? targetRadius * 0.1 : ClearanceDistance;

				if (OwnerEntity.InGroup)
				{
					double2 groupRelPos = OwnerEntity.Group.GetFormationPosition(OwnerEntity);

					Entity leader = OwnerEntity.Group.Leader;
					double separation = targetRadius + leader.Radius + targetClearance;
					double2 normal = math.normalize(leader.Position - TargetEntity.Position);

					TargetPosition = TargetEntity.Position + separation * normal + groupRelPos;
				}
				else
				{
					double separation = targetRadius + OwnerEntity.Radius + targetClearance;
					double2 normal = math.normalize(OwnerEntity.Position - TargetEntity.Position);

					TargetPosition = TargetEntity.Position + separation * normal;
				}
			}
			else
			{
				TargetPosition = TargetEntity.Position;
			}

			TargetVelocity = TargetEntity.Velocity;
		}

		/// <summary>
		/// Statically computes the gravity-compensated quartic time of flight for an intercept trajectory.
		/// </summary>
		public static float ComputeInterceptTimeOfFlight(double2 currentPos, double2 currentVel, double2 targetPos, double2 targetVel, double mass, float maxForce, double mu, PhysicsBody parentSoI, float initialGuess = -1f)
		{
			double2 R = targetPos - currentPos;
			double2 V = targetVel - currentVel;
			double distance = math.length(R);

			if (distance <= 1.0 || double.IsNaN(distance)) return 0f;

			float acceleration = (float)(maxForce / mass);
			if (acceleration <= 0f || float.IsNaN(acceleration)) return 0f;

			double2 gravityAcc = double2.zero;
			if (parentSoI != null)
			{
				double2 rVec = currentPos - parentSoI.Position;
				double rDist = math.length(rVec);
				if (rDist > 1.0)
				{
					gravityAcc = -mu * rVec / (rDist * rDist * rDist);
				}
			}
			else
			{
				double rDist = math.length(currentPos);
				if (rDist > 1.0)
				{
					gravityAcc = -mu * currentPos / (rDist * rDist * rDist);
				}
			}

			float t = (initialGuess > 0f && distance > 10.0) ? initialGuess : (float)math.sqrt(2.0 * distance / acceleration);
			if (float.IsNaN(t) || float.IsInfinity(t) || t <= 0.001f) t = 0.01f;

			bool converged = false;
			float bestT = t;

			for (int i = 0; i < 12; i++)
			{
				if (double.IsNaN(t) || double.IsInfinity(t) || t <= 0.0) break;

				bestT = t;
				double2 R_corrected = R - (0.5 * gravityAcc * (t * t));

				double V_sq = math.lengthsq(V);
				double R_dot_V = math.dot(R_corrected, V);
				double R_sq = math.lengthsq(R_corrected);

				double t2 = t * t;
				double t3 = t2 * t;
				double t4 = t2 * t2;

				double f = 0.25 * acceleration * acceleration * t4 - V_sq * t2 - 2.0 * R_dot_V * t - R_sq;
				double df = acceleration * acceleration * t3 - 2.0 * V_sq * t - 2.0 * R_dot_V;

				if (math.abs(df) < 1e-6) break;

				float dtStep = (float)(f / df);
				if (float.IsNaN(dtStep) || float.IsInfinity(dtStep)) break;

				t -= dtStep;

				if (math.abs(dtStep) < 1e-4 || i == 11)
				{
					converged = true;
					if (t <= 0.001f) t = 0.001f;
					bestT = t;
					break;
				}
			}

			if (!converged && bestT > 0f && !float.IsNaN(bestT) && !float.IsInfinity(bestT))
			{
				t = bestT;
			}

			if (converged && t > 0f && !float.IsNaN(t) && !float.IsInfinity(t))
			{
				return t;
			}

			return (float)math.sqrt(2.0 * distance / acceleration);
		}

		/// <summary>
		/// Calculates the required burn orientation using a gravity-compensated quartic time-of-flight solver.
		/// </summary>
		public bool ComputeInterceptTrajectory()
		{
			BurnOrientation = 0f;
			TimeOfFlight = _lastFlightTime > 0f ? _lastFlightTime : 1.0f;

			double mass = OwnerEntity.Mass;
			double2 currentPos = OwnerEntity.Position;
			double2 currentVel = OwnerEntity.Velocity;
			double mu = OwnerEntity.PhysicsBody.GravParam;

			if (mass <= 0.0) return false;

			UpdateTargetState();

			if (double.IsNaN(TargetPosition.x) || double.IsNaN(TargetPosition.y)) return false;

			double2 R = TargetPosition - currentPos;
			double2 V = TargetVelocity - currentVel;
			double distance = math.length(R);

			if (distance <= 1.0 || double.IsNaN(distance)) return false;

			float acceleration = (float)(MaxForce / mass);
			if (acceleration <= 0f || float.IsNaN(acceleration)) return false;

			PhysicsBody parentSoI = OwnerEntity.DominantEntity?.PhysicsBody ?? null;

			// Invoke the static helper to get the time of flight
			float t = ComputeInterceptTimeOfFlight(currentPos, currentVel, TargetPosition, TargetVelocity, mass, MaxForce, mu, parentSoI, _lastFlightTime);

			if (t > 0f && !float.IsNaN(t) && !float.IsInfinity(t))
			{
				_lastFlightTime = t;
				TimeOfFlight = t;
			}
			else
			{
				TimeOfFlight = _lastFlightTime > 0f ? _lastFlightTime : (float)math.sqrt(2.0 * distance / acceleration);
				t = TimeOfFlight;
			}

			double2 gravityAcc = double2.zero;
			if (parentSoI != null)
			{
				double2 rVec = currentPos - parentSoI.Position;
				double rDist = math.length(rVec);
				if (rDist > 1.0)
				{
					gravityAcc = -mu * rVec / (rDist * rDist * rDist);
				}
			}
			else
			{
				double rDist = math.length(currentPos);
				if (rDist > 1.0)
				{
					gravityAcc = -mu * currentPos / (rDist * rDist * rDist);
				}
			}

			double2 R_corrected = R - (0.5 * gravityAcc * (t * t));
			double2 requiredVelChange = R_corrected + (V * t);

			double len = math.length(requiredVelChange);
			if (len > 1e-5 && !double.IsNaN(len) && !double.IsInfinity(len))
			{
				double2 burnDir = requiredVelChange / len;
				if (!double.IsNaN(burnDir.x) && !double.IsNaN(burnDir.y))
				{
					BurnOrientation = math.atan2((float)burnDir.y, (float)burnDir.x);
					return true;
				}
			}

			if (distance > 1e-5)
			{
				double2 burnDir = R / distance;
				BurnOrientation = math.atan2((float)burnDir.y, (float)burnDir.x);
				return true;
			}

			return false;
		}

		/// <summary>
		/// Statically computes the time of flight / time-to-go for a rendezvous trajectory 
		/// using a gravity-augmented constant-acceleration profile.
		/// </summary>
		public static float ComputeRendezvousTimeOfFlight(double2 currentPos, double2 currentVel, double2 targetPos, double2 targetVel, double mass, float maxForce, double mu, PhysicsBody parentSoI, float arrivalThreshold = 10.0f)
		{
			double2 R = targetPos - currentPos;
			double2 V = targetVel - currentVel;
			double distance = math.length(R);
			double relSpeed = math.length(V);

			if (distance <= arrivalThreshold && relSpeed < 0.5) return 0f;

			float acceleration = (float)(maxForce / mass);
			if (acceleration <= 0f || float.IsNaN(acceleration)) return 1f;

			// Constant-acceleration rendezvous time of flight (bridges distance R and matches relative velocity V under max acceleration a)
			float t = (float)((relSpeed + math.sqrt(relSpeed * relSpeed + 4.0 * acceleration * distance)) / (2.0 * acceleration));

			if (float.IsNaN(t) || float.IsInfinity(t) || t <= 0.05f) t = 0.05f;
			return t;
		}

		public bool ComputeRendezvousTrajectory()
		{
			BurnOrientation = 0f;
			
			double mass = OwnerEntity.Mass;
			double2 currentPos = OwnerEntity.Position;
			double2 currentVel = OwnerEntity.Velocity;

			if (mass <= 0.0) return false;

			UpdateTargetState();

			if (double.IsNaN(TargetPosition.x) || double.IsNaN(TargetPosition.y)) return false;

			double2 R = TargetPosition - currentPos;
			double2 V = TargetVelocity - currentVel;
			double distance = math.length(R);
			double relSpeed = math.length(currentVel - TargetVelocity);

			if (distance <= ArrivalThreshold && relSpeed < 1.0)
			{
				TimeOfFlight = 0f;
				return false;
			}

			float acceleration = (float)(MaxForce / mass);
			if (acceleration <= 0f || float.IsNaN(acceleration)) return false;

			PhysicsBody parentSoI = OwnerEntity.DominantEntity?.PhysicsBody ?? null;
			double mu = OwnerEntity.PhysicsBody != null ? OwnerEntity.PhysicsBody.GravParam : 0.0;

			// Compute and save to TimeOfFlight using the robust kinematic solver
			TimeOfFlight = ComputeRendezvousTimeOfFlight(currentPos, currentVel, TargetPosition, TargetVelocity, mass, MaxForce, mu, parentSoI, ArrivalThreshold);

			float tgo = TimeOfFlight;
			if (float.IsNaN(tgo) || float.IsInfinity(tgo) || tgo <= 0.05f) tgo = 0.05f;

			// Compute local gravitational acceleration
			double2 gravityAcc = double2.zero;
			if (parentSoI != null)
			{
				double2 rVec = currentPos - parentSoI.Position;
				double rDist = math.length(rVec);
				if (rDist > 1.0)
				{
					gravityAcc = -mu * rVec / (rDist * rDist * rDist);
				}
			}

			// Gravity-compensated Zero Effort Miss (ZEM)
			double2 ZEM = R + (V * tgo) + (0.5 * gravityAcc * tgo * tgo);

			// Constant-acceleration ZEM/ZEV guidance law
			double2 requiredVelChange = (6.0 * ZEM / (tgo * tgo)) - (4.0 * V / tgo);

			double len = math.length(requiredVelChange);
			if (len > 1e-5 && !double.IsNaN(len) && !double.IsInfinity(len))
			{
				double2 burnDir = requiredVelChange / len;
				if (!double.IsNaN(burnDir.x) && !double.IsNaN(burnDir.y))
				{
					BurnOrientation = math.atan2((float)burnDir.y, (float)burnDir.x);
					return true;
				}
			}

			if (distance > 1e-5)
			{
				double2 burnDir = R / distance;
				BurnOrientation = math.atan2((float)burnDir.y, (float)burnDir.x);
				return true;
			}

			return false;
		}

		/// <summary>
		/// Computes the trajectory adjustment for relative position hold (station-keeping) relative to the target entity or position.
		/// </summary>
		public bool ComputeStationKeepingTrajectory()
		{
			double mass = OwnerEntity.Mass;
			double2 currentPos = OwnerEntity.Position;
			double2 currentVel = OwnerEntity.Velocity;

			if (mass <= 0.0) return false;

			double2 targetPos = TargetEntity != null && !TargetEntity.IsDestroyed ? TargetEntity.Position : TargetPosition;
			double2 targetVel = TargetEntity != null && !TargetEntity.IsDestroyed ? TargetEntity.Velocity : TargetVelocity;

			if (double.IsNaN(targetPos.x) || double.IsNaN(targetPos.y)) return false;

			double2 R = targetPos - currentPos; // Relative position error
			double2 V = currentVel - targetVel; // Relative velocity error

			// PD controller for maintaining relative position and nulling relative velocity
			double2 requiredVelChange = R - (V * 2.0);

			double len = math.length(requiredVelChange);
			if (len > 1e-5 && !double.IsNaN(len) && !double.IsInfinity(len))
			{
				double2 burnDir = requiredVelChange / len;
				if (!double.IsNaN(burnDir.x) && !double.IsNaN(burnDir.y))
				{
					BurnOrientation = math.atan2((float)burnDir.y, (float)burnDir.x);
					return true;
				}
			}

			return false;
		}
	
		/// <summary>
		/// Computes the trajectory to match a defined target orbit's energy and angular momentum invariants, 
		/// ensuring exact geometric orientation without rotational offset.
		/// </summary>
		public bool ComputeOrbitMatchTrajectory()
		{
			double mass = OwnerEntity.Mass;
			double2 currentPos = OwnerEntity.Position;
			double2 currentVel = OwnerEntity.Velocity;

			RequiredOrbitMatchDeltaV = 0.0;
			if (mass <= 0.0) return false;

			PhysicsBody parentSoI = OwnerEntity.DominantEntity?.PhysicsBody ?? null;
			if (parentSoI == null) return false;

			// Dynamically compute absolute target position from relative offset if manually placed
			double2 tPos = TargetOrbitEntity != null && !TargetOrbitEntity.IsDestroyed 
				? TargetOrbitEntity.Position 
				: parentSoI.Position + TargetOrbitPositionOffset;
			
			double2 tVel = TargetOrbitEntity != null && !TargetOrbitEntity.IsDestroyed 
				? TargetOrbitEntity.Velocity 
				: TargetOrbitVelocity;

			if (double.IsNaN(tPos.x) || double.IsNaN(tPos.y) || double.IsInfinity(tPos.x) || double.IsInfinity(tPos.y) ||
				double.IsNaN(tVel.x) || double.IsNaN(tVel.y) || double.IsInfinity(tVel.x) || double.IsInfinity(tVel.y))
				return false;

			double mu = OwnerEntity.PhysicsBody.GravParam;
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
			double2 vCurrRel = currentVel - parentSoI.Velocity;
			double rCurrDist = math.length(rCurrRel);
			if (rCurrDist <= 1.0) return false;

			double2 uR = rCurrRel / rCurrDist;
			double2 uTheta = new double2(-uR.y, uR.x);

			double targetP = (targetH * targetH) / mu;
			double radiusDenominator = 1.0 + math.dot(targetEccVec, uR);
			if (radiusDenominator <= 1e-6) return false;
			double targetRadiusAtCurrentAngle = targetP / radiusDenominator;

			double targetRadialVelocity = -(mu / targetH) * math.dot(targetEccVec, uTheta);

			double maxAccel = MaxForce / mass;
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

			double len = math.length(requiredVelChange);
			if (len > 1e-8 && !double.IsNaN(len) && !double.IsInfinity(len))
			{
				RequiredOrbitMatchDeltaV = len;
				double2 burnDir = requiredVelChange / len;
				if (!double.IsNaN(burnDir.x) && !double.IsNaN(burnDir.y) && !double.IsInfinity(burnDir.x) && !double.IsInfinity(burnDir.y))
				{
					BurnOrientation = math.atan2((float)burnDir.y, (float)burnDir.x);
					return true;
				}
			}

			return false;
		}

		public void ResetOrbitMatchGuidance()
		{
			RequiredOrbitMatchDeltaV = 0.0;
			_smoothedOrbitMatchBurnDirection = double2.zero;
			_hasSmoothedOrbitMatchBurnDirection = false;
		}

		/// <summary>
		/// Numerically propagates the powered intercept trajectory over time to generate a renderable path.
		/// </summary>
		public static List<double2> PredictInterceptPath(double2 currentPos, double2 currentVel, double2 targetPos, double2 targetVel, double mass, float maxForce, double mu, double timeOfFlight, PhysicsBody parent, out double2 finalVel, int steps = 150, float arrivalThreshold = 10f)
		{
			List<double2> path = new List<double2>();
			finalVel = currentVel;
			if (mass <= 0.0) return path;

			double2 pos = currentPos;
			float acceleration = maxForce / (float)mass;

			double dt = timeOfFlight / steps;
			path.Add(pos);

			for (int i = 0; i < steps; i++)
			{
				double2 R = targetPos - pos;
				double distance = math.length(R);

				if (distance <= arrivalThreshold)
				{
					break;
				}

				// Re-evaluate intercept guidance dynamically at each step (matching ComputeInterceptTrajectory)
				double2 V = targetVel - finalVel;
				double2 gravityAcc = double2.zero;
				if (parent != null)
				{
					double2 rVec = pos - parent.Position;
					double rDist = math.length(rVec);
					if (rDist > 1.0)
					{
						gravityAcc = -mu * rVec / (rDist * rDist * rDist);
					}
				}

				double tRem = math.max(0.1, timeOfFlight - (i * dt));
				double2 R_corrected = R - (0.5 * gravityAcc * (tRem * tRem));
				double2 requiredVelChange = R_corrected + (V * tRem);

				double len = math.length(requiredVelChange);
				float burnAngle = 0f;
				if (len > 1e-5 && !double.IsNaN(len) && !double.IsInfinity(len))
				{
					double2 burnDir = requiredVelChange / len;
					burnAngle = math.atan2((float)burnDir.y, (float)burnDir.x);
				}
				else if (distance > 1e-5)
				{
					burnAngle = math.atan2((float)R.y, (float)R.x);
				}

				// 1. Compute thrust acceleration vector
				double2 thrustAcc = new double2(math.cos(burnAngle), math.sin(burnAngle)) * acceleration;

				double2 totalAcc = thrustAcc + gravityAcc;

				// 2. Integration step for vehicle
				finalVel += totalAcc * dt;
				pos += finalVel * dt;

				// Propagate target position and velocity forward
				double2 targetGravityAcc = double2.zero;
				if (parent != null)
				{
					double2 rVecTarget = targetPos - parent.Position;
					double rDistTarget = math.length(rVecTarget);
					if (rDistTarget > 1.0)
					{
						targetGravityAcc = -mu * rVecTarget / (rDistTarget * rDistTarget * rDistTarget);
					}
				}
				targetVel += targetGravityAcc * dt;
				targetPos += targetVel * dt;

				path.Add(pos);
			}

			return path;
		}

		/// <summary>
		/// Numerically propagates the powered rendezvous trajectory over time to generate a renderable path.
		/// </summary>
		public List<double2> PredictRendezvousPath(double maxTime = -1.0, int steps = 150)
		{
			List<double2> path = new List<double2>();
			if (OwnerEntity == null || OwnerEntity.IsDestroyed || OwnerEntity.Mass <= 0.0) return path;

			double2 pos = OwnerEntity.Position;
			double2 vel = OwnerEntity.Velocity;
			double mass = OwnerEntity.Mass;
			float acceleration = MaxForce / (float)mass;

			double effectiveMaxTime = (maxTime > 0.0) ? maxTime : (TimeOfFlight > 0f ? TimeOfFlight : 3600.0);
			double dt = effectiveMaxTime / steps;
			path.Add(pos);

			PhysicsBody parentSoI = OwnerEntity.DominantEntity?.PhysicsBody ?? null;
			double mu = OwnerEntity.PhysicsBody != null ? OwnerEntity.PhysicsBody.GravParam : 0.0;

			double2 targetPos = TargetPosition;
			double2 targetVel = TargetVelocity;

			for (int i = 0; i < steps; i++)
			{
				double2 R = targetPos - pos;
				double2 V = targetVel - vel;
				double distance = math.length(R);

				if (distance <= 2000)
				{
					break;
				}

				// Re-evaluate active rendezvous guidance vector at current simulated state
				double2 requiredVelChange;
				if (distance < 1500.0)
				{
					double alpha = math.min(0.1, 0.02 + (distance / 1500.0) * 0.03);
					double2 desiredRelVel = R * alpha;
					double2 actualRelVel = vel - targetVel;
					requiredVelChange = -(actualRelVel - desiredRelVel);
				}
				else
				{
					double closingSpeed = math.dot(R, -V) / math.max(1.0, distance);
					float tgo = (float)math.max(0.05, effectiveMaxTime - (i * dt));
					if (float.IsNaN(tgo) || float.IsInfinity(tgo) || tgo <= 0.05f) tgo = 1.0f;

					double2 ZEM = R + (V * tgo);
					requiredVelChange = (6.0 * ZEM / (tgo * tgo)) - (4.0 * V / tgo);
				}

				double2 burnDir = double2.zero;
				double len = math.length(requiredVelChange);
				if (len > 1e-5 && !double.IsNaN(len) && !double.IsInfinity(len))
				{
					burnDir = requiredVelChange / len;
				}
				else if (distance > 1e-5)
				{
					burnDir = R / distance;
				}

				// 1. Compute thrust acceleration vector
				double2 thrustAcc = burnDir * acceleration;

				// 2. Compute gravitational acceleration for vehicle
				double2 gravityAcc = double2.zero;
				if (parentSoI != null)
				{
					double2 rVec = pos - parentSoI.Position;
					double rDist = math.length(rVec);
					if (rDist > 1.0)
					{
						gravityAcc = -mu * rVec / (rDist * rDist * rDist);
					}
				}

				double2 totalAcc = thrustAcc + gravityAcc;

				// 3. Integration step for vehicle
				vel += totalAcc * dt;
				pos += vel * dt;

				// Propagate target position and velocity forward
				double2 targetGravityAcc = double2.zero;
				if (parentSoI != null)
				{
					double2 rVecTarget = targetPos - parentSoI.Position;
					double rDistTarget = math.length(rVecTarget);
					if (rDistTarget > 1.0)
					{
						targetGravityAcc = -mu * rVecTarget / (rDistTarget * rDistTarget * rDistTarget);
					}
				}
				targetVel += targetGravityAcc * dt;
				targetPos += targetVel * dt;

				path.Add(pos);
			}

			return path;
		}
	}

	public enum EntityType : int
	{
		CelestialBody,
		Asteroid,
		Vehicle,
		Missile,
		Projectile
	}


	public class Entity
	{
		private PhysicsBody _physicsBody;
		private bool _isOwned;
		private string _name;
		private double _dryMass;
		private double _maxFuelMass;
		private double _addedMass;
		private float _fuelPercent;
		private bool _isTracked = false;
		private bool _isKnown;
		private bool _isObserved;
		private Entity _dominantEntity;
		private EntityType _entityType;
		private EntityGroup _group;
		private int _numSensors;
		private AutonomyController _autonomyController;

		private List<Entity> _observedEntities = new List<Entity>();
		private List<IEntityCapability> _capabilities = new List<IEntityCapability>();
		private LaunchCapability _launchCapability;
		private MissileCapability _missileCapability;
		private NavigationCapability _navigationCapability;

		private double2 _lastObservedPosition;
		private double2 _lastObservedVelocity;
		private double _lastObservedTime;

		public PhysicsBody PhysicsBody => _physicsBody;
		public bool IsOwned => _isOwned;
		public string Name => _name;
		public bool IsTracked => _isTracked;
		public bool IsKnown => _isKnown || _isOwned;
		public bool IsObserved => _isObserved || _isOwned;
		public Entity DominantEntity => _dominantEntity;
		public double2 Position => _physicsBody.Position;
		public double2 Velocity => _physicsBody.Velocity;
		public double Radius => _physicsBody.Radius;
		public double Mass => _physicsBody.Mass;
		public float Orientation => _physicsBody.Orientation;
		public float AngularVelocity => _physicsBody.AngularVelocity;
		public bool IsSpawned => !_physicsBody.IsPending;
		public bool IsDestroyed => _physicsBody.IsDestroyed;
		public bool IsCelestial => _physicsBody.IsCelestial;
		public bool IsAsteroid => _entityType == EntityType.Asteroid;
		public bool IsVehicle => _entityType == EntityType.Vehicle;
		public bool IsMissile => _entityType == EntityType.Missile;
		public bool IsProjectile => _entityType == EntityType.Projectile;
		public bool IsLauncher => _launchCapability != null;
		public bool IsMissileLauncher => _missileCapability != null;
		public bool IsNavigable => _navigationCapability != null;
		public LaunchCapability LaunchCapability => _launchCapability;
		public MissileCapability MissileCapability => _missileCapability;
		public NavigationCapability NavigationCapability => _navigationCapability;
		public List<Entity> ObservedEntities => _observedEntities;
		public EntityType Type => _entityType;
		public CompositionProfile Composition => _physicsBody.Composition;
		public EntityGroup Group => _group;
		public bool InGroup => _group != null && !_group.IsDestroyed;
		public int NumSensors => _numSensors;
		public bool IsAutonomouslyRunning => _autonomyController.IsRunning;

		public Entity(PhysicsBody body, string name, EntityType type, bool isOwned = false)
		{
			// Input validation: Body reference is strictly required for entity evaluation
			if (body == null) throw new ArgumentNullException(nameof(body), "Entity initialization requires a valid PhysicsBody.");
			
			_physicsBody = body;
			_isOwned = isOwned;
			_isKnown = isOwned;
			_isObserved = isOwned;
			_name = name;
			_entityType = type;

			_autonomyController = new AutonomyController(this);
			_autonomyController.OnRunningChanged += EntitiesManager.Instance.NotifyAutonomous;

			UpdateDominantEntity(_physicsBody.ParentSoI);
			_physicsBody.OnSoIChanged += UpdateDominantEntity;
			_physicsBody.OnDestroyed += Remove;
		}

		public void RemoveManual()
		{
			_physicsBody.Remove();
			Remove();
		}

		public void Remove()
		{
			_physicsBody.OnSoIChanged -= UpdateDominantEntity;
			_autonomyController.OnRunningChanged -= EntitiesManager.Instance.NotifyAutonomous;
			_observedEntities.Clear();
			EntitiesManager.Instance.UnregisterEntity(this);
		}

		public void MarkObserved()
		{
			_isKnown = true;
			_isObserved = true;
		}

		public void MarkUnobserved()
		{
			_isObserved = false;
			_lastObservedPosition = _physicsBody.Position;
			_lastObservedVelocity = _physicsBody.Velocity;
			_lastObservedTime = PhysicsManager2D.Instance.SimulationElapsedTime;
		}

		#region Capabilities Management

		public void AddCapability(IEntityCapability capability)
		{
			if (capability == null) throw new ArgumentNullException(nameof(capability));

			if (capability is LaunchCapability launchCap)
			{
				if (_launchCapability != null)
				{
					Debug.Log($"[{GetType().Name}, Entity {_name}] already has a launch capability, can't add new one.");
					return;
				}
				
				_launchCapability = launchCap;
			}

			if (capability is MissileCapability misCap)
			{
				if (_missileCapability != null)
				{
					Debug.Log($"[{GetType().Name}, Entity {_name}] already has a missile capability, can't add new one.");
					return;
				}
				
				_missileCapability = misCap;
			}

			// Automatically compute attitude gains using vehicle inertia tensor when adding NavigationCapability
			if (capability is NavigationCapability navCap)
			{
				if (_navigationCapability != null)
				{
					Debug.Log($"[{GetType().Name}, Entity {_name}] already has a navigation capability, can't add new one.");
					return;
				}

				_navigationCapability = navCap;
				_navigationCapability.OwnerEntity = this;
			}

			if (capability is SensorCapability senCap)
			{
				_numSensors++;
			}

			_capabilities.Add(capability);
			EntitiesManager.Instance?.NotifyCapabilityAdded(this, capability);

			Debug.Log($"[{GetType().Name}, Entity {_name}] Added capability: {capability.GetType().Name}");
		}

		public T GetCapability<T>() where T : class, IEntityCapability
		{
			foreach (var cap in _capabilities)
			{
				if (cap is T match) return match;
			}
			return null;
		}

		public IEnumerable<T> GetCapabilities<T>() where T : class, IEntityCapability
		{
			foreach (var cap in _capabilities)
			{
				if (cap is T match) yield return match;
			}
		}

		public bool HasCapability<T>() where T : class, IEntityCapability
		{
			return GetCapability<T>() != null;
		}

		#endregion

		public void SetTracked(bool isTracked)
		{
			_isTracked = isTracked;
			_physicsBody.SetTracked(isTracked);
		}

		public void Fracture() => _physicsBody.Fracture();

		private void UpdateDominantEntity(PhysicsBody parentSoI)
		{
			if (parentSoI == null)
			{
				_dominantEntity = null;
				return;
			}
			_dominantEntity = EntitiesManager.Instance.EntityLookup[parentSoI.Name];
		}

		private void UpdatePhysicsBodyMass()
		{
			double totalMass = _dryMass + _addedMass + _maxFuelMass * _fuelPercent;
			_physicsBody.SetMass(totalMass);
		}

		/// <summary>
		/// Queries all other entities visible to this entity's active sensors using the spatial hash map.
		/// </summary>
		public void UpdateObservedEntities()
		{
			_observedEntities.Clear();

			if (PhysicsManager2D.Instance == null || !PhysicsManager2D.Instance.IsInitialized)
				return;

			var sensors = GetCapabilities<SensorCapability>();
			var sensorList = sensors != null ? new List<SensorCapability>(sensors) : new List<SensorCapability>();

			if (sensorList.Count == 0) return;

			PhysicsManager2D.Instance.QuerySpatialHash((spatialMap, bodyStateVectors, cellSize) =>
			{
				double maxRange = 0.0;
				for (int i = 0; i < sensorList.Count; i++)
				{
					if (sensorList[i].Range > maxRange) maxRange = sensorList[i].Range;
				}

				HashSet<int> evaluatedIndices = new HashSet<int>();
				int cellRadius = math.max(1, (int)math.ceil(maxRange / cellSize));
				
				int centerCellX = (int)math.floor(Position.x / cellSize);
				int centerCellY = (int)math.floor(Position.y / cellSize);

				for (int x = -cellRadius; x <= cellRadius; x++)
				{
					for (int y = -cellRadius; y <= cellRadius; y++)
					{
						int hash = SpatialHash.GetCellHash(centerCellX + x, centerCellY + y);

						if (spatialMap.TryGetFirstValue(hash, out int bodyIndex, out var iterator))
						{
							do
							{
								if (bodyIndex == _physicsBody.Index || evaluatedIndices.Contains(bodyIndex)) continue;
								evaluatedIndices.Add(bodyIndex);

								if (bodyIndex < 0 || bodyIndex >= bodyStateVectors.Length) continue;

								StateVector targetStateVector = bodyStateVectors[bodyIndex];
								PhysicsBody targetBody = PhysicsManager2D.Instance.PhysicsBodies[bodyIndex];
								if (targetBody == null || targetBody.IsDestroyed) continue;

								double targetRadius = targetBody.Radius;
								double targetMass = targetBody.Mass;
								double2 targetPos = targetStateVector.Position;
								double2 dir = targetPos - Position;
								double dist = math.length(dir);

								for (int s = 0; s < sensorList.Count; s++)
								{
									var sensor = sensorList[s];
									if (dist > sensor.Range || dist <= 1e-5) continue;

									// Field of View Check
									if (sensor.FieldOfViewDegrees < 360f)
									{
										float angleToTarget = math.atan2((float)dir.y, (float)dir.x);
										float heading = Orientation + sensor.OrientationOffset;
										float deltaAngle = angleToTarget - heading;

										while (deltaAngle > (float)math.PI) deltaAngle -= 2.0f * (float)math.PI;
										while (deltaAngle < -(float)math.PI) deltaAngle += 2.0f * (float)math.PI;

										if (math.abs(deltaAngle) > (sensor.FieldOfViewDegrees * 0.5f * math.PI / 180.0f))
										{
											continue;
										}
									}

									// Evaluate Distance Fall-off & Detection Strength by Sensor Type (matching FilterEntitiesJob)
									double detectionScore = 0.0;
									switch (sensor.Type)
									{
										case SensorType.Optical:
											detectionScore = (targetRadius * targetRadius) / (dist * dist) * 1e4 * sensor.Sensitivity;
											break;
										case SensorType.Infrared:
											detectionScore = (targetMass / 1000.0) / (dist * dist) * 1e8 * sensor.Sensitivity;
											break;
										case SensorType.Radar:
											double distSq = dist * dist;
											detectionScore = (targetRadius * targetRadius) / (distSq * distSq) * 1e15 * sensor.Sensitivity;
											break;
									}

									// Threshold for successful contact detection matching the visualizer bounds
									if (detectionScore >= 1.0)
									{
										if (EntitiesManager.Instance != null && 
											EntitiesManager.Instance.EntityBodyLookup.TryGetValue(targetBody, out Entity matchedEntity))
										{
											if (!_observedEntities.Contains(matchedEntity))
											{
												_observedEntities.Add(matchedEntity);
											}
										}
										break; // Successfully detected by at least one sensor
									}
								}

							} while (spatialMap.TryGetNextValue(out bodyIndex, ref iterator));
						}
					}
				}
			});
		}

		#region Autonomy Controls

		public void EnqueueTask(IAutonomousTask task, bool addToLoopTemplate = false)
		{
			if (_autonomyController == null)
			{
				_autonomyController = new AutonomyController(this);
			}
			_autonomyController.EnqueueTask(task, addToLoopTemplate);
		}

		public void StartAutonomy(bool loop = false)
		{
			if (_autonomyController == null) return;
			_autonomyController.IsLooping = loop;
			_autonomyController.StartAutonomy();
		}

		public void StopAutonomy() => _autonomyController?.StopAutonomy();
		
		public void ClearTasks() => _autonomyController?.ClearTasks();

		public void EvaluateAutonomy(float dt) => _autonomyController?.EvaluateAutonomy(dt);

		public void StartMiningLoop(Entity station, Entity asteroid)
		{
			if (station == null || station.IsDestroyed || asteroid == null || asteroid.IsDestroyed) return;

			// Setup harvesting and deposit cycle
			EnqueueTask(new NavigateTask(asteroid, isRendezvous: true), addToLoopTemplate: true);
			EnqueueTask(new ExtractMassTask(_autonomyController, asteroid, extractionAmount: 200.0), addToLoopTemplate: true);
			EnqueueTask(new NavigateTask(station, isRendezvous: true), addToLoopTemplate: true);
			EnqueueTask(new DepositMassTask(_autonomyController, station), addToLoopTemplate: true);

			// Start with continuous infinite looping enabled
			StartAutonomy(loop: true);
		}

		public void StartMiningLoopHard()
		{
			var station = EntitiesManager.Instance.EntityLookup["Mining Station"];
			var asteroid = EntitiesManager.Instance.EntityLookup["M-Type Asteroid"];

			StartMiningLoop(station, asteroid);
		}

		#endregion

		#region Grouping

		public void FormGroup(params Entity[] otherEntities)
		{
			new EntityGroup(this, otherEntities);
		}

		public bool JoinGroup(EntityGroup group)
		{
			if (InGroup) LeaveGroup();

			if (group.AddEntity(this))
			{
				return true;
			}

			return false;
		}

		public void LeaveGroup()
		{
			_group.RemoveEntity(this);
		}

		public void SetGroup(EntityGroup group)
		{
			_group = group;
		}

		#endregion

		#region Navigation Controls

		public void HandleNavigationTask(IAutonomousTask task, bool enqueue)
		{
			var navCap = GetCapability<NavigationCapability>();
			if (navCap == null)
			{
				Debug.LogWarning($"[{GetType().Name}] {Name} lacks a NavigationCapability.");
				return;
			}

			if (!enqueue)
			{
				ClearTasks();
			}

			EnqueueTask(task);

			if (_autonomyController != null && !_autonomyController.IsRunning)
			{
				StartAutonomy();
			}
		}

		public void InterceptEntity(Entity targetEntity, float arrivalThreshold = 10f, float timeoutSeconds = float.PositiveInfinity, bool enqueue = false)
		{
			if (targetEntity == null)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot navigate: Target entity is null.");
				return;
			}
			HandleNavigationTask(new NavigateTask(targetEntity, isRendezvous: false, arrivalThreshold, timeoutSeconds), enqueue);
		}

		public void InterceptPosition(double2 targetPosition, float arrivalThreshold = 10f, float timeoutSeconds = 300f, bool enqueue = false)
		{
			HandleNavigationTask(new NavigateTask(targetPosition, isRendezvous: false, arrivalThreshold, timeoutSeconds), enqueue);
		}

		public void RendezvousWithEntity(Entity targetEntity, float arrivalThreshold = 10f, float timeoutSeconds = float.PositiveInfinity, bool enqueue = false)
		{
			if (targetEntity == null)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot navigate: Target entity is null.");
				return;
			}
			HandleNavigationTask(new NavigateTask(targetEntity, isRendezvous: true, arrivalThreshold, timeoutSeconds), enqueue);
		}

		public void RendezvousWithPosition(double2 targetPosition, float arrivalThreshold = 10f, float timeoutSeconds = 300f, bool enqueue = false)
		{
			HandleNavigationTask(new NavigateTask(targetPosition, isRendezvous: true, arrivalThreshold, timeoutSeconds), enqueue);
		}

		public void StationKeepWithEntity(Entity targetEntity, float timeoutSeconds = float.PositiveInfinity, bool enqueue = false)
		{
			if (targetEntity == null)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot station keep: Target entity is null.");
				return;
			}
			HandleNavigationTask(new StationKeepTask(targetEntity, timeoutSeconds), enqueue);
		}

		public void StationKeepWithPosition(double2 targetPosition, float timeoutSeconds = float.PositiveInfinity, bool enqueue = false)
		{
			HandleNavigationTask(new StationKeepTask(targetPosition, timeoutSeconds), enqueue);
		}

		public void OrbitMatchWithEntity(Entity targetEntity, float timeoutSeconds = float.PositiveInfinity, bool enqueue = false)
		{
			if (targetEntity == null) return;
			HandleNavigationTask(new OrbitMatchTask(targetEntity, timeoutSeconds), enqueue);
		}

		public void OrbitMatchWithState(double2 targetPosition, double2 targetVelocity, float timeoutSeconds = float.PositiveInfinity, bool enqueue = false)
		{
			HandleNavigationTask(new OrbitMatchTask(targetPosition, targetVelocity, timeoutSeconds), enqueue);
		}

		public void SetTargetOrientation(float targetOrientation, float timeoutSeconds = float.PositiveInfinity, bool enqueue = false)
		{
			HandleNavigationTask(new OrientationTask(targetOrientation, timeoutSeconds), enqueue);
		}

		/// <summary>
		/// Disables navigation and attitude holding, then puts the entity into a constant spin 
		/// at the specified angular velocity by applying a single torque impulse.
		/// </summary>
		public void StartConstantSpin(float targetAngularVelocity)
		{
			if (_physicsBody == null || _physicsBody.IsDestroyed)
			{
				Debug.LogWarning($"[{GetType().Name}] Cannot apply spin: PhysicsBody is null or destroyed.");
				return;
			}

			// 1. Disable active navigation and attitude control so they don't counteract the spin
			if (_navigationCapability != null)
			{
				_navigationCapability.EndNavigation();
			}

			// 2. Compute the required torque impulse from body inertia and velocity delta
			double inertia = _physicsBody.Inertia;
			if (inertia <= 0.0 || double.IsNaN(inertia) || double.IsInfinity(inertia))
			{
				inertia = 1.0;
			}

			float currentOmega = AngularVelocity;
			float deltaOmega = targetAngularVelocity - currentOmega;

			// Retrieve live physics timestep for correct impulse scaling
			float dt = Time.fixedDeltaTime;
			PhysicsManager2D manager = PhysicsManager2D.Instance;
			if (manager != null && manager.IsInitialized)
			{
				float scaledStep = manager.ScaledPhysicsTimeStep;
				if (scaledStep > 0.00001f)
					dt = scaledStep;
			}

			float torqueImpulse = (float)(inertia * deltaOmega) / Mathf.Max(0.0001f, dt);

			// 3. Apply the single torque impulse to the physics body
			_physicsBody.AddTorque(torqueImpulse);

			Debug.Log($"[{GetType().Name}] Entity {_name} navigation disabled. Constant spin initiated at {targetAngularVelocity} rad/s (torque impulse: {torqueImpulse:F2}).");
		}

		#endregion
	}
}