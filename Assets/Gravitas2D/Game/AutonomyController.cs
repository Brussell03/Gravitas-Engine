using System;
using System.Collections.Generic;
using Gravitas2D;
using Unity.Mathematics;
using UnityEngine;

namespace GravitasDemo
{
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

			// Perform mass transfer
			_targetEntity.PhysicsBody.SetMass(_targetEntity.Mass - amountToExtract);
			entity.PhysicsBody.SetMass(entity.Mass + amountToExtract);

			if (_controller != null)
			{
				entity.CargoMass = math.min(entity.MaxCargoCapacity, entity.CargoMass + amountToExtract);
			}

			_extractedTotal += amountToExtract;

			if (_extractedTotal >= _targetExtractionAmount || entity.CargoMass >= entity.MaxCargoCapacity)
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
		private readonly double _depositRatePerSecond;
		private readonly float _maxRange;

		public DepositMassTask(AutonomyController controller, Entity stationEntity, double ratePerSecond = 50.0, float maxRange = 5000.0f)
		{
			_controller = controller ?? throw new ArgumentNullException(nameof(controller));
			_stationEntity = stationEntity ?? throw new ArgumentNullException(nameof(stationEntity));
			_depositRatePerSecond = ratePerSecond;
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

			if (_controller == null || entity.CargoMass <= 0.0)
			{
				Status = TaskStatus.Completed;
				return Status;
			}

			// Calculate deposit amount for this timestep based on rate per second
			double amountToDeposit = math.min(_depositRatePerSecond * (double)dt, entity.CargoMass);
			if (amountToDeposit <= 0.0)
			{
				Status = TaskStatus.Completed;
				return Status;
			}

			// Transfer mass from vehicle cargo to station physics body
			_stationEntity.PhysicsBody.SetMass(_stationEntity.Mass + amountToDeposit);
			entity.CargoMass = math.max(0.0, entity.CargoMass - amountToDeposit);
			entity.PhysicsBody.SetMass(entity.Mass - amountToDeposit);

			if (entity.CargoMass <= 0.0)
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
}