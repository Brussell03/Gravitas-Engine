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

	public class InterceptTask : IAutonomousTask
	{
		public string TaskName => "InterceptTask";
		public TaskStatus Status { get; private set; } = TaskStatus.Pending;

		private readonly Entity _targetEntity;
		private readonly double2 _targetPosition;
		private readonly float _arrivalThreshold;
		private readonly float _timeoutSeconds;
		private float _elapsedTime = 0f;

		public InterceptTask(Entity targetEntity, float arrivalThreshold = 10f, float timeoutSeconds = float.PositiveInfinity)
		{
			_targetEntity = targetEntity ?? throw new ArgumentNullException(nameof(targetEntity));
			_arrivalThreshold = arrivalThreshold;
			_timeoutSeconds = timeoutSeconds;
		}

		public InterceptTask(double2 targetPosition, float arrivalThreshold = 10f, float timeoutSeconds = 300f)
		{
			_targetPosition = targetPosition;
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

			navCap.EnterIntercept(_arrivalThreshold, _targetEntity, _targetPosition);
		}

		public TaskStatus Tick(Entity entity, float dt)
		{
			if (Status != TaskStatus.Running) return Status;

			var navCap = entity.NavigationCapability;

			// Check for failures
			if (navCap == null || !navCap.IsNavigating)
			{
				Status = TaskStatus.Failed;
			}

			_elapsedTime += dt;
			if (_elapsedTime >= _timeoutSeconds)
			{
				Status = TaskStatus.Failed;
				navCap.EndNavigation();
				return Status;
			}

			if (_targetEntity != null && _targetEntity.IsDestroyed)
			{
				Status = TaskStatus.Failed;
				navCap.EndNavigation();
				return Status;
			}

			// Evaluate navigation step directly from the active task
			UpdateNavigation(entity, dt);

			if (navCap != null && navCap.CheckCompleted(dt))
			{
				Status = TaskStatus.Completed;
				navCap.EndNavigation();
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

			if (navCap.TargetEntity != null && navCap.TargetEntity.IsDestroyed)
			{
				navCap.EndNavigation();
				return;
			}

			bool trajectoryComputed = navCap.UpdateIntercept();
			
			navCap.UpdateAttitude(navCap.BurnOrientation, dt);

			if (trajectoryComputed)
			{
				float burnAngle = navCap.BurnOrientation;

				float2 worldForce = new float2(math.cos(burnAngle), math.sin(burnAngle)) * (navCap.MaxForce * navCap.BurnThrottle);
				if (navCap.BurnThrottle > 1e-5f)
					entity.PhysicsBody.AddWorldForce(worldForce);

				Debug.Log(navCap.TimeOfFlight);
			}
		}
	}

	public class RendezvousTask : IAutonomousTask
	{
		public string TaskName => "RendezvousTask";
		public TaskStatus Status { get; private set; } = TaskStatus.Pending;

		private readonly Entity _targetEntity;
		private readonly double2 _targetPosition;
		private readonly float _arrivalThreshold;
		private readonly float _timeoutSeconds;
		private float _elapsedTime = 0f;

		public RendezvousTask(Entity targetEntity, float arrivalThreshold = 10f, float timeoutSeconds = float.PositiveInfinity)
		{
			_targetEntity = targetEntity ?? throw new ArgumentNullException(nameof(targetEntity));
			_arrivalThreshold = arrivalThreshold;
			_timeoutSeconds = timeoutSeconds;
		}

		public RendezvousTask(double2 targetPosition, float arrivalThreshold = 10f, float timeoutSeconds = 300f)
		{
			_targetPosition = targetPosition;
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

			navCap.EnterRendezvous(_arrivalThreshold, _targetEntity, _targetPosition);
		}

		public TaskStatus Tick(Entity entity, float dt)
		{
			if (Status != TaskStatus.Running) return Status;

			var navCap = entity.NavigationCapability;

			// Check for failures
			if (navCap == null || !navCap.IsNavigating)
			{
				Status = TaskStatus.Failed;
			}

			_elapsedTime += dt;
			if (_elapsedTime >= _timeoutSeconds)
			{
				Status = TaskStatus.Failed;
				navCap.EndNavigation();
				return Status;
			}

			if (_targetEntity != null && _targetEntity.IsDestroyed)
			{
				Status = TaskStatus.Failed;
				navCap.EndNavigation();
				return Status;
			}

			// Evaluate navigation step directly from the active task
			UpdateNavigation(entity, dt);

			if (navCap != null && navCap.CheckCompleted(dt))
			{
				Status = TaskStatus.Completed;
				navCap.EndNavigation();
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

			if (navCap.TargetEntity != null && navCap.TargetEntity.IsDestroyed)
			{
				navCap.EndNavigation();
				return;
			}

			bool trajectoryComputed = navCap.UpdateRendezvous(dt);
			
			navCap.UpdateAttitude(navCap.BurnOrientation, dt);

			if (trajectoryComputed)
			{
				float burnAngle = navCap.BurnOrientation;

				float2 worldForce = new float2(math.cos(burnAngle), math.sin(burnAngle)) * (navCap.MaxForce * navCap.BurnThrottle);
				if (navCap.BurnThrottle > 1e-5f)
					entity.PhysicsBody.AddWorldForce(worldForce);

				Debug.Log(navCap.TimeOfFlight);
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

			navCap.EnterStationKeep(_targetEntity, _targetPosition);
		}

		public TaskStatus Tick(Entity entity, float dt)
		{
			if (Status != TaskStatus.Running) return Status;

			var navCap = entity.NavigationCapability;

			// Check for failures
			if (navCap == null || !navCap.IsNavigating)
			{
				Status = TaskStatus.Failed;
			}

			_elapsedTime += dt;
			if (_elapsedTime >= _timeoutSeconds)
			{
				Status = TaskStatus.Failed;
				navCap.EndNavigation();
				return Status;
			}

			if (_targetEntity != null && _targetEntity.IsDestroyed)
			{
				Status = TaskStatus.Failed;
				navCap.EndNavigation();
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

			if (navCap.TargetEntity != null && navCap.TargetEntity.IsDestroyed)
			{
				navCap.EndNavigation();
				return;
			}

			bool trajectoryComputed = navCap.UpdateStationKeep();
			
			navCap.UpdateAttitude(navCap.BurnOrientation, dt);

			if (trajectoryComputed)
			{
				float burnAngle = navCap.BurnOrientation;
				
				float2 vehicleForward = new float2(math.cos(entity.Orientation), math.sin(entity.Orientation));
				float2 burnDir = new float2(math.cos(burnAngle), math.sin(burnAngle));
				float alignment = math.max(0f, math.dot(vehicleForward, burnDir));

				float2 worldForce = vehicleForward * (navCap.MaxForce * navCap.BurnThrottle * alignment);
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

			if (_useEntity)
			{
				navCap.EnterOrbitMatch(_targetOrbitEntity);
			}
			else
			{
				PhysicsBody parentSoI = entity.PhysicsBody != null ? entity.PhysicsBody.ParentSoI : null;
				double2 targetOrbitPositionOffset = parentSoI != null ? _targetOrbitPosition - parentSoI.Position : _targetOrbitPosition;
				navCap.EnterOrbitMatch(_targetOrbitEntity, targetOrbitPositionOffset, _targetOrbitVelocity);
			}
		}

		public TaskStatus Tick(Entity entity, float dt)
		{
			if (Status != TaskStatus.Running) return Status;

			var navCap = entity.NavigationCapability;

			// Check for failures
			if (navCap == null || !navCap.IsNavigating)
			{
				Status = TaskStatus.Failed;
			}

			_elapsedTime += dt;
			if (_elapsedTime >= _timeoutSeconds)
			{
				Status = TaskStatus.Failed;
				navCap.EndNavigation();
				return Status;
			}

			if (_useEntity && _targetOrbitEntity != null && _targetOrbitEntity.IsDestroyed)
			{
				Status = TaskStatus.Failed;
				navCap.EndNavigation();
				return Status;
			}

			UpdateNavigation(entity, dt);

			if (navCap != null && navCap.CheckCompleted(dt))
			{
				Status = TaskStatus.Completed;
				navCap.EndNavigation();
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

			bool trajectoryComputed = navCap.UpdateOrbitMatch(dt);

			navCap.UpdateAttitude(navCap.BurnOrientation, dt);

			if (trajectoryComputed)
			{
				float burnAngle = navCap.BurnOrientation;
				if (navCap.BurnThrottle > 1e-4 && (!navCap.RequireAttitudeAlignmentForThrust || navCap.IsBurnAligned(dt)))
				{
					float2 worldForce = new float2(math.cos(burnAngle), math.sin(burnAngle)) * navCap.MaxForce * (float)navCap.BurnThrottle;
					entity.PhysicsBody.AddWorldForce(worldForce);
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

			var navCap = entity.NavigationCapability;

			// Check for failures
			if (navCap == null || !navCap.IsNavigating)
			{
				Status = TaskStatus.Failed;
			}

			_elapsedTime += dt;
			if (_elapsedTime >= _timeoutSeconds)
			{
				Status = TaskStatus.Failed;
				navCap.EndNavigation();
				return Status;
			}

			UpdateNavigation(entity, dt);

			if (navCap != null && navCap.CheckCompleted(dt))
			{
				Status = TaskStatus.Completed;
				navCap.EndNavigation();
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

			navCap.UpdateAttitude(navCap.TargetOrientation, dt);
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