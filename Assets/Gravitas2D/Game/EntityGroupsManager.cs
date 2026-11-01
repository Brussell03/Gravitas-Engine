using System.Collections.Generic;
using Gravitas2D;
using UnityEngine;

namespace GravitasDemo
{
	public class EntityGroupsManager : MonoBehaviour
	{
		public static EntityGroupsManager Instance { get; private set; }

		private List<EntityGroup> _groups = new List<EntityGroup>();
		private readonly Dictionary<EntityGroup, EntityGroupMapObject> _activeVisuals = new Dictionary<EntityGroup, EntityGroupMapObject>();

		public List<EntityGroup> Groups => _groups;

		private void Start()
		{
			if (Instance != null && Instance != this)
			{
				Destroy(gameObject);
				return;
			}
			Instance = this;

			Debug.Log($"[{GetType().Name}] Initialized");

			PhysicsManager2D.Instance.OnPhysicsUpdated += UpdateGroupPositions;
		}
	
		private void OnDestroy()
		{
			if (Instance == this) Instance = null;

			if (PhysicsManager2D.Instance)
			{
				PhysicsManager2D.Instance.OnPhysicsUpdated -= UpdateGroupPositions;
			}
		}

		private void UpdateGroupPositions()
		{
			foreach (var group in _groups)
			{
				group.UpdateProperties();
			}
		}

		public bool RegisterGroup(EntityGroup group)
		{
			if (group == null) return false;
			if (_groups.Contains(group)) return false;

			_groups.Add(group);

			EntityGroupMapObject newVisual = SimulationRenderer.Instance.GetGroupFromPool(group);
			_activeVisuals.Add(group, newVisual);

			return true;
		}

		public void RemoveGroup(EntityGroup group)
		{
			if (group == null) return;
			_groups.Remove(group);

			if (_activeVisuals.TryGetValue(group, out var visual))
			{
				visual.Deactivate();
				_activeVisuals.Remove(group);
			}
		}

		public EntityGroupMapObject GetVisualForGroup(EntityGroup group)
		{
			return _activeVisuals.TryGetValue(group, out var bodyVisual) ? bodyVisual : null;
		}
	}
}