using System.Collections.Generic;
using Unity.Mathematics;

namespace GravitasDemo
{
	public class EntityGroup
	{
		private List<Entity> _entities;
		private double2 _averagePos;
		private bool _isDestroyed;
		private bool _isKnown;
		private float _formationSeparation = 1000;
		private string _name;
		
		public List<Entity> Entities => _entities;
		public Entity Leader => _entities[0] ?? null;
		public int NumEntities => _entities.Count;
		public double2 Position => _averagePos;
		public bool IsDestroyed => _isDestroyed;
		public bool IsKnown => _isKnown;
		public string Name => _name;

		public EntityGroup(Entity firstEntity, params Entity[] otherEntities)
		{
			_entities = new List<Entity>();

			if (firstEntity != null && !firstEntity.IsDestroyed)
			{
				if (firstEntity.InGroup) firstEntity.LeaveGroup();
				_entities.Add(firstEntity);
				firstEntity.SetGroup(this);
			}

			foreach (var entity in otherEntities)
			{
				if (entity == null || entity.IsDestroyed) continue;
				if (_entities.Contains(entity)) continue; // Allows first entity to also be in other entities without causing duplication
				if (entity.InGroup) entity.LeaveGroup();
				_entities.Add(entity);
				entity.SetGroup(this);
			}

			UpdateProperties();

			EntityGroupsManager.Instance?.RegisterGroup(this);
		}

		public void RemoveGroup()
		{
			for (int i = _entities.Count - 1; i >= 0; i--)
			{
				if (_entities[i] == null) return;
				_entities[i].SetGroup(null);
				_entities.RemoveAt(i);
			}

			EntityGroupsManager.Instance?.RemoveGroup(this);
			_isDestroyed = true;
		}

		public bool AddEntity(Entity entity)
		{
			if (entity == null || entity.IsDestroyed) return false;
			if (_entities.Contains(entity)) return false;

			_entities.Add(entity);
			entity.SetGroup(this);

			UpdateProperties();

			return true;
		}

		public void RemoveEntity(Entity entity)
		{
			if (entity == null) return;
			_entities.Remove(entity);
			entity.SetGroup(null);

			if (_entities.Count <= 1) RemoveGroup();
		}

		public void SetLeader(Entity entity)
		{
			if (entity == null || entity.IsDestroyed) return;

			if (_entities.Contains(entity))
			{
				_entities.Remove(entity);
			}
			_entities.Insert(0, entity);
			entity.SetGroup(this);
		}

		public void UpdateProperties()
		{
			// Update average entity position in group
			_averagePos = double2.zero;
			foreach (var entity in _entities)
			{
				_averagePos += entity.Position;
			}
			_averagePos /= _entities.Count;

			// Check if any entity in the group is known about
			if (!_isKnown)
			{
				foreach (var entity in _entities)
				{
					if (entity.IsKnown)
					{
						_isKnown = true;
						break;
					}
				}
			}
		}

		public double2 GetFormationPosition(Entity entity)
		{
			int index = _entities.IndexOf(entity);
			if (index == -1) return double2.zero;

			return new double2(_formationSeparation * index, 0);
		}
	}
}