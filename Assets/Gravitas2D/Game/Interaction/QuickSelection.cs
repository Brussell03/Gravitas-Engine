using GravitasDemo.Rendering;
using TMPro;
using UnityEngine;

namespace GravitasDemo.Interaction
{
	public class QuickSelection : MonoBehaviour
	{
		[SerializeField] private KeyCode _saveKey = KeyCode.LeftShift;
		[SerializeField] private KeyCode _clearKey = KeyCode.LeftControl;
		[SerializeField] private TextMeshProUGUI[] _selectionUIText = new TextMeshProUGUI[10];

		private MapObject[] _savedMapObjects = new MapObject[10];

		void Update()
		{
			// Cleanup destroyed entities automatically
			for (int i = 0; i < _savedMapObjects.Length; i++)
			{
				if (_savedMapObjects[i] != null && _savedMapObjects[i].IsDestroyed)
				{
					_savedMapObjects[i] = null;
				}
			}

			// Poll Number Keys 1 through 9 (Slot Indices 0 to 8)
			for (int keyNum = 1; keyNum <= 9; keyNum++)
			{
				KeyCode numberKey = KeyCode.Alpha0 + keyNum;
				int slotIndex = keyNum - 1; // Key 1 -> Slot 0, Key 9 -> Slot 8

				if (Input.GetKeyDown(numberKey))
				{
					HandleSlotInput(slotIndex);
				}
			}

			// Poll Number Key 0 (Slot Index 9 - last slot in UI layout 1-9, 0)
			if (Input.GetKeyDown(KeyCode.Alpha0))
			{
				HandleSlotInput(9);
			}

			// Synchronize UI labels to reflect slot states
			UpdateSlotUI();
		}

		private void HandleSlotInput(int slotIndex)
		{
			if (Input.GetKey(_saveKey))
			{
				SaveSelectedToSlot(slotIndex);
			}
			else if (Input.GetKey(_clearKey))
			{
				ClearSlot(slotIndex);
			}
			else
			{
				SelectSlot(slotIndex);
			}
		}

		public void SaveSelectedToSlot(int slot)
		{
			if (slot < 0 || slot >= _savedMapObjects.Length) return;

			MapObject entity = SimulationInteractionController.Instance != null
				? SimulationInteractionController.Instance.SelectedMapObject
				: null;
			
			if (entity != null && !entity.IsDestroyed) SaveToSlot(entity, slot);
		}

		public void SaveToSlot(MapObject mapObject, int slot)
		{
			if (slot < 0 || slot >= _savedMapObjects.Length) return;

			// Remove entity from any previous slot if already assigned elsewhere
			for (int i = 0; i < _savedMapObjects.Length; i++)
			{
				if (_savedMapObjects[i] == mapObject)
				{
					_savedMapObjects[i] = null;
				}
			}

			_savedMapObjects[slot] = mapObject;
			Debug.Log($"[{GetType().Name}] Saved '{mapObject.Name}' to slot {slot}.");
		}

		public void ClearSlot(int slot)
		{
			if (slot < 0 || slot >= _savedMapObjects.Length) return;
			_savedMapObjects[slot] = null;
			Debug.Log($"[{GetType().Name}] Cleared quick selection slot {slot}.");
		}

		public void SelectSlot(int slot)
		{
			if (slot < 0 || slot >= _savedMapObjects.Length) return;

			MapObject target = _savedMapObjects[slot];
			if (target != null && !target.IsDestroyed && SimulationInteractionController.Instance != null)
			{
				SimulationInteractionController.Instance.DeselectAll();
				SimulationInteractionController.Instance.SelectMapObject(target);
				Debug.Log($"[{GetType().Name}] Selected '{target.Name}' from slot {slot}.");
			}
		}

		private void UpdateSlotUI()
		{
			if (_selectionUIText == null) return;

			for (int i = 0; i < _selectionUIText.Length && i < _savedMapObjects.Length; i++)
			{
				TextMeshProUGUI label = _selectionUIText[i];
				if (label == null) continue;

				MapObject mapObject = _savedMapObjects[i];
				if (mapObject != null && !mapObject.IsDestroyed)
				{
					label.text = mapObject.Name;
				}
				else
				{
					label.text = "Empty";
				}
			}
		}
	}
}