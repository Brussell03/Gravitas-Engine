using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Events;

namespace CustomUI
{
	[AddComponentMenu("UI/Advanced Toggle Group")]
	[DisallowMultipleComponent]
	public class AdvancedToggleGroup : MonoBehaviour
	{
		[Tooltip("If true, pressing the active toggle will turn it off, leaving no toggles active.")]
		public bool allowSwitchOff = false;

		[Tooltip("If true, uses the manually assigned list of toggles below. If false, automatically scans child toggles.")]
		public bool useManualList = false;

		[SerializeField] private List<AdvancedInteractable> _toggles = new List<AdvancedInteractable>();

		// Tracks specific listeners per-toggle to allow clean unsubscription
		private Dictionary<AdvancedInteractable, UnityAction<bool>> _toggleListeners = new Dictionary<AdvancedInteractable, UnityAction<bool>>();

		// Mutex flag to prevent recursive stack overflows during batch state changes
		private bool _isChangingState = false;

		private void Awake()
		{
			RefreshToggles();
		}

		/// <summary>
		/// Clears and re-registers state-change listeners. Scans children automatically if manual list is disabled.
		/// </summary>
		public void RefreshToggles()
		{
			// Clean up existing listener assignments
			foreach (var kvp in _toggleListeners)
			{
				if (kvp.Key != null)
				{
					kvp.Key.onValueChanged.RemoveListener(kvp.Value);
				}
			}
			
			_toggleListeners.Clear();

			// If manual list is disabled, scan children automatically
			if (!useManualList)
			{
				_toggles.Clear();
				var childInteractables = GetComponentsInChildren<AdvancedInteractable>(true);
				
				foreach (var interactable in childInteractables)
				{
					if (interactable.interactionMode == AdvancedInteractable.InteractionMode.Toggle)
					{
						_toggles.Add(interactable);
					}
				}
			}

			// Register listeners for all active toggles in the list
			foreach (var interactable in _toggles)
			{
				if (interactable != null && interactable.interactionMode == AdvancedInteractable.InteractionMode.Toggle)
				{
					AdvancedInteractable t = interactable;
					UnityAction<bool> listener = (isOn) => OnToggleValueChanged(t, isOn);
					
					t.onValueChanged.AddListener(listener);
					_toggleListeners.Add(t, listener);
				}
			}

			EnsureValidState();
		}

		/// <summary>
		/// Handles single-active validation when a managed toggle state changes.
		/// </summary>
		private void OnToggleValueChanged(AdvancedInteractable sender, bool isOn)
		{
			if (_isChangingState) return;

			_isChangingState = true;

			if (isOn)
			{
				// Deactivate all other toggles
				foreach (var toggle in _toggles)
				{
					if (toggle != sender && toggle != null && toggle.IsOn)
					{
						toggle.SetIsOn(false, true); 
					}
				}
			}
			else if (!allowSwitchOff)
			{
				// Revert state if attempting to switch off the only active toggle
				bool anyOn = _toggles.Any(t => t != null && t.IsOn);
				if (!anyOn)
				{
					sender.SetIsOn(true, true);
				}
			}

			_isChangingState = false;
			
		}

		/// <summary>
		/// Ensures the group complies with the allowSwitchOff rule upon initialization.
		/// </summary>
		public void EnsureValidState()
		{
			if (allowSwitchOff || _toggles.Count == 0) return;

			bool anyOn = _toggles.Any(t => t != null && t.IsOn);
			if (!anyOn)
			{
				var firstValidToggle = _toggles.FirstOrDefault(t => t != null);
				if (firstValidToggle != null)
				{
					firstValidToggle.SetIsOn(true, true);
				}
			}
		}

		public AdvancedInteractable GetFirstActiveToggle()
		{
			return _toggles.FirstOrDefault(t => t != null && t.IsOn);
		}

		public IEnumerable<AdvancedInteractable> GetActiveToggles()
		{
			return _toggles.Where(t => t != null && t.IsOn);
		}

		public void SetAllTogglesOff()
		{
			bool oldAllowSwitchOff = allowSwitchOff;
			allowSwitchOff = true;

			foreach (var toggle in _toggles)
			{
				if (toggle != null)
				{
					toggle.SetIsOn(false, true);
				}
			}

			allowSwitchOff = oldAllowSwitchOff;
		}
	}
}