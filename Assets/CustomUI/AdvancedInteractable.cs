using System;
using System.Reflection;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;
using UnityEngine.EventSystems;

namespace CustomUI
{
	[AddComponentMenu("UI/Advanced Interactable")]
	[SelectionBase]
	[DisallowMultipleComponent]
	public class AdvancedInteractable : Selectable, IPointerClickHandler, ISubmitHandler
	{
		public enum InteractionMode { Button, Toggle }

		[Header("Interaction Settings")]
		public InteractionMode interactionMode = InteractionMode.Button;
		
		[Tooltip("The graphic to show/hide based on the toggle state (e.g., a checkmark).")]
		public Graphic toggleGraphic;

		[Tooltip("Starting state if set to Toggle. Will be overridden by Variable Binding if used.")]
		[SerializeField] private bool _isOn = false;

		[Header("Events")]
		public UnityEvent onClick;
		public UnityEvent onToggleOn;
		public UnityEvent onToggleOff;
		public UnityEvent<bool> onValueChanged;

		[Header("Variable Binding (Optional)")]
		[Tooltip("Target component containing the boolean variable to sync with.")]
		public Component bindingTarget;
		[Tooltip("Name of the boolean Field or Property to sync with.")]
		public string bindingVariableName;
		[Tooltip("How often (in seconds) to poll the bound variable for external changes.")]
		public float pollingRate = 0.1f;

		// Reflection caching
		private FieldInfo _boundField;
		private PropertyInfo _boundProperty;
		private bool _hasBinding = false;
		private Coroutine _pollingCoroutine;

		public bool IsOn
		{
			get => _isOn;
			set => SetIsOn(value, true);
		}

		protected override void Awake()
		{
			base.Awake();
			InitializeBinding();
		}

		protected override void OnEnable()
		{
			base.OnEnable();
			
			// Resume polling if bound
			if (_hasBinding && Application.isPlaying)
			{
				_pollingCoroutine = StartCoroutine(PollBoundVariableCoroutine());
			}
			
			// Force transition application upon enabling
			UpdateToggleGraphic(true);
			DoStateTransition(currentSelectionState, true);
		}

		protected override void OnDisable()
		{
			base.OnDisable();
			if (_pollingCoroutine != null)
			{
				StopCoroutine(_pollingCoroutine);
				_pollingCoroutine = null;
			}
		}

		#if UNITY_EDITOR
		protected override void OnValidate()
		{
			base.OnValidate();
			// Update visuals in the editor immediately if the isOn toggle is clicked manually
			UpdateToggleGraphic(true);
			DoStateTransition(currentSelectionState, true);
		}
		#endif

		/// <summary>
		/// Evaluates Reflection data and establishes the link to the target variable.
		/// </summary>
		private void InitializeBinding()
		{
			if (bindingTarget == null || string.IsNullOrEmpty(bindingVariableName)) return;

			Type targetType = bindingTarget.GetType();

			// Search for a field, then a property, utilizing flags that cover standard visibility
			_boundField = targetType.GetField(bindingVariableName, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
			if (_boundField == null)
			{
				_boundProperty = targetType.GetProperty(bindingVariableName, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
			}

			if (_boundField == null && _boundProperty == null)
			{
				Debug.LogError($"[{GetType().Name}] Failed to find boolean variable '{bindingVariableName}' on {bindingTarget.GetType().Name}. Binding aborted.", this);
				return;
			}

			// Validate type is boolean
			Type varType = _boundField != null ? _boundField.FieldType : _boundProperty.PropertyType;
			if (varType != typeof(bool))
			{
				Debug.LogError($"[{GetType().Name}] Bound variable '{bindingVariableName}' is of type {varType.Name}, but must be bool. Binding aborted.", this);
				return;
			}

			_hasBinding = true;
		}

		/// <summary>
		/// Coroutine to continuously poll the bound variable and sync the UI if it changes externally.
		/// </summary>
		private IEnumerator PollBoundVariableCoroutine()
		{
			var wait = new WaitForSeconds(pollingRate);
			while (true)
			{
				yield return wait;

				// Validate target component existence to prevent runtime exceptions if destroyed
				if (bindingTarget == null)
				{
					_hasBinding = false;
					yield break;
				}

				bool externalValue = false;
				
				try
				{
					if (_boundField != null)
						externalValue = (bool)_boundField.GetValue(bindingTarget);
					else if (_boundProperty != null && _boundProperty.CanRead)
						externalValue = (bool)_boundProperty.GetValue(bindingTarget);
				}
				catch (Exception e)
				{
					Debug.LogError($"[{GetType().Name}] Error reading bound variable: {e.Message}");
					_hasBinding = false;
					continue;
				}

				if (externalValue != _isOn)
				{
					SetIsOnWithoutNotify(externalValue);
				}
			}
		}

		/// <summary>
		/// Writes the current UI state back to the bound variable.
		/// </summary>
		private void UpdateBoundVariable()
		{
			if (!_hasBinding) return;

			try
			{
				if (_boundField != null)
					_boundField.SetValue(bindingTarget, _isOn);
				else if (_boundProperty != null && _boundProperty.CanWrite)
					_boundProperty.SetValue(bindingTarget, _isOn);
			}
			catch (Exception e)
			{
				Debug.LogError($"[{GetType().Name}] Error writing to bound variable: {e.Message}");
			}
		}

		/// <summary>
		/// Changes the state and fires associated unity events.
		/// </summary>
		public void SetIsOn(bool value, bool sendCallback)
		{
			if (_isOn == value) return;

			_isOn = value;

			UpdateBoundVariable();
			UpdateToggleGraphic(false);

			// Force the visual transition to apply immediately when the state flips
			DoStateTransition(currentSelectionState, false);

			if (sendCallback)
			{
				onValueChanged?.Invoke(_isOn);
				if (_isOn) onToggleOn?.Invoke();
				else onToggleOff?.Invoke();
			}
		}

		/// <summary>
		/// Changes the state silently without firing callbacks, preventing infinite event loops.
		/// </summary>
		public void SetIsOnWithoutNotify(bool value)
		{
			SetIsOn(value, false);
		}

		private void UpdateToggleGraphic(bool instant)
		{
			if (toggleGraphic == null) return;

	#if UNITY_EDITOR
			if (!Application.isPlaying) instant = true;
	#endif

			if (instant)
			{
				toggleGraphic.canvasRenderer.SetAlpha(_isOn ? 1f : 0f);
			}
			else
			{
				toggleGraphic.CrossFadeAlpha(_isOn ? 1f : 0f, colors.fadeDuration, true);
			}
		}
		
		// --- Custom Transition Logic ---

		protected override void DoStateTransition(SelectionState state, bool instant)
		{
			if (interactionMode == InteractionMode.Toggle)
			{
				if (_isOn)
				{
					// Spoof state to Selected if the toggle is ON and the pointer has moved away
					if (state == SelectionState.Normal)
					{
						state = SelectionState.Selected;
					}
				}
				else
				{
					// Spoof state to Normal if the toggle is OFF to prevent EventSystem focus from artificially coloring it
					if (state == SelectionState.Selected)
					{
						state = SelectionState.Normal;
					}
				}
			}
			else if (interactionMode == InteractionMode.Button)
			{
				// Map Selected state to Normal in Button mode so selection focus does not force selected color
				if (state == SelectionState.Selected)
				{
					state = SelectionState.Normal;
				}
			}

			base.DoStateTransition(state, instant);
		}

		// --- Interaction Interfaces ---

		public virtual void OnPointerClick(PointerEventData eventData)
		{
			// Validate event arguments and require left-click execution
			if (eventData == null || eventData.button != PointerEventData.InputButton.Left) return;
			
			ExecuteInteraction();

			// Clear EventSystem focus on click in Button mode to release persistent selection state
			if (interactionMode == InteractionMode.Button && EventSystem.current != null)
			{
				EventSystem.current.SetSelectedGameObject(null);
			}
		}

		public virtual void OnSubmit(BaseEventData eventData)
		{
			ExecuteInteraction();
		}

		private void ExecuteInteraction()
		{
			if (!IsActive() || !IsInteractable()) return;

			if (interactionMode == InteractionMode.Button)
			{
				onClick?.Invoke();
			}
			else if (interactionMode == InteractionMode.Toggle)
			{
				IsOn = !IsOn; // Triggers SetIsOn logic
				onClick?.Invoke(); // Provide standard click event support even when configured as a toggle
			}
		}
	}
}