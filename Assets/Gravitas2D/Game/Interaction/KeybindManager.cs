using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

namespace GravitasDemo
{
	[Serializable]
	public struct Keybind
	{
		public string Name;
		public KeyCode Key;
		public UnityEvent OnPressed;
	}

	public class KeybindManager : MonoBehaviour
	{
		[SerializeField] private Keybind _focusKeybind = new Keybind { Name = "Focus Selected", Key = KeyCode.F };
		[SerializeField] private Keybind _clearSelectionKeybind = new Keybind { Name = "Clear Selected", Key = KeyCode.X };
		[SerializeField] private Keybind _targetKeybind = new Keybind { Name = "Target Selected", Key = KeyCode.R };
		[SerializeField] private Keybind _trackKeybind = new Keybind { Name = "Track Selected", Key = KeyCode.T };
		[SerializeField] private Keybind _pinKeybind = new Keybind { Name = "Pin Selected", Key = KeyCode.G };
		[SerializeField] private Keybind _speedUpKeybind = new Keybind { Name = "Increase Game Speed", Key = KeyCode.Period };
		[SerializeField] private Keybind _speedDownKeybind = new Keybind { Name = "Decrease Game Speed", Key = KeyCode.Comma };
		[SerializeField] private Keybind _pauseKeybind = new Keybind { Name = "Pause Simulation", Key = KeyCode.Space };
		[SerializeField] private Keybind _formGroupKeybind = new Keybind { Name = "Form Group", Key = KeyCode.J };

		// Action dictionary to map keybindings to their respective logic cleanly
		private List<Keybind> _keybinds = new List<Keybind>();

		void Start()
		{
			// Initialize and map the actions
			_keybinds.Add(_focusKeybind);
			_keybinds.Add(_clearSelectionKeybind);
			_keybinds.Add(_targetKeybind);
			_keybinds.Add(_trackKeybind);
			_keybinds.Add(_pinKeybind);
			_keybinds.Add(_speedUpKeybind);
			_keybinds.Add(_speedDownKeybind);
			_keybinds.Add(_pauseKeybind);
			_keybinds.Add(_formGroupKeybind);
		}
	
		private void Update()
		{
			foreach (var keybind in _keybinds)
			{
				if (Input.GetKeyDown(keybind.Key))
				{
					keybind.OnPressed?.Invoke();
				}
			}
		}
	}
}