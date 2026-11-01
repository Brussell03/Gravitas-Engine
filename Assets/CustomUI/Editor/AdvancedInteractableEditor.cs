#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.UI;

namespace CustomUI
{
	[CustomEditor(typeof(AdvancedInteractable))]
	[CanEditMultipleObjects]
	public class AdvancedInteractableEditor : SelectableEditor
	{
		private SerializedProperty interactionMode;
		private SerializedProperty _isOn;
		private SerializedProperty toggleGraphic;
		private SerializedProperty onClick;
		private SerializedProperty onToggleOn;
		private SerializedProperty onToggleOff;
		private SerializedProperty onValueChanged;
		private SerializedProperty bindingTarget;
		private SerializedProperty bindingVariableName;
		private SerializedProperty pollingRate;

		protected override void OnEnable()
		{
			base.OnEnable();
			interactionMode = serializedObject.FindProperty("interactionMode");
			_isOn = serializedObject.FindProperty("_isOn");
			toggleGraphic = serializedObject.FindProperty("toggleGraphic");
			onClick = serializedObject.FindProperty("onClick");
			onToggleOn = serializedObject.FindProperty("onToggleOn");
			onToggleOff = serializedObject.FindProperty("onToggleOff");
			onValueChanged = serializedObject.FindProperty("onValueChanged");
			bindingTarget = serializedObject.FindProperty("bindingTarget");
			bindingVariableName = serializedObject.FindProperty("bindingVariableName");
			pollingRate = serializedObject.FindProperty("pollingRate");
		}

		public override void OnInspectorGUI()
		{
			// Draw base Selectable properties (Interactable, Transition, Navigation, etc.)
			base.OnInspectorGUI();

			serializedObject.Update();

			EditorGUILayout.Space();
			EditorGUILayout.LabelField("Advanced Settings", EditorStyles.boldLabel);
			EditorGUILayout.PropertyField(interactionMode);

			bool isToggleMode = interactionMode.enumValueIndex == (int)AdvancedInteractable.InteractionMode.Toggle;

			if (isToggleMode)
			{
				EditorGUILayout.PropertyField(_isOn);
				EditorGUILayout.PropertyField(toggleGraphic);

				EditorGUILayout.Space();
				EditorGUILayout.LabelField("Variable Binding", EditorStyles.boldLabel);
				EditorGUILayout.PropertyField(bindingTarget);
				EditorGUILayout.PropertyField(bindingVariableName);
				EditorGUILayout.PropertyField(pollingRate);
			}

			EditorGUILayout.Space();
			EditorGUILayout.LabelField("Events", EditorStyles.boldLabel);
			EditorGUILayout.PropertyField(onClick);

			if (isToggleMode)
			{
				EditorGUILayout.PropertyField(onToggleOn);
				EditorGUILayout.PropertyField(onToggleOff);
				EditorGUILayout.PropertyField(onValueChanged);
			}

			serializedObject.ApplyModifiedProperties();
		}
	}
}
#endif