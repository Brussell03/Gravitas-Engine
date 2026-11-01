#if UNITY_EDITOR
using UnityEditor;

namespace CustomUI
{
	[CustomEditor(typeof(AdvancedToggleGroup))]
	public class AdvancedToggleGroupEditor : Editor
	{
		public override void OnInspectorGUI()
		{
			serializedObject.Update();

			SerializedProperty allowSwitchOffProp = serializedObject.FindProperty("allowSwitchOff");
			SerializedProperty useManualListProp = serializedObject.FindProperty("useManualList");
			SerializedProperty togglesProp = serializedObject.FindProperty("_toggles");

			EditorGUILayout.PropertyField(allowSwitchOffProp);
			EditorGUILayout.PropertyField(useManualListProp);

			// Display the manual toggle list array only if useManualList is checked
			if (useManualListProp.boolValue)
			{
				EditorGUI.indentLevel++;
				EditorGUILayout.PropertyField(togglesProp, true);
				EditorGUI.indentLevel--;
			}

			serializedObject.ApplyModifiedProperties();
		}
	}
}
#endif