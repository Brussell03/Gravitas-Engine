using CustomUI;
using GravitasDemo.Rendering;
using TMPro;
using Unity.Mathematics;
using UnityEngine;

namespace GravitasDemo.Interaction
{
	public class SelectedUIManager : MonoBehaviour
	{
		[SerializeField] private GameObject _selectedPanelContent;
		[SerializeField] private GameObject _entityPanelContent;
		[SerializeField] private GameObject _groupPanelContent;
		[SerializeField] private GameObject _firePanelContent;
		[SerializeField] private GameObject _navPanelContent;

		[SerializeField] private TextMeshProUGUI _nameLabel;
		[SerializeField] private TextMeshProUGUI _compositionLabel;
		[SerializeField] private TextMeshProUGUI _massLabel;
		[SerializeField] private TextMeshProUGUI _radiusLabel;
		[SerializeField] private TextMeshProUGUI _velocityLabel;
		[SerializeField] private TextMeshProUGUI _taskLabel;
		[SerializeField] private TextMeshProUGUI _railgunLabel;
		[SerializeField] private TextMeshProUGUI _missileLabel;

		[SerializeField] private AdvancedInteractable _fractureButton;
		[SerializeField] private AdvancedInteractable _firePanelToggle;
		[SerializeField] private AdvancedInteractable _navPanelToggle;
		[SerializeField] private AdvancedInteractable _fireAutoButton;
		[SerializeField] private AdvancedInteractable _fireMissileButton;

		[Header("Formatting")]
		[Tooltip("Standard numeric format string (e.g., 'F2' for 2 decimal places).")]
		[SerializeField] private string textFormat = "F2";

		private void Awake()
		{
			// Fail loudly if UI bindings are missing to prevent continuous runtime exceptions
			if (_selectedPanelContent == null || _entityPanelContent == null || _groupPanelContent == null || _firePanelContent == null || _navPanelContent == null || _nameLabel == null || _velocityLabel == null || _missileLabel == null || _compositionLabel == null || _taskLabel == null || _radiusLabel == null || _railgunLabel == null)
			{
				Debug.LogError($"[{nameof(SelectedUIManager)}] Missing required UI references. Disabling component.");
				enabled = false;
			}

			SimulationInteractionController.Instance.OnMapObjectSelected += ClearSubPanels;
		}

		private void OnDestroy()
		{
			if (SimulationInteractionController.Instance) SimulationInteractionController.Instance.OnMapObjectSelected -= ClearSubPanels;
		}

		private void Update()
		{
			UpdateSelectedPanel();
		}

		private void ClearSubPanels(MapObject mapObject)
		{
			_firePanelToggle.SetIsOn(false, true);
			_navPanelToggle.SetIsOn(false, true);
		}

		private void UpdateSelectedPanel()
		{
			var selectedEntity = SimulationInteractionController.Instance?.SelectedEntity;
			var selectedGroup = SimulationInteractionController.Instance?.SelectedGroup;

			if (selectedEntity == null && selectedGroup == null)
			{
				_selectedPanelContent.SetActive(false);
				_firePanelContent.SetActive(false);
				_navPanelContent.SetActive(false);
				return;
			}

			if (selectedEntity != null)
			{
				_entityPanelContent.SetActive(true);
				_groupPanelContent.SetActive(false);

				UpdateEntityContent(selectedEntity);
			}
			else
			{
				_entityPanelContent.SetActive(false);
				_groupPanelContent.SetActive(true);

				UpdateGroupContent(selectedGroup);
			}

			if (_selectedPanelContent.activeSelf == false) _selectedPanelContent.SetActive(true);
		}

		private void UpdateEntityContent(Entity entity)
		{
			_nameLabel.text = entity.Name;
			_compositionLabel.text = entity.Composition.Name;
			_massLabel.text = entity.Mass.ToString();
			_radiusLabel.text = entity.Radius.ToString(textFormat);
			_velocityLabel.text = math.length(entity.Velocity).ToString(textFormat);
			_taskLabel.text = entity.CurrentTask?.TaskName ?? "None";
			_railgunLabel.text = entity.IsLauncher ? entity.LaunchCapability.Ammo.ToString() : "None";
			_missileLabel.text = entity.IsMissileLauncher ? entity.MissileCapability.Ammo.ToString() : "None";

			_fractureButton.interactable = entity.PhysicsBody.IsFracturable;
			_fireAutoButton.interactable = entity.IsLauncher;
			_fireMissileButton.interactable = entity.IsMissileLauncher;
			_navPanelToggle.interactable = entity.IsNavigable;
		}

		private void UpdateGroupContent(EntityGroup group)
		{
			_nameLabel.text = group.Name;
		}
	}
}