using Gravitas2D;
using GravitasDemo.Interaction;
using TMPro;
using Unity.Mathematics;
using UnityEngine;

namespace GravitasDemo.Rendering
{
	public class SimulationClosestApproachUI : MonoBehaviour
	{
		[SerializeField] private RectTransform _closestApproachMarker1;
		[SerializeField] private RectTransform _closestApproachMarker2;
		[SerializeField] private TextMeshProUGUI _closestApproachText;
		[SerializeField] private Vector2 _textOffset = new Vector2(0f, 25f);
		[SerializeField] private string _distanceFormat = "E2";

		void Start()
		{
			HideClosestApproachVisuals();
		}

		void LateUpdate()
		{
			if (SimulationInteractionController.Instance == null) return;
			
			PhysicsBody targetedBody = SimulationInteractionController.Instance.TargetedEntity?.PhysicsBody;
			
			// Bail early if there is no target to compare against
			if (targetedBody == null)
			{
				HideClosestApproachVisuals();
				return;
			}

			CloseApproachData caData = default;
			bool hasData = false;

			// Evaluate CA against the standard selected body
			PhysicsBody selectedBody = SimulationInteractionController.Instance.SelectedEntity?.PhysicsBody;
			if (selectedBody != null && selectedBody.ParentSoI == targetedBody.ParentSoI && selectedBody != targetedBody)
			{
				caData = CloseApproachCalculator.Calculate(selectedBody, targetedBody);
				hasData = true;
			}

			// 3. Render resolved UI visuals
			if (hasData && caData.IsValid)
			{
				if (_closestApproachMarker1 != null)
				{
					_closestApproachMarker1.gameObject.SetActive(true);
					SimulationRenderer.Instance.SetUIPositionFromSimulationPosition(_closestApproachMarker1, caData.Body1Position);
				}
				
				if (_closestApproachMarker2 != null)
				{
					_closestApproachMarker2.gameObject.SetActive(true);
					SimulationRenderer.Instance.SetUIPositionFromSimulationPosition(_closestApproachMarker2, caData.Body2Position);
				}
				
				if (_closestApproachText != null)
				{
					_closestApproachText.gameObject.SetActive(true);
					double separationDistance = math.distance(caData.Body1Position, caData.Body2Position);
					_closestApproachText.text = $"{separationDistance.ToString(_distanceFormat)} m";

					SimulationRenderer.Instance.SetUIPositionFromSimulationPosition(_closestApproachText.rectTransform, caData.Body2Position);
					_closestApproachText.rectTransform.localPosition += (Vector3)_textOffset;
				}
			}
			else
			{
				HideClosestApproachVisuals();
			}
		}

		private void HideClosestApproachVisuals()
		{
			if (_closestApproachMarker1 != null) _closestApproachMarker1.gameObject.SetActive(false);
			if (_closestApproachMarker2 != null) _closestApproachMarker2.gameObject.SetActive(false);
			if (_closestApproachText != null) _closestApproachText.gameObject.SetActive(false);
		}
	}
}