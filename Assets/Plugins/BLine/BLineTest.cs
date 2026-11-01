using UnityEngine;

[ExecuteInEditMode]
public class BLineTest : MonoBehaviour
{

	BLine.BLine line;
	private void Start()
	{
		line = BLine.BLine.Circle(100, 1, Color.red);
		line.Draw();
	}

	void Update()
	{
		//BLineManager.Render();
	}
}
