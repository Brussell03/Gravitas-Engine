using UnityEngine;

[RequireComponent(typeof(Light))]
public class SunLight : MonoBehaviour
{
	Light sunLight;
	Transform player;

	void Start()
	{
		sunLight = GetComponent<Light>();
		player = Camera.main.transform;
	}
	
	void FixedUpdate()
	{
		//sunLight.transform.LookAt(player.position);
	}
}