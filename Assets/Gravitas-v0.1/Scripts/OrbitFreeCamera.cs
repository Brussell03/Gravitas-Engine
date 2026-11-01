using System.Collections.Generic;
using Gravitas;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class OrbitFreeCamera : MonoBehaviour
{
	const float k_MouseSensitivityMultiplier = 0.01f;

	/// <summary>
	/// Rotation speed when using the mouse.
	/// </summary>
	public float m_LookSpeedMouse = 4.0f;
	/// <summary>
	/// Movement speed.
	/// </summary>
	public float m_MoveSpeed = 10.0f;
	/// <summary>
	/// Scale factor of the turbo mode.
	/// </summary>
	public float m_Turbo = 10.0f;
	/// <summary>
	/// Scroll speed.
	/// </summary>
	public float m_ScrollSpeed = 100000.0f;
	/// <summary>
	/// Zooming speed.
	/// </summary>
	public float m_ZoomSpeed = 10000.0f;
	public Camera m_Camera;

	float m_OrbitRadius = 60.0f;
	float m_TargetOrbitRadius = 60.0f;
	bool m_IsFocused = false;
	Transform m_FocusedObject;
	/// <summary>
	/// Rotation of the camera in orbit mode.
	/// </summary>
	Vector3 m_CamRotation;
	float m_OrbitRadiusMin = 0.2f;
	float m_OrbitRadiusMax = 100.0f;

	private static string kMouseX = "Mouse X";
	private static string kMouseY = "Mouse Y";
	private static string kMouseScroll = "Mouse ScrollWheel";
	private static string kVertical = "Vertical";
	private static string kHorizontal = "Horizontal";

	private static string kYAxis = "YAxis";
	private static string kSpeedAxis = "Speed Axis";

	PhysicsBody physicsBody;
	Rigidbody rb;

	void Start()
	{
		RegisterInputs();

		if (transform.parent) SetFocusedObject(transform.parent);

		rb = GetComponent<Rigidbody>();
		rb.useGravity = false;
		rb.isKinematic = true;

		physicsBody = new PhysicsBody(new StateVector(transform), new VehicleModel(), isKinematic: true, isCelestial: false, name: name, rb: rb);
		physicsBody.DeclarePlayerBody();
	}

	void RegisterInputs()
	{

#if UNITY_EDITOR
		List<InputManagerEntry> inputEntries = new List<InputManagerEntry>();

		// Add new bindings
		inputEntries.Add(new InputManagerEntry { name = kYAxis, kind = InputManagerEntry.Kind.KeyOrButton, btnPositive = "page up", altBtnPositive = "joystick button 5", btnNegative = "page down", altBtnNegative = "joystick button 4", gravity = 1000.0f, deadZone = 0.001f, sensitivity = 1000.0f });
		inputEntries.Add(new InputManagerEntry { name = kYAxis, kind = InputManagerEntry.Kind.KeyOrButton, btnPositive = "q", btnNegative = "e", gravity = 1000.0f, deadZone = 0.001f, sensitivity = 1000.0f });

		inputEntries.Add(new InputManagerEntry { name = kSpeedAxis, kind = InputManagerEntry.Kind.KeyOrButton, btnPositive = "home", btnNegative = "end", gravity = 1000.0f, deadZone = 0.001f, sensitivity = 1000.0f });
		inputEntries.Add(new InputManagerEntry { name = kSpeedAxis, kind = InputManagerEntry.Kind.Axis, axis = InputManagerEntry.Axis.Seventh, gravity = 1000.0f, deadZone = 0.001f, sensitivity = 1000.0f });

		InputRegistering.RegisterInputs(inputEntries);
#endif
	}

	float inputRotateAxisX, inputRotateAxisY;
	float inputChangeSpeed;
	float inputVertical, inputHorizontal;
	float inputScrollDelta;
	bool leftShiftBoost, leftShift;
	float inputMiddleClickRotateAxisX;
	float inputMiddleClickRotateAxisY;

	void UpdateInputs()
	{
		inputRotateAxisX = 0.0f;
		inputRotateAxisY = 0.0f;
		inputScrollDelta = 0.0f;
		inputMiddleClickRotateAxisX = 0.0f;
		inputMiddleClickRotateAxisY = 0.0f;
		leftShiftBoost = false;
		if (Input.GetMouseButton(1))
		{
			leftShiftBoost = true;
			inputRotateAxisX = Input.GetAxis(kMouseX) * m_LookSpeedMouse;
			inputRotateAxisY = Input.GetAxis(kMouseY) * m_LookSpeedMouse;
		}

		if (Input.GetMouseButton(2))
		{
			inputMiddleClickRotateAxisX = Input.GetAxis(kMouseX);
			inputMiddleClickRotateAxisY = Input.GetAxis(kMouseY);
		}

		leftShift = Input.GetKey(KeyCode.LeftShift);

		//inputChangeSpeed = Input.GetAxis(kSpeedAxis);

		inputScrollDelta = Input.GetAxis(kMouseScroll);

		inputVertical = Input.GetAxis(kVertical);
		inputHorizontal = Input.GetAxis(kHorizontal);
	}

	void SetFocusedObject(Transform obj)
	{
		m_FocusedObject = obj;
		transform.parent = obj;
		m_IsFocused = false;
		m_CamRotation = transform.localEulerAngles;

		m_OrbitRadius = Vector3.Distance(m_Camera.transform.position, m_FocusedObject.position);
		m_TargetOrbitRadius = Mathf.Clamp(m_OrbitRadius, m_OrbitRadiusMin, m_OrbitRadiusMax);

		m_OrbitRadiusMin = m_FocusedObject.transform.localScale.x / 2;
		m_OrbitRadiusMax = m_OrbitRadiusMin * 100.0f;
	}

	void Update()
	{

		// If the debug menu is running, we don't want to conflict with its inputs.
		if (UnityEngine.Rendering.DebugManager.instance.displayRuntimeUI)
		{

			if (m_IsFocused)
			{
				Quaternion QT = Quaternion.Euler(m_CamRotation.y, m_CamRotation.x, 0.0f);
				//transform.rotation = QT;
				physicsBody.Look(QT);
			}
			else
			{
				//transform.rotation = Quaternion.identity;
				physicsBody.Look(Quaternion.identity);
			}

			return;
		}

		UpdateInputs();

		if (Input.GetKeyDown(KeyCode.F))
		{
			// Check for focusable object in front of camera
			RaycastHit hit;
			if (Physics.Raycast(transform.position, transform.forward, out hit))
			{
				SetFocusedObject(hit.transform);
			}
			// else
			// {
			// 	SetFocusedObject(FloatingOrigin.TrueOriginTransform);
			// }
		}

		if (Input.GetKeyDown(KeyCode.R) && m_FocusedObject)
		{
			m_IsFocused = !m_IsFocused;

			if (m_IsFocused)
			{ // Switching to Focused mode from Free
				m_OrbitRadius = Vector3.Distance(transform.position, m_FocusedObject.position);
				m_TargetOrbitRadius = Mathf.Clamp(m_OrbitRadius, m_OrbitRadiusMin, m_OrbitRadiusMax);

				transform.LookAt(m_FocusedObject);
				m_CamRotation = new Vector3(Mathf.Clamp(transform.eulerAngles.x, -80.0f, 80.0f), transform.eulerAngles.y, 0.0f);

				physicsBody.MoveTo(m_FocusedObject.position);
				//transform.position = m_FocusedObject.position;

				m_Camera.transform.localPosition = Vector3.back * m_OrbitRadius;
				m_Camera.transform.localRotation = Quaternion.identity;
			}
			else
			{ // Switching to Free mode from Focused
				physicsBody.MoveTo(m_Camera.transform.position);
				//transform.position = m_Camera.transform.position;
				m_Camera.transform.localPosition = Vector3.zero;
			}
		}

		if (m_IsFocused)
		{
			// Camera is in focused mode
			Orbit();
		}
		else
		{
			// Camera is in tracking mode
			// m_CameraPivot.position = new Vector3(m_CameraPivot.parent.position.x, m_CameraPivot.parent.position.y, m_CameraPivot.parent.position.z);
			// m_CameraPivot.rotation = Quaternion.identity;

			Move();

			// if (transform.localPosition.sqrMagnitude > 10000.0f)
			// {
			// 	Vector3 normal = transform.localPosition.normalized;
			// 	transform.localPosition = normal * 100.0f;
			// }
		}
	}

	void Move()
	{

		bool moved = inputRotateAxisX != 0.0f || inputRotateAxisY != 0.0f || inputVertical != 0.0f || inputHorizontal != 0.0f;
		if (moved)
		{
			float rotationX = transform.localEulerAngles.x;
			float newRotationY = transform.localEulerAngles.y + inputRotateAxisX;

			// Weird clamping code due to weird Euler angle mapping...
			float newRotationX = (rotationX - inputRotateAxisY);
			if (rotationX <= 90.0f && newRotationX >= 0.0f)
				newRotationX = Mathf.Clamp(newRotationX, 0.0f, 90.0f);
			if (rotationX >= 270.0f)
				newRotationX = Mathf.Clamp(newRotationX, 270.0f, 360.0f);

			//transform.localRotation = Quaternion.Euler(newRotationX, newRotationY, transform.localEulerAngles.z);
			physicsBody.Look(Quaternion.Euler(newRotationX, newRotationY, transform.localEulerAngles.z));

			float moveSpeed = Time.deltaTime * m_MoveSpeed;
			if (leftShiftBoost && leftShift)
				moveSpeed *= m_Turbo;

			physicsBody.Move(transform.forward * moveSpeed * inputVertical);
			physicsBody.Move(transform.right * moveSpeed * inputHorizontal);
			// transform.position += transform.forward * moveSpeed * inputVertical;
			// transform.position += transform.right * moveSpeed * inputHorizontal;
		}
	}

	void Orbit()
	{

		if (inputRotateAxisX != 0.0f || inputRotateAxisY != 0.0f)
		{
			m_CamRotation.y += inputRotateAxisX;
			m_CamRotation.x -= inputRotateAxisY;

			if (m_CamRotation.y > 360.0f)
			{
				m_CamRotation.y -= 360.0f;
			}
			else if (m_CamRotation.y < 0.0f)
			{
				m_CamRotation.y += 360.0f;
			}

			m_CamRotation.x = Mathf.Clamp(m_CamRotation.x, -80.0f, 80.0f);
		}

		Quaternion QT = Quaternion.Euler(m_CamRotation.x, m_CamRotation.y, 0.0f);
		//transform.rotation = QT;
		physicsBody.Look(QT);

		// float tol = 0.001f;
		float tol = 10f;

		if (inputScrollDelta != 0.0f)
		{

			m_TargetOrbitRadius -= inputScrollDelta * m_ScrollSpeed * Time.deltaTime;
			m_TargetOrbitRadius = Mathf.Clamp(m_TargetOrbitRadius, m_OrbitRadiusMin, m_OrbitRadiusMax);

			m_OrbitRadius = Mathf.Lerp(m_OrbitRadius, m_TargetOrbitRadius, Time.deltaTime * m_ZoomSpeed);
			m_Camera.transform.localPosition = Vector3.back * m_OrbitRadius;

		}
		else if (m_TargetOrbitRadius - m_OrbitRadius > tol || m_OrbitRadius - m_TargetOrbitRadius > tol)
		{

			m_OrbitRadius = Mathf.Lerp(m_OrbitRadius, m_TargetOrbitRadius, Time.deltaTime * m_ZoomSpeed);
			m_Camera.transform.localPosition = Vector3.back * m_OrbitRadius;
		}

	}
}