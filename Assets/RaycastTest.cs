using UnityEngine;

public class RaycastTest : MonoBehaviour
{
    Collider myCollider;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {

        myCollider = GetComponent<Collider>();
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            const float raycastStartOffset = 0.5f;
            const float maxPenetration = 1.0f;

            Vector3 rayStart = transform.position + transform.forward * 0.5f + -transform.forward * raycastStartOffset;

            Ray ray = new Ray(rayStart, transform.forward);
            Debug.Log("Check");

            float rayLength = (raycastStartOffset + maxPenetration) * 50;
            Debug.DrawLine(rayStart, rayStart + rayLength * transform.forward, Color.red, 10.0f);

            // if (Physics.Raycast(ray, out RaycastHit hit, rayLength))
            // {
            //     float penetrationDepth = hit.distance - raycastStartOffset;

            //     if (penetrationDepth > 0.0001f)
            //     {
            //         //physicsBody.RegisterContact(otherBody, hit.point, contact.normal, penetrationDepth);
            //     }

            //     Debug.Log($"Penetration: {penetrationDepth}");
            // }

            if (myCollider.Raycast(ray, out RaycastHit hit, rayLength))
            {
                float penetrationDepth = hit.distance - raycastStartOffset;

                if (penetrationDepth > 0.0001f)
                {
                    //physicsBody.RegisterContact(otherBody, hit.point, contact.normal, penetrationDepth);
                }

                Debug.Log($"Penetration: {penetrationDepth}");
            }
        }
    }

	void OnCollisionStay(Collision collision)
	{
		// const float raycastStartOffset = 5.0f;
        // const float maxPenetration = 10.0f;

        // ContactPoint contact = collision.GetContact(0);

        // Vector3 rayStart = contact.point + contact.normal * raycastStartOffset;

        // Ray ray = new Ray(rayStart, -contact.normal);

        // float rayLength = (raycastStartOffset + maxPenetration);
        // Debug.DrawLine(rayStart, rayStart + rayLength * transform.forward, Color.red, 1.0f);

        // if (collision.collider.Raycast(ray, out RaycastHit hit, rayLength))
        // {
        //     float penetrationDepth = raycastStartOffset - hit.distance;

        //     if (penetrationDepth > 0.0001f)
        //     {
        //         //physicsBody.RegisterContact(otherBody, hit.point, contact.normal, penetrationDepth);

        //         Debug.Log($"Penetration: {penetrationDepth}");
        //     }
        // }

        for (int i = 0; i < collision.contactCount; i++)
        {
            ContactPoint contact = collision.GetContact(i);

            const float raycastStartOffset = 5.0f;
            const float maxPenetration = 10.0f;

            Vector3 rayStart = contact.point + contact.normal * raycastStartOffset;

            Ray ray = new Ray(rayStart, -contact.normal);

            if (collision.collider.Raycast(ray, out RaycastHit hit, raycastStartOffset + maxPenetration))
            {
                float penetrationDepth = raycastStartOffset - hit.distance;

                if (penetrationDepth > 0.0001f)
                {
                    //physicsBody.RegisterContact(otherBody, hit.point, contact.normal, penetrationDepth);
                    Debug.Log($"Penetration: {penetrationDepth}");
                }
            }
        }
	}
}
