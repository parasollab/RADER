using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;

public class SetupGrabBase : MonoBehaviour
{
    // Start is called before the first frame update
    public GameObject Base;
    public GameObject Robot;
    void Start()
    {
        GrabBaseSetup(Base);
    }

    void GrabBaseSetup(GameObject obj)
    {
        // The Rigidbody lives on Base (the base_joint mesh), but we attach the
        // XRGrabInteractable to Robot (the prefab root) so the whole hierarchy
        // moves when grabbed — including BaseTransform and any other children.
        Rigidbody baseGrabRb = obj.GetComponent<Rigidbody>();
        baseGrabRb.isKinematic = true;
        baseGrabRb.useGravity = false;

        Rigidbody robotRb = Robot.GetComponent<Rigidbody>();
        if (robotRb == null)
        {
            robotRb = Robot.AddComponent<Rigidbody>();
        }
        robotRb.isKinematic = true;
        robotRb.useGravity = false;

        UnityEngine.XR.Interaction.Toolkit.Interactables.XRGrabInteractable grab =
            Robot.AddComponent<UnityEngine.XR.Interaction.Toolkit.Interactables.XRGrabInteractable>();

        grab.colliders.Clear();

        MeshCollider meshCollider = obj.GetComponentInChildren<MeshCollider>();
        grab.colliders.Add(meshCollider);

        grab.selectExited.AddListener((SelectExitEventArgs interactor) => {
            GroundRobot(Robot);
        });
    }

    void GroundRobot(GameObject robot)
    {
        // Raycast downwards to find the ground
        RaycastHit hit;
        Vector3 rayOrigin = robot.transform.position;
        Vector3 rayDirection = Vector3.down;
        float rayLength = 100f;

        if (Physics.Raycast(rayOrigin, rayDirection, out hit, rayLength))
        {
            Vector3 endPosition = new Vector3(robot.transform.position.x, hit.point.y, robot.transform.position.z);
            robot.transform.position = endPosition;
            robot.transform.rotation = Quaternion.identity;
        }
    }
}
