using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.XR.ARFoundation;

public class MarkerRobotPlacement : MonoBehaviour
{
    [SerializeField] private InputActionReference placeRobotAction;
    [SerializeField] private GameObject markerIndicator;
    [SerializeField] private ARPlaneManager arPlaneManager;

    [SerializeField] private GameObject leftRobot;
    [SerializeField] private GameObject rightRobot;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        placeRobotAction.action.performed += OnPlaceRobot;
    }

    void OnPlaceRobot(InputAction.CallbackContext context)
    {
        if (markerIndicator == null)
        {
            return;
        }

        if (arPlaneManager == null)
        {
            return;
        }

        // Get the closest plane to the marker
        var closestPlane = GetClosestPlane(markerIndicator.transform.position);
        if (closestPlane == null)
        {
            return;
        }

        // Get the plane's normal and a reference point on the plane
        Vector3 planeNormal = closestPlane.transform.up;
        Vector3 planePoint = closestPlane.transform.position;

        // Project the marker's position onto the plane
        Vector3 markerPosition = markerIndicator.transform.position;
        Vector3 projectedMarkerPosition = markerPosition - Vector3.Dot(markerPosition - planePoint, planeNormal) * planeNormal;

        // Project the marker's right vector onto the plane to ensure it lies in the plane
        Vector3 markerRight = markerIndicator.transform.right;
        Vector3 projectedMarkerRight = Vector3.ProjectOnPlane(markerRight, planeNormal).normalized;

        // If only one robot is assigned, place it directly on the marker (projected position)
        bool hasLeft = leftRobot != null;
        bool hasRight = rightRobot != null;
        int robotCount = (hasLeft ? 1 : 0) + (hasRight ? 1 : 0);

        Quaternion newRotation = Quaternion.LookRotation(projectedMarkerRight, planeNormal);

        if (robotCount == 1)
        {
            if (hasLeft)
            {
                leftRobot.transform.position = projectedMarkerPosition;
                leftRobot.transform.rotation = newRotation;
            }
            else if (hasRight)
            {
                rightRobot.transform.position = projectedMarkerPosition;
                rightRobot.transform.rotation = newRotation;
            }
        }
        else if (robotCount == 2)
        {
            // Get the current distance between the robots
            var currentLeftRobotPosition = leftRobot.transform.position;
            var currentRightRobotPosition = rightRobot.transform.position;
            var distanceBetweenRobots = Vector3.Distance(currentLeftRobotPosition, currentRightRobotPosition);
            var halfDistanceBetweenRobots = distanceBetweenRobots / 2;

            // Set the new positions so that the projected marker position is centered between them
            var newLeftRobotPosition = projectedMarkerPosition - projectedMarkerRight * halfDistanceBetweenRobots;
            var newRightRobotPosition = projectedMarkerPosition + projectedMarkerRight * halfDistanceBetweenRobots;

            leftRobot.transform.position = newLeftRobotPosition;
            rightRobot.transform.position = newRightRobotPosition;

            leftRobot.transform.rotation = newRotation;
            rightRobot.transform.rotation = newRotation;
        }
    }

    private ARPlane GetClosestPlane(Vector3 position)
    {
        ARPlane closestPlane = null;
        float closestDistance = float.MaxValue;

        foreach (var plane in arPlaneManager.trackables)
        {
            var planePosition = plane.transform.position;
            var distance = Vector3.Distance(position, planePosition);

            if (distance < closestDistance)
            {
                closestPlane = plane;
                closestDistance = distance;
            }
        }

        return closestPlane;
    }
}
