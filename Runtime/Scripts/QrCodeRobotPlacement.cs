using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.XR.ARFoundation;

public class QrCodeRobotPlacement : MonoBehaviour
{
    [SerializeField] private QrCodeDisplayManager _qrCodeDisplayManager;
    [SerializeField] private ARPlaneManager _arPlaneManager; // Is this needed?
    [SerializeField] private InputActionReference _setRobotPositionAction;
    [SerializeField] private GameObject _leftRobot;
    [SerializeField] private GameObject _rightRobot;
    [SerializeField] private List<GameObject> _additionalGameObjects;

    private bool _isRobotPlaced = false;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        if (_qrCodeDisplayManager == null)
        {
            Debug.LogError("QrCodeDisplayManager not set");
        }

        if (_arPlaneManager == null)
        {
            Debug.LogError("ARPlaneManager not set");
        }

        if (_setRobotPositionAction == null)
        {
            Debug.LogError("SetRobotPositionAction not set");
        }

        if (_leftRobot == null)
        {
            Debug.LogError("LeftRobot not set");
        }

        if (_rightRobot == null)
        {
            Debug.LogError("RightRobot not set");
        }

        _setRobotPositionAction.action.performed += OnSetRobotPositionAction;
    }

    // Update is called once per frame
    void Update()
    {
        if (_isRobotPlaced)
        {
            return;
        }

        if (_qrCodeDisplayManager == null)
        {
            return;
        }

        if (_arPlaneManager == null)
        {
            return;
        }

        // Get the first marker found by the QR code scanner
        var marker = _qrCodeDisplayManager.GetFirstMarker();
        if (marker == null)
        {
            return;
        }

        // Get the closest plane to the marker
        var closestPlane = GetClosestPlane(marker.transform.position);
        if (closestPlane == null)
        {
            return;
        }

        // Get the plane's normal and a reference point on the plane
        Vector3 planeNormal = closestPlane.transform.up;
        Vector3 planePoint = closestPlane.transform.position;

        // Project the marker's position onto the plane
        Vector3 markerPosition = marker.transform.position;
        Vector3 projectedMarkerPosition = markerPosition - Vector3.Dot(markerPosition - planePoint, planeNormal) * planeNormal;

        // Project the marker's right vector onto the plane to ensure it lies in the plane
        Vector3 markerRight = marker.transform.right;
        Vector3 projectedMarkerRight = Vector3.ProjectOnPlane(markerRight, planeNormal).normalized;

        // If there are additional game objects, set their x and z to the projected marker position
        foreach (var obj in _additionalGameObjects)
        {
            if (obj != null)
            {
                var objPosition = obj.transform.position;
                obj.transform.position = new Vector3(projectedMarkerPosition.x, objPosition.y, projectedMarkerPosition.z);
            }
        }

        // If only one robot is assigned, place it directly on the QR code (projected position)
        bool hasLeft = _leftRobot != null;
        bool hasRight = _rightRobot != null;
        int robotCount = (hasLeft ? 1 : 0) + (hasRight ? 1 : 0);

        Quaternion newRotation = Quaternion.LookRotation(projectedMarkerRight, planeNormal);

        if (robotCount == 1)
        {
            if (hasLeft)
            {
                _leftRobot.transform.position = projectedMarkerPosition;
                // _leftRobot.transform.rotation = newRotation;
            }
            else if (hasRight)
            {
                _rightRobot.transform.position = projectedMarkerPosition;
                // _rightRobot.transform.rotation = newRotation;
            }
        }
        else if (robotCount == 2)
        {
            // Get the current distance between the robots
            var currentLeftRobotPosition = _leftRobot.transform.position;
            var currentRightRobotPosition = _rightRobot.transform.position;
            var distanceBetweenRobots = Vector3.Distance(currentLeftRobotPosition, currentRightRobotPosition);
            var halfDistanceBetweenRobots = distanceBetweenRobots / 2;

            // Set the new positions so that the projected marker position is centered between them
            var newLeftRobotPosition = projectedMarkerPosition - projectedMarkerRight * halfDistanceBetweenRobots;
            var newRightRobotPosition = projectedMarkerPosition + projectedMarkerRight * halfDistanceBetweenRobots;

            _leftRobot.transform.position = newLeftRobotPosition;
            _rightRobot.transform.position = newRightRobotPosition;

            _leftRobot.transform.rotation = newRotation;
            _rightRobot.transform.rotation = newRotation;
        }
    }

    private ARPlane GetClosestPlane(Vector3 position)
    {
        ARPlane closestPlane = null;
        float closestDistance = float.MaxValue;

        foreach (var plane in _arPlaneManager.trackables)
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

    private void OnSetRobotPositionAction(InputAction.CallbackContext context)
    {
        _isRobotPlaced = true;
    }
}
