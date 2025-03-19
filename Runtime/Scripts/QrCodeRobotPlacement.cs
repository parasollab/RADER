using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.XR.ARFoundation;

public class QrCodeRobotPlacement : MonoBehaviour
{
    [SerializeField] private QrCodeDisplayManager _qrCodeDisplayManager;
    [SerializeField] private ARPlaneManager _arPlaneManager;
    [SerializeField] private InputActionReference _setRobotPositionAction;
    [SerializeField] private GameObject _leftRobot;
    [SerializeField] private GameObject _rightRobot;

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

        // Find the closest plane to the marker
        // ARPlane closestPlane = null;
        // float closestDistance = float.MaxValue;
        // foreach (var plane in _arPlaneManager.trackables)
        // {
        //     var distance = Vector3.Distance(marker.transform.position, plane.transform.position);
        //     if (distance < closestDistance)
        //     {
        //         closestPlane = plane;
        //         closestDistance = distance;
        //     }
        // }

        // Get the displacement between the robots
        var displacement = _rightRobot.transform.position - _leftRobot.transform.position;

        // Place the robots on the plane such that the marker is between them and the displacement is maintained
        // Also rotate the robots so that their forward direction is the same as the marker's forward direction
        // var markerForward = marker.transform.forward;
        // var robotForward = _leftRobot.transform.forward;
        // var angle = Vector3.SignedAngle(robotForward, markerForward, Vector3.up);
        // _leftRobot.transform.Rotate(Vector3.up, angle);
        // _rightRobot.transform.Rotate(Vector3.up, angle);

        // Set the position of the robots such that the marker is between them and the displacement is maintained along the forward direction
        var markerPosition = marker.transform.position;
        var robotPosition = markerPosition - displacement / 2;
        _leftRobot.transform.position = robotPosition;
        _rightRobot.transform.position = robotPosition + displacement;

        // _isRobotPlaced = true;
    }

    private void OnSetRobotPositionAction(InputAction.CallbackContext context)
    {
        _isRobotPlaced = true;
    }
}
