using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.XR.Hands;
using UnityEngine.XR.Management;

public class HandMirror : MonoBehaviour
{
    public GameObject humanReferenceObject;
    public GameObject robotReferenceObject;
    public GameObject leftEEIndicator;
    public GameObject rightEEIndicator;
    public GameObject leftRobot;
    public GameObject rightRobot;
    private XRHandSubsystem handSubsystem;
    private RobotManager leftRobotManager;
    private RobotManager rightRobotManager;

    // Start is called before the first frame update
    void Start()
    {
        handSubsystem = XRGeneralSettings.Instance?.Manager?.activeLoader?.GetLoadedSubsystem<XRHandSubsystem>();
        if (handSubsystem != null)
        {
            handSubsystem.updatedHands += OnHandUpdate;
        }
        else
        {
            Debug.LogError("No hand subsystem found");
        }

        if (leftEEIndicator == null)
        {
            Debug.LogError("No left end effector indicator found. This is required.");
        }
        if (rightEEIndicator == null)
        {
            Debug.LogError("No right end effector indicator found. This is required.");
        }

        if (leftRobot != null)
        {
            leftRobotManager = leftRobot.GetComponent<RobotManager>();
        }

        if (rightRobot != null)
        {
            rightRobotManager = rightRobot.GetComponent<RobotManager>();
        }


        if (leftRobotManager == null)
        {
            Debug.LogError("No RobotManager found on left robot");
        }
        if (rightRobotManager == null)
        {
            Debug.LogError("No RobotManager found on right robot");
        }
    }

    void OnHandUpdate(XRHandSubsystem subsystem,
                      XRHandSubsystem.UpdateSuccessFlags updateSuccessFlags,
                      XRHandSubsystem.UpdateType updateType)
    {
        XRHand leftHand = subsystem.leftHand;
        XRHand rightHand = subsystem.rightHand;

        switch (updateType)
        {
            case XRHandSubsystem.UpdateType.Dynamic:
                // Update game logic that uses hand data
                break;
            case XRHandSubsystem.UpdateType.BeforeRender: 
                // Update visual objects that use hand data
                if (leftHand.isTracked)
                {
                    // Get the transform from the human reference object to the left hand
                    Pose leftHandPose = leftHand.rootPose;
                    Vector3 humanToHandPosition = humanReferenceObject.transform.InverseTransformPoint(leftHandPose.position);
                    Quaternion humanToHandRotation = Quaternion.Inverse(humanReferenceObject.transform.rotation) * leftHandPose.rotation;

                    // Apply the transform to the left end effector indicator relative to the robot reference object
                    leftEEIndicator.transform.position = robotReferenceObject.transform.TransformPoint(humanToHandPosition);
                    leftEEIndicator.transform.rotation = robotReferenceObject.transform.rotation * humanToHandRotation;

                    if (leftRobotManager != null)
                    {
                        // Set the target end effector pose for the left robot
                        leftRobotManager.SetTargetEEPose(leftEEIndicator.transform);
                    }
                }
                else
                {
                    // Hide the left end effector indicator
                    leftEEIndicator.transform.position = new Vector3(0, -1000, 0);
                }

                // Repeat the same process for the right hand
                if (rightHand.isTracked)
                {
                    Pose rightHandPose = rightHand.rootPose;
                    Vector3 humanToHandPosition = humanReferenceObject.transform.InverseTransformPoint(rightHandPose.position);
                    Quaternion humanToHandRotation = Quaternion.Inverse(humanReferenceObject.transform.rotation) * rightHandPose.rotation;

                    rightEEIndicator.transform.position = robotReferenceObject.transform.TransformPoint(humanToHandPosition);
                    rightEEIndicator.transform.rotation = robotReferenceObject.transform.rotation * humanToHandRotation;

                    if (rightRobotManager != null)
                    {
                        rightRobotManager.SetTargetEEPose(rightEEIndicator.transform);
                    }
                }
                else
                {
                    rightEEIndicator.transform.position = new Vector3(0, -1000, 0);
                }
                break;
        }
    }
}
