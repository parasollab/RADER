using System;
using UnityEngine;
using UnityEngine.XR.Hands;
using UnityEngine.XR.Management;

public class Robotiq2fGripperMirror : MonoBehaviour
{
    public GameObject leftRobot;
    public GameObject rightRobot;
    public string leftMasterJoint = "finger_joint";
    public string rightMasterJoint = "finger_joint";

    private RobotManager leftRobotManager;
    private RobotManager rightRobotManager;
    private XRHandSubsystem handSubsystem;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        leftRobotManager = leftRobot.GetComponent<RobotManager>();
        if (leftRobotManager == null)
        {
          Debug.Log("No RobotManager found on left robot");
        }

        rightRobotManager = rightRobot.GetComponent<RobotManager>();
        if (rightRobotManager == null)
        {
          Debug.Log("No RobotManager found on right robot");
        }

        handSubsystem = XRGeneralSettings.Instance?.Manager?.activeLoader?.GetLoadedSubsystem<XRHandSubsystem>();
        if (handSubsystem != null)
        {
            handSubsystem.updatedHands += OnHandUpdate;
        }
        else
        {
            Debug.LogError("No hand subsystem found");
        }
    }

    private void OnHandUpdate(XRHandSubsystem subsystem, 
                              XRHandSubsystem.UpdateSuccessFlags flags, 
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
                if (leftHand.isTracked && leftRobotManager != null)
                {
                    // Get the distance between the thumb and the index finger
                    XRHandJoint thumbTip = leftHand.GetJoint(XRHandJointID.ThumbTip);
                    XRHandJoint indexTip = leftHand.GetJoint(XRHandJointID.IndexTip);

                    Vector3 thumbPosition;
                    Pose thumbPose;
                    if (thumbTip.TryGetPose(out thumbPose))
                    {
                      thumbPosition = thumbPose.position;
                    } else {
                      break;
                    }

                    Vector3 indexPosition;
                    Pose indexPose;
                    if (indexTip.TryGetPose(out indexPose))
                    {
                      indexPosition = indexPose.position;
                    } else {
                      break;
                    }

                    float distance = Vector3.Distance(thumbPosition, indexPosition);

                    // Print the positions and the distance between them
                    Debug.Log("Thumb position: " + thumbPosition);
                    Debug.Log("Index position: " + indexPosition);
                    Debug.Log("Distance: " + distance);

                    // Set the gripper's joint angle based on the distance between the thumb and the index finger
                    float jointAngle = distance * 100;
                    leftRobotManager.SetGripperByJointName(leftMasterJoint, jointAngle);

                    // Print the joint angle
                    Debug.Log("Joint angle: " + jointAngle);
                }

                // Repeat the same process for the right hand
                if (rightHand.isTracked)
                {
                    // TODO
                }
                break;
        }
    }
}
