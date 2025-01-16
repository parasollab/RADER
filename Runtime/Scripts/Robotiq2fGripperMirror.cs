using System;
using UnityEngine;
using UnityEngine.XR.Hands;
using UnityEngine.XR.Management;

public class Robotiq2fGripperMirror : MonoBehaviour
{
    public GameObject leftRobot;
    public GameObject rightRobot;
    public string leftMasterJoint = "finger_joint";
    public string leftMasterJointUnityName = "left_outer_knuckle";
    public string rightMasterJoint = "finger_joint";
    public string rightMasterJointUnityName = "left_outer_knuckle";
    public float maxFingerDist = 0.11f;
    public float maxJointAngle = 45.0f;

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

                    // Set the gripper's joint angle based on the distance between the thumb and the index finger
                    float jointAngle = Map(distance, 0.0f, maxFingerDist, 0.0f, maxJointAngle, true);
                    leftRobotManager.SetGripperByJointName(leftMasterJoint, jointAngle, true);
                    leftRobotManager.SetGripperByJointName(leftMasterJointUnityName, jointAngle, false);
                }

                // Repeat the same process for the right hand
                if (rightHand.isTracked)
                {
                    // Get the distance between the thumb and the index finger
                    XRHandJoint thumbTip = rightHand.GetJoint(XRHandJointID.ThumbTip);
                    XRHandJoint indexTip = rightHand.GetJoint(XRHandJointID.IndexTip);

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

                    // Set the gripper's joint angle based on the distance between the thumb and the index finger
                    float jointAngle = Map(distance, 0.0f, maxFingerDist, 0.0f, maxJointAngle, true);
                    rightRobotManager.SetGripperByJointName(rightMasterJoint, jointAngle, true);
                    rightRobotManager.SetGripperByJointName(rightMasterJointUnityName, jointAngle, false);
                }
                break;
        }
    }

    private float Map(float value, float fromLow, float fromHigh, float toLow, float toHigh, bool invert = false)
    {
        return invert ? toHigh - (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) : (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
    }
}
