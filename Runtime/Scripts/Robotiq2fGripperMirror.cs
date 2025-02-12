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
    public bool gripperOnly = false;
    public GameObject leftTransform;
    public GameObject rightTransform;

    private RobotManager leftRobotManager;
    private RobotManager rightRobotManager;
    private XRHandSubsystem handSubsystem;
    private float gripperLength = 0.1493f;

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

        // Instantiate the left and right hand transforms
        if (leftTransform != null)
        {
            leftTransform = Instantiate(leftTransform, Vector3.zero, Quaternion.identity);
            leftTransform.name = "Left Hand Transform";
        }

        if (rightTransform != null)
        {
            rightTransform = Instantiate(rightTransform, Vector3.zero, Quaternion.identity);
            rightTransform.name = "Right Hand Transform";
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
                    Quaternion thumbRotation;
                    Pose thumbPose;
                    if (thumbTip.TryGetPose(out thumbPose))
                    {
                      thumbPosition = thumbPose.position;
                      thumbRotation = thumbPose.rotation;
                    } else {
                      break;
                    }

                    Vector3 indexPosition;
                    Quaternion indexRotation;
                    Pose indexPose;
                    if (indexTip.TryGetPose(out indexPose))
                    {
                      indexPosition = indexPose.position;
                      indexRotation = indexPose.rotation;
                    } else {
                      break;
                    }

                    if(!gripperOnly) {
                      // Get the transformation that represents the midpoint between the thumb and the index finger
                      Vector3 midpoint = (thumbPosition + indexPosition) / 2;
                      Quaternion rotation = Quaternion.Slerp(thumbRotation, indexRotation, 0.5f) * Quaternion.Euler(90, 0, 0) * Quaternion.Euler(0, 90, 0);

                      // Move the point back by the gripper length along the forward axis of the midpoint
                      midpoint -= rotation * Vector3.up * gripperLength;

                      // Set the robot's end effector to the midpoint between the thumb and the index finger
                      leftRobotManager.SetTargetEEPose(midpoint, rotation);

                      leftTransform.transform.position = midpoint;
                      leftTransform.transform.rotation = rotation;
                    }

                    // Set the gripper's joint angle based on the distance between the thumb and the index finger
                    float distance = Vector3.Distance(thumbPosition, indexPosition);
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
                    Quaternion thumbRotation;
                    Pose thumbPose;
                    if (thumbTip.TryGetPose(out thumbPose))
                    {
                      thumbPosition = thumbPose.position;
                      thumbRotation = thumbPose.rotation;
                    } else {
                      break;
                    }

                    Vector3 indexPosition;
                    Quaternion indexRotation;
                    Pose indexPose;
                    if (indexTip.TryGetPose(out indexPose))
                    {
                      indexPosition = indexPose.position;
                      indexRotation = indexPose.rotation;
                    } else {
                      break;
                    }

                    if(!gripperOnly) {
                      // Get the transformation that represents the midpoint between the thumb and the index finger
                      Vector3 midpoint = (thumbPosition + indexPosition) / 2;
                      Quaternion rotation = Quaternion.Slerp(thumbRotation, indexRotation, 0.5f) * Quaternion.Euler(90, 0, 0) * Quaternion.Euler(0, 90, 0);

                      // Move the point back by the gripper length along the forward axis of the midpoint
                      midpoint -= rotation * Vector3.up * gripperLength;

                      // Set the robot's end effector to the midpoint between the thumb and the index finger
                      rightRobotManager.SetTargetEEPose(midpoint, rotation);

                      rightTransform.transform.position = midpoint;
                      rightTransform.transform.rotation = rotation;
                    }

                    // Set the gripper's joint angle based on the distance between the thumb and the index finger
                    float distance = Vector3.Distance(thumbPosition, indexPosition);
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
