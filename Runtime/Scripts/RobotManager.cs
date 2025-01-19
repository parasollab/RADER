using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using Unity.VRTemplate;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit.AffordanceSystem.Theme.Primitives;

public class RobotManager : MonoBehaviour
{
    public GameObject urdfModel;  // Reference to the base of the robot's URDF model
    public GameObject gripper;
    public KnobAxis gripperKnobAxis = KnobAxis.Y;
    public string robotNamespace; // The namespace of the robot
    public IKSolver ikSolver;  // Reference to the IK solver component
    [Obsolete]
    public ColorAffordanceThemeDatumProperty affordanceThemeDatum;

    private ProcessUrdf processUrdf = new ProcessUrdf();
    private ProcessUrdf gripperProcessUrdf = new ProcessUrdf();

    // Start is called before the first frame update
    [Obsolete]
    void Awake()
    {
        processUrdf.ProcessModel(urdfModel, affordanceThemeDatum, ikSolver);

        if (gripper != null)
        {
            gripperProcessUrdf.ProcessModel(gripper, affordanceThemeDatum, knobAxis: gripperKnobAxis);
            AddGripper(gripper, processUrdf.LastLink);
        }
    }

    public void SetTargetEEPose(Transform target)
    {
        float[] currentAngles = GetJointAngles();
        float[] jointAngles = ikSolver.InverseKinematics(target, currentAngles);
        if (jointAngles == null)
        {
            return;
        }
        
        SetJointAngles(jointAngles);
    }

    public Transform GetEEPose()
    {
        return processUrdf.LastLink.transform;
    }

    public float[] GetJointAngles()
    {
        return urdfModel.GetComponent<SetupIK>().GetJointAngles();
    }

    public void SetJointAngles(float[] jointAngles)
    {
        urdfModel.GetComponent<SetupIK>().SetJointAngles(jointAngles);
    }

    public float GetJointAngle(int jointIndex)
    {
        return urdfModel.GetComponent<SetupIK>().GetJointAngle(jointIndex);
    }

    public void SetJointAngle(int jointIndex, float jointAngle)
    {
        urdfModel.GetComponent<SetupIK>().SetJointAngle(jointIndex, jointAngle);
    }

    public List<string> GetJointNames()
    {
        return processUrdf.GetJointNames();
    }

    public List<Tuple<float, float>> GetJointLimits()
    {
        return processUrdf.GetJointLimits();
    }

    public void SetHomePosition()
    {
        processUrdf.SetHomePosition();
    }

    public void ResetHomePosition()
    {
        processUrdf.ResetHomePosition();
    }

    public void SetGripperByJointName(string jointName, float jointAngle, bool ignoreNotFound=true)
    {
        gripper.GetComponent<SetupIK>().SetJointAngle(jointName, jointAngle, ignoreNotFound);
    }

    private void AddGripper(GameObject gripper, GameObject lastLink)
    {
        // Set the gripper's transform to the last link's position and rotation
        gripper.transform.position = lastLink.transform.position;
        gripper.transform.rotation = lastLink.transform.rotation;
        gripper.transform.SetParent(lastLink.transform);
        gripper.transform.localPosition = Vector3.zero;

        // Rotate the gripper by 90 degrees around the y-axis (TODO is this necessary?)
        gripper.transform.localRotation = Quaternion.Euler(0, 90, 0);
    }
}
