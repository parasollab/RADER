using System;
using System.Collections;
using System.Collections.Generic;
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
    public bool grabBase = false;

    private ProcessUrdf processUrdf;
    private ProcessUrdf gripperProcessUrdf;

    [Obsolete]
    void Awake()
    {
        // Correctly instantiate the ProcessUrdf components
        processUrdf = gameObject.AddComponent<ProcessUrdf>();
        gripperProcessUrdf = gameObject.AddComponent<ProcessUrdf>();

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

        // Calculate the joint angles to reach the target
        float[] jointAngles = ikSolver.InverseKinematics(target.position, target.rotation, currentAngles, urdfModel.transform);
        if (jointAngles == null)
        {
            return;
        }
        
        SetJointAngles(jointAngles);
    }

    public void SetTargetEEPose(Vector3 position, Quaternion rotation)
    {
        float[] currentAngles = GetJointAngles();

        float[] jointAngles = ikSolver.InverseKinematics(position, rotation, currentAngles, urdfModel.transform);
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

    public float[] GetJointAngles(bool includeGripper=false)
    {
        if (includeGripper)
        {
            float[] armJointAngles = processUrdf.GetComponent<SetupIK>().GetJointAngles();
            float[] gripperJointAngles = gripperProcessUrdf.GetComponent<SetupIK>().GetJointAngles();
            float[] jointAngles = new float[armJointAngles.Length + gripperJointAngles.Length];
            armJointAngles.CopyTo(jointAngles, 0);
            gripperJointAngles.CopyTo(jointAngles, armJointAngles.Length);
            return jointAngles;
        }

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

    public List<string> GetJointNames(bool includeGripper=false)
    {
        if (includeGripper)
        {
            List<string> jointNames = new List<string>();
            jointNames.AddRange(processUrdf.GetJointNames());
            jointNames.AddRange(gripperProcessUrdf.GetJointNames());
            return jointNames;
        }

        return processUrdf.GetJointNames();
    }

    public List<Tuple<float, float>> GetJointLimits(bool includeGripper=false)
    {
        if (includeGripper)
        {
            List<Tuple<float, float>> jointLimits = new List<Tuple<float, float>>();
            jointLimits.AddRange(processUrdf.GetJointLimits());
            jointLimits.AddRange(gripperProcessUrdf.GetJointLimits());
            return jointLimits;
        }

        return processUrdf.GetJointLimits();
    }

    // TODO: include gripper in these?
    public void SetHomePosition()
    {
        processUrdf.SetHomePosition();
    }

    public void ResetHomePosition()
    {
        processUrdf.ResetHomePosition();
    }

    public List<string> GetGripperJointNames()
    {
        return gripperProcessUrdf.GetJointNames();
    }

    public void SetGripperByJointName(string jointName, float jointAngle, bool ignoreNotFound=true)
    {
        gripper.GetComponent<SetupIK>().SetJointAngle(jointName, jointAngle, ignoreNotFound);
    }

    public void GetzGripperJointAngles()
    {
        gripper.GetComponent<SetupIK>().GetJointAngles();
    }

    public void GetGripperJointAngle(string jointName)
    {
        gripper.GetComponent<SetupIK>().GetJointAngle(jointName);
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
