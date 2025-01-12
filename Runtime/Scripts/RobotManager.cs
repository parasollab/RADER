using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit.AffordanceSystem.Theme.Primitives;

public class RobotManager : MonoBehaviour
{
    public GameObject urdfModel;  // Reference to the base of the robot's URDF model
    public string robotNamespace; // The namespace of the robot
    public IKSolver ikSolver;  // Reference to the IK solver component
    public ColorAffordanceThemeDatumProperty affordanceThemeDatum;

    public GameObject gripper;  // Reference to the gripper object if any
    public GameObject graspedObject;  // Reference to the object being grasped if any
    public Vector3 graspedObjectPosition = Vector3.zero;  // Local position of the grasped object
    public Vector3 graspedObjectRotation = Vector3.zero;  // Local rotation of the grasped object in Euler angles

    private ProcessUrdf processUrdf = new ProcessUrdf();

    // Start is called before the first frame update
    void Awake()
    {
        processUrdf.ProcessModel(urdfModel, gripper, graspedObject, 
                                graspedObjectPosition, graspedObjectRotation, 
                                affordanceThemeDatum, ikSolver);
    }

    public void SetTargetEEPose(Transform target)
    {
        float[] currentAngles = GetJointAngles();
        float[] jointAngles = ikSolver.InverseKinematics(target, currentAngles);
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

    public void SetJointAngle(int jointIndex, float jointAngle)
    {
        throw new System.NotImplementedException();
    }

    public void SetGripperConfiguration()
    {
       // TODO: Implement gripper configuration
    }
}
