using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

// READ BEFORE WORKING ON THIS FILE:
// This file contains the analytical inverse kinematics solver for the UR5e robot.
// The solver is adapted from the ur_kinematics package in the ROS Industrial repository.
// The original code is written in C++ and can be found at:
// https://github.com/ros-industrial/universal_robot/tree/noetic-devel/ur_kinematics
// (we should include the license info from the original code when we finalize the implementation)

// The current version does not work as expected and needs to be fixed.
// Here are the issues:
// 1) Unity uses left-handed coordinate system, while the UR5e uses a right-handed coordinate system.
//   This means that the axes of the robot are different from the axes of Unity.
//   The solver needs to take this into account and convert the target position and orientation from Unity to the robot's coordinate system.
//   This file includes a forward kinematics function that also suffers from this issue (for use in testing the solver).
// 2) The solver does not handle the case when there is no solution for the given target pose.
//   In this case, the solver should find the closest reachable point to the target pose.
// 3) The base of the robot could be in a different position from the origin of the world in Unity.
//   The solver should take this into account and make the target pose relative to the robot's base.
// 4) The solver should return the joint angles in degrees, as Unity uses degrees for angles.
//   This might be already implemented, but it needs to be verified that eveything is in the units expected by respective components.
// 5) It is unclear whether this matches the desired end effector orientation.
//   If it doesn't, we should choose the solution that gets closest to the desired orientation.
//   Additionally, we might want to look into using IKFast to generate a more accurate solver.

public class UR5eAnalyticalIK : IKSolver
{

    // 1) We define an opaque pointer type.
    //    We'll store it in an IntPtr in C#.
    private struct KinematicsWrapperPtr { }

    // 2) DllImport the creation/destruction
    [DllImport("ur5e_ikfast")]
    private static extern IntPtr create_kinematics();

    [DllImport("ur5e_ikfast")]
    private static extern void destroy_kinematics(IntPtr kin);

    // 3) forward_kinematics
    [DllImport("ur5e_ikfast")]
    private static extern void forward_kinematics(
        IntPtr kin,
        float[] joint_config, 
        int joint_config_length,
        float[] out_array, 
        int out_array_length
    );

    // 4) inverse_kinematics
    [DllImport("ur5e_ikfast")]
    private static extern void inverse_kinematics(
        IntPtr kin,
        float[] ee_pose,
        int ee_pose_length,
        float[] out_array,
        int out_array_length
    );

    private static IntPtr _instance = IntPtr.Zero;

    public static void Init()
    {
        if (_instance == IntPtr.Zero)
        {
            _instance = create_kinematics();
        }
    }

    public static void Cleanup()
    {
        if (_instance != IntPtr.Zero)
        {
            destroy_kinematics(_instance);
            _instance = IntPtr.Zero;
        }
    }

    // forward 
    public static float[] Forward(float[] jointConfig)
    {
        if (_instance == IntPtr.Zero)
            Init();

        float[] result = new float[16]; // 4x4 matrix
        forward_kinematics(_instance, jointConfig, jointConfig.Length, result, result.Length);
        return result;
    }

    // inverse 
    public static float[] Inverse(float[] eePose)
    {
        if (_instance == IntPtr.Zero)
            Init();

        float[] result = new float[8*6]; // max 8 solutions, 6 joints
        inverse_kinematics(_instance, eePose, eePose.Length, result, result.Length);
        return result;
    }

    private Matrix4x4 Unity2RosHT, Ros2UnityHT;

    void Start() {
        Unity2RosHT = Matrix4x4.zero;
        Unity2RosHT[0, 0] = -1.0f;
        Unity2RosHT[1, 2] = -1.0f;
        Unity2RosHT[2, 1] =  1.0f;        

        Ros2UnityHT = Matrix4x4.zero;
        Ros2UnityHT[0, 0] = -1.0f;
        Ros2UnityHT[1, 2] =  1.0f;
        Ros2UnityHT[2, 1] = -1.0f;  
    }

    public override float[] InverseKinematics(Transform target, float[] initialAngles)
    {
        Matrix4x4 unityHT, rosHT;
        Quaternion corrected_ori;

        // Performs a 90° rotation around X so that the EE faces the forward axis
        Vector3 pos = target.position;
        Quaternion ori = target.rotation;
        corrected_ori = new Quaternion(ori.x, ori.z, -ori.y, ori.w) * Quaternion.Euler(90.0f, 0, 0) ;
        unityHT = Matrix4x4.TRS(pos, corrected_ori, Vector3.one);
        rosHT = Unity2RosHT * unityHT;

        // float[] eePose = new float[7]; // 3 pos + 4 quat
        // eePose[0] = rosHT[0, 3];
        // eePose[1] = rosHT[1, 3];
        // eePose[2] = rosHT[2, 3];

        // // Get the quaternion from the matrix
        // double qw = Math.Sqrt(1.0 + rosHT[0, 0] + rosHT[1, 1] + rosHT[2, 2]) / 2.0;
        // double qx = (rosHT[2, 1] - rosHT[1, 2]) / (4.0 * qw);
        // double qy = (rosHT[0, 2] - rosHT[2, 0]) / (4.0 * qw);
        // double qz = (rosHT[1, 0] - rosHT[0, 1]) / (4.0 * qw);

        // eePose[3] = (float)qw;
        // eePose[4] = (float)qx;
        // eePose[5] = (float)qy;
        // eePose[6] = (float)qz;

        float[] eePose = new float[] {-0.47659859f, -0.15102799f, 0.49082398f, -0.0206427f, 0.6922559f, 0.7213566f, -0.0004294f};

        // Print the target matrix for debugging
        // Debug.Log("Target position: " + pos);
        // Debug.Log("Target quaternion: " + qw + ", " + qx + ", " + qy + ", " + qz);

        // Call the inverse kinematics function
        float[] result = Inverse(eePose);

        // Print the result for debugging
        Debug.Log("Inverse kinematics result: ");
        for (int i = 0; i < result.Length; i += 6)
        {
            string strJointAngles = "";
            for (int j = 0; j < 6; j++)
            {
                strJointAngles += result[i + j] + ", ";
            }
            Debug.Log("Solution " + i/6 + ": " + strJointAngles);
        }

        // Return the first result for now
        float[] jointAngles = new float[6];
        for (int i = 0; i < 6; i++)
        {
            jointAngles[i] = result[i];
        }

        return jointAngles;
    }

    void OnDestroy()
    {
        Cleanup();
    }
}
