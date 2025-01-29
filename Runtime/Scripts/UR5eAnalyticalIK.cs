using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

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

    // TODO I think this is not needed (we know the tooltip location)
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

    public override float[] InverseKinematics(Vector3 pos, Quaternion ori, float[] initialAngles)
    {
        // Print the input for debugging
        // Debug.Log("Input pose: " + pos.x + ", " + pos.y + ", " + pos.z + ", " + ori.x + ", " + ori.y + ", " + ori.z + ", " + ori.w);
        // Debug.Log("Initial angles: " + initialAngles[0] + ", " + initialAngles[1] + ", " + initialAngles[2] + ", " + initialAngles[3] + ", " + initialAngles[4] + ", " + initialAngles[5]);

        // Convert the Unity coordinates to the UR5e coordinates (left-handed to right-handed)
        Quaternion corrected_ori;
        corrected_ori = new Quaternion(ori.x, ori.z, ori.y, ori.w) * Quaternion.Euler(0, 0, 0); // TODO check if this final rotation is correct (check gripper rotation relative to EE)

        Vector3 corrected_pos = new Vector3(pos.x, pos.z, pos.y);
        Vector3 rotated_pos = new Vector3(corrected_pos.y, -corrected_pos.x, corrected_pos.z);

        float[] eePose = new float[7]; // 3 pos + 4 quat
        eePose[0] = rotated_pos.x;
        eePose[1] = rotated_pos.y;
        eePose[2] = rotated_pos.z;

        eePose[3] = corrected_ori.w;
        eePose[4] = corrected_ori.x;
        eePose[5] = corrected_ori.y;
        eePose[6] = corrected_ori.z;

        // Call the inverse kinematics function
        float[] result = Inverse(eePose);

        // Print the result for debugging
        // Debug.Log("Inverse kinematics result: ");
        // for (int i = 0; i < result.Length; i += 6)
        // {
        //     string strJointAngles = "";
        //     for (int j = 0; j < 6; j++)
        //     {
        //         strJointAngles += result[i + j] + ", ";
        //     }
        //     Debug.Log("Solution " + i/6 + ": " + strJointAngles);
        // }

        // Get the number of non-zero solutions
        int maxSolutions = 8;
        int numSolutions = 0;
        for (int i = 0; i < maxSolutions; i++)
        {
            bool valid = false;
            for (int j = 0; j < 6; j++)
            {
                valid = valid || result[i * 6 + j] != 0;
            }
            if (valid)
            {
                numSolutions++;
            }
        }

        if (numSolutions == 0)
        {
            Debug.LogError("No valid solutions found");
            return null;
        }

        // Find the non-zero solution closest to the initial angles
        float minDist = float.MaxValue;
        int minIndex = -1;
        for (int i = 0; i < numSolutions; i++)
        {
            float dist = 0;
            for (int j = 0; j < 6; j++)
            {
                // Calculate the distance between the initial angles and the solution
                // The initial angles are in degrees and negated, the solutions are in radians
                dist += Mathf.Abs(result[i * 6 + j] + initialAngles[j] * Mathf.Deg2Rad);
            }
            if (dist < minDist)
            {
                minDist = dist;
                minIndex = i;
            }
        }

        // Debug.Log("Min dist: " + minDist + ", min index: " + minIndex);

        // Return the joint angles of the closest solution
        float[] jointAngles = new float[6];
        for (int i = 0; i < 6; i++)
        {
            jointAngles[i] = -1.0f * result[minIndex * 6 + i] * Mathf.Rad2Deg;
        }

        // Debug.Log("Joint angles: " + jointAngles[0] + ", " + jointAngles[1] + ", " + jointAngles[2] + ", " + jointAngles[3] + ", " + jointAngles[4] + ", " + jointAngles[5]);

        return jointAngles;
    }

    void OnDestroy()
    {
        Cleanup();
    }
}
