using System;
using Unity.VRTemplate;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single;
using System.Collections.Generic; // For single-precision (float) matrices

[System.Serializable]
public class DHParameter
{
    public float a; // Link length
    public float alpha; // Link twist
    public float d; // Link offset
    public float theta; // Joint angle
}

public class CCDIK : MonoBehaviour
{
    [Header("Assign your end-effector (the IK tip)")]
    public Transform Tooltip;

    [Header("Your chain of joints (each with a CCDIKJoint component)")]
    public CCDIKJoint[] joints;

    [Header("If you have UI knobs or another angle readout, assign them here")]
    public XRKnobAlt[] knobs;

    [Header("Number of CCD iterations per frame/update")]
    public int iterations = 20;
    public float positionThreshold = 0.1f;
    public float rotationThreshold = 1.0f;
    public float delta = 0.01f;
    public float alpha = 0.05f;

    /// <summary>
    /// Call this to run CCD on the chain, trying to match the target.
    /// </summary>
    public void InverseKinematics(List<DHParameter> dHParameters, Vector3 baseRotation, Transform target)
    {
        // TODO get the desired transform relative to the base of the robot
        // or apply the real world to base transform in forward kinematics

        Debug.Log("-> Running CCDIK on the chain...");
        // We do multiple passes over the entire chain
        for (int i = 0; i < iterations; i++)
        {
            // Get the error between the current position and the target
            Vector3 error = target.position - Tooltip.position;
            Debug.Log("Error vector: " + error);

            // Print the current end-effector position to the console
            Debug.Log("Current end-effector position: " + Tooltip.position);

            // Print the desired end-effector position to the console
            Debug.Log("Desired end-effector position: " + target.position);

            // Check if we're close enough to the target
            if (error.magnitude < positionThreshold)
            {
                Debug.Log("Reached target position after " + i + " iterations.");
                break;
            }

            // Reorder the error vector to match the DH coordinate system
            error = new Vector3(-error.z, error.x, error.y);

            // Print the reordered error vector to the console
            Debug.Log("Reordered error vector: " + error);

            // Compute the Jacobian
            float[,] jacobian = computeJacobian(dHParameters, baseRotation);
            
            // Print the Jacobian to the console
            Debug.Log("Jacobian:");
            for (int r = 0; r < jacobian.GetLength(0); r++)
            {
                string row = "";
                for (int c = 0; c < jacobian.GetLength(1); c++)
                {
                    row += jacobian[r, c] + " ";
                }
                Debug.Log(row);
            }

            // Compute the pseudoinverse of the Jacobian
            float[,] pseudoInverse = MoorePenrosePseudoInverse(jacobian);
            
            // Print the pseudoinverse to the console
            Debug.Log("Pseudoinverse:");
            for (int r = 0; r < pseudoInverse.GetLength(0); r++)
            {
                string row = "";
                for (int c = 0; c < pseudoInverse.GetLength(1); c++)
                {
                    row += pseudoInverse[r, c] + " ";
                }
                Debug.Log(row);
            }

            // Compute the delta angles using matrix multiplication
            var errorVector = DenseVector.OfArray(new float[] { error.x, error.y, error.z });
            var pseudoInverseMatrix = DenseMatrix.OfArray(pseudoInverse);
            var deltaAnglesVector = -1 * pseudoInverseMatrix * errorVector * alpha;
            float[] deltaAngles = deltaAnglesVector.ToArray();
            
            // Print the delta angles to the console
            string deltaAnglesStr = "";
            for (int j = 0; j < deltaAngles.Length; j++)
            {
                deltaAnglesStr += deltaAngles[j] + " ";
            }
            Debug.Log("Delta angles: " + deltaAnglesStr);

            // TODO check that the new angles are in degrees (I think the deltas are in radians)

            // Apply the delta angles to the joints and update the knobs
            for (int j = 0; j < joints.Length; j++)
            {
                joints[j].transform.localEulerAngles += new Vector3(0f, deltaAngles[j], 0f);
                if (knobs != null && j < knobs.Length && knobs[j] != null)
                {
                    knobs[j].jointAngle += deltaAngles[j] * Mathf.Rad2Deg;
                }
            }
        }
    }

    protected float[,] computeJacobian(List<DHParameter> dHParameters, Vector3 baseRotation)
    {
        float[,] jacobian = new float[3, joints.Length];

        // Print the current joint angles to the console
        string currentAnglesStr = "";
        for (int j = 0; j < joints.Length; j++)
        {
            currentAnglesStr += joints[j].transform.localEulerAngles.y + " ";
        }
        Debug.Log("Current joint angles: " + currentAnglesStr);

        // Get the current end-effector position using forward kinematics
        float[] currentAngles = new float[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            currentAngles[i] = joints[i].transform.localEulerAngles.y * Mathf.Deg2Rad;
        }
        Matrix4x4 currentTransform = ForwardKinematics(dHParameters, baseRotation, currentAngles);

        // Print the current transform to the console
        Debug.Log("Current transform:");
        for (int r = 0; r < 4; r++)
        {
            string row = "";
            for (int c = 0; c < 4; c++)
            {
                row += currentTransform[r, c] + " ";
            }
            Debug.Log(row);
        }

        for (int i = 0; i < joints.Length; i++)
        {
            // Perturb the current angle
            float[] forward = (float[])currentAngles.Clone();
            float[] backward = (float[])currentAngles.Clone();
            forward[i] += delta;
            backward[i] -= delta;

            // Print the forward and backward angles to the console
            string forwardStr = "";
            for (int j = 0; j < forward.Length; j++)
            {
                forwardStr += forward[j] + " ";
            }
            Debug.Log("Forward angles: " + forwardStr);
            string backwardStr = "";
            for (int j = 0; j < backward.Length; j++)
            {
                backwardStr += backward[j] + " ";
            }
            Debug.Log("Backward angles: " + backwardStr);

            // Get the new end-effector position using forward kinematics
            Matrix4x4 forwardTransform = ForwardKinematics(dHParameters, baseRotation, forward);
            Vector3 forwardEndEffectorPosition = forwardTransform.GetColumn(3);
            Matrix4x4 backwardTransform = ForwardKinematics(dHParameters, baseRotation, backward);
            Vector3 backwardEndEffectorPosition = backwardTransform.GetColumn(3);

            // Compute the difference in position (ignore rotation for now)
            Vector3 diff = forwardEndEffectorPosition - backwardEndEffectorPosition;

            // Store the difference in the Jacobian
            jacobian[0, i] = diff.x / delta;
            jacobian[1, i] = diff.y / delta;
            jacobian[2, i] = diff.z / delta;
        }

        return jacobian;
    }

    /// <summary>
    /// Simple utility to get the difference between two angles, keeping it in [-180, 180].
    /// </summary>
    protected float angleDifference(float a, float b)
    {
        float diff = a - b;
        while (diff > 180f) diff -= 360f;
        while (diff < -180f) diff += 360f;
        return diff;
    }
    
    /// <summary>
    /// Computes the Moore-Penrose pseudoinverse of an arbitrary-sized 2D float array.
    /// Requires MathNet.Numerics to be referenced in the project.
    /// </summary>
    /// <param name="matrix">2D float array representing the matrix (rows x columns).</param>
    /// <returns>2D float array of the pseudoinverse (columns x rows).</returns>
    protected float[,] MoorePenrosePseudoInverse(float[,] matrix)
    {
        // Create a Math.NET Numerics matrix from the 2D float array
        var mat = DenseMatrix.OfArray(matrix);

        // Compute the pseudoinverse using SVD under the hood
        var pseudoInv = mat.PseudoInverse();

        // Convert result back into a 2D float array
        float[,] result = new float[pseudoInv.RowCount, pseudoInv.ColumnCount];
        for (int i = 0; i < pseudoInv.RowCount; i++)
        {
            for (int j = 0; j < pseudoInv.ColumnCount; j++)
            {
                result[i, j] = pseudoInv[i, j];
            }
        }

        return result;
    }

    /// <summary>
    /// Computes the forward kinematics for the given DH parameters and joint angles.
    /// </summary>
    /// <param name="dHParameters">List of DH parameters for each joint.</param>
    /// <param name="jointAngles">Array of current joint angles.</param>
    /// <returns>The transformation matrix of the end-effector.</returns>
    public Matrix4x4 ForwardKinematics(List<DHParameter> dHParameters, Vector3 baseRotation, float[] jointAngles)
    {
        if (dHParameters.Count != jointAngles.Length)
        {
            throw new ArgumentException("The number of DH parameters must match the number of joint angles.");
        }

        Matrix4x4 result = Matrix4x4.Rotate(Quaternion.Euler(baseRotation));

        for (int i = 0; i < dHParameters.Count; i++)
        {
            DHParameter dh = dHParameters[i];
            // float theta = jointAngles[i] + dh.theta;

            // Matrix4x4 transform = Matrix4x4.TRS(
            //     new Vector3(dh.a, 0, dh.d),
            //     Quaternion.Euler(dh.alpha * Mathf.Rad2Deg, theta * Mathf.Rad2Deg, 0),
            //     Vector3.one
            // );

            Matrix4x4 dhTransform = DHTransform(dh, jointAngles[i]);

            // Multiply into the cumulative result:
            result = result * dhTransform.transpose;
        }
        
        return result;
    }

    private Matrix4x4 DHTransform(DHParameter dh, float angle) 
    {
        float theta = angle + dh.theta;
        return new Matrix4x4(new Vector4(Mathf.Cos(theta), -Mathf.Sin(theta) * Mathf.Cos(dh.alpha), Mathf.Sin(theta) * Mathf.Sin(dh.alpha), dh.a * Mathf.Cos(theta)),
                            new Vector4(Mathf.Sin(theta) , Mathf.Cos(theta) * Mathf.Cos(dh.alpha), -Mathf.Sin(dh.alpha) * Mathf.Cos(theta), Mathf.Sin(theta) * dh.a),
                            new Vector4(0, Mathf.Sin(dh.alpha), Mathf.Cos(dh.alpha), dh.d),
                            new Vector4(0, 0, 0, 1));
    }
}


// public class CCDIK : MonoBehaviour
// {
//     [Header("End-effector (IK tip)")]
//     public Transform Tooltip;

//     [Header("Chain of joints (in order from base to end-effector)")]
//     public CCDIKJoint[] joints;

//     [Header("Knobs that track each joint's angle (same length as joints)")]
//     public XRKnobAlt[] knobs;

//     [Header("IK iterations per update")]
//     public int iterations = 1;

//     [Header("Step size (learning rate)")]
//     public float alpha = 0.1f;

//     [Header("Finite difference step for Jacobian")]
//     public float delta = 0.01f;

//     [Header("Position error stop threshold (meters)")]
//     public float positionThreshold = 0.01f;

//     [Header("Orientation error stop threshold (degrees)")]
//     public float orientationThreshold = 0.5f;

//     [Header("Scale factor for orientation error (balance pos vs. rot)")]
//     public float orientationErrorScale = 0.0f;

//     /// <summary>
//     /// Performs a Jacobian-based IK step for both position and orientation,
//     /// then updates 'knobs' to reflect the change in local Y angles for each joint.
//     /// </summary>
//     public void InverseKinematics(Transform target)
//     {
//         // 1) Collect current angles (assuming each joint rotates around local Y).
//         float[] angles = new float[joints.Length];
//         for (int i = 0; i < joints.Length; i++)
//         {
//             angles[i] = joints[i].transform.localEulerAngles.y;
//         }

//         // We'll do multiple iterations each frame
//         for (int iter = 0; iter < iterations; iter++)
//         {
//             // Store the old local angles so we can measure the delta afterward
//             // (to update knobs in the same style as your original code).
//             float[] oldLocalAngles = new float[joints.Length];
//             for (int j = 0; j < joints.Length; j++)
//             {
//                 oldLocalAngles[j] = joints[j].transform.localEulerAngles.y;
//             }

//             // 2) Compute current 6D error = [ pos.x, pos.y, pos.z, rot.x, rot.y, rot.z ]
//             float[] baselineError = ComputeCurrentError(target);

//             // Check stopping criteria
//             Vector3 posErr = new Vector3(baselineError[0], baselineError[1], baselineError[2]);
//             float posErrMag = posErr.magnitude;

//             Vector3 rotErr = new Vector3(baselineError[3], baselineError[4], baselineError[5]);
//             // this is in radians, convert magnitude to degrees for readability
//             float rotErrDeg = rotErr.magnitude * Mathf.Rad2Deg; 

//             if (posErrMag < positionThreshold && rotErrDeg < orientationThreshold)
//                 break;

//             // 3) Build the 6xN Jacobian with finite differences
//             float[,] J = new float[6, angles.Length];

//             // For each joint, perturb by +delta
//             for (int j = 0; j < angles.Length; j++)
//             {
//                 float oldAngle = angles[j];

//                 angles[j] += delta;
//                 SetAngles(angles); // temporarily apply to measure effect

//                 float[] newError = ComputeCurrentError(target);

//                 // partial derivative wrt this joint's angle
//                 for (int row = 0; row < 6; row++)
//                 {
//                     float diff = newError[row] - baselineError[row];
//                     J[row, j] = diff / delta;
//                 }

//                 // revert
//                 angles[j] = oldAngle;
//                 SetAngles(angles);
//             }

//             // 4) Compute deltaAngles = alpha * J^T * error
//             float[] deltaAngles = new float[angles.Length];
//             for (int j = 0; j < angles.Length; j++)
//             {
//                 float dot = 0f;
//                 for (int row = 0; row < 6; row++)
//                 {
//                     dot += J[row, j] * baselineError[row];
//                 }
//                 deltaAngles[j] = alpha * dot;
//             }

//             // 5) Update angles
//             for (int j = 0; j < angles.Length; j++)
//             {
//                 angles[j] += deltaAngles[j];
//             }

//             // 6) (Optional) Enforce joint limits or hinge constraints
//             // e.g.: angles[j] = Mathf.Clamp(angles[j], minAngle, maxAngle);

//             // 7) Apply updated angles
//             SetAngles(angles);

//             // 8) Now measure the difference in local Y for each joint from oldLocalAngles,
//             //    then update knobs the same way your original code did.
//             if (knobs != null && knobs.Length == joints.Length)
//             {
//                 for (int j = 0; j < joints.Length; j++)
//                 {
//                     float newAngle = joints[j].transform.localEulerAngles.y;
//                     float diff = AngleDifference(oldLocalAngles[j], newAngle);

//                     // Subtract the difference from the knob angle
//                     // (Same as knobs[j].jointAngle = knobs[j].jointAngle - diff)
//                     knobs[j].jointAngle -= diff;
//                 }
//             }
//         }
//     }

//     /// <summary>
//     /// Applies the given angles to each joint's localEulerAngles.y.
//     /// </summary>
//     void SetAngles(float[] angles)
//     {
//         for (int i = 0; i < joints.Length; i++)
//         {
//             Vector3 e = joints[i].transform.localEulerAngles;
//             e.y = angles[i];
//             joints[i].transform.localEulerAngles = e;
//         }
//     }

//     /// <summary>
//     /// Computes a 6D error vector: 
//     ///   [pos.x, pos.y, pos.z,  rot.x, rot.y, rot.z]
//     /// Position error is the difference in world space.
//     /// Orientation error is an axis-angle vector (axis * angle in radians)
//     /// of qError = Target.rotation * Inverse(Tooltip.rotation).
//     /// Multiplied by 'orientationErrorScale' to control weighting.
//     /// </summary>
//     float[] ComputeCurrentError(Transform target)
//     {
//         float[] err = new float[6];

//         // Position
//         Vector3 currentPos = Tooltip.position;
//         Vector3 targetPos = target.position;
//         Vector3 posError = targetPos - currentPos;

//         err[0] = posError.x;
//         err[1] = posError.y;
//         err[2] = posError.z;

//         // Orientation (axis-angle in radians)
//         Quaternion qError = target.rotation * Quaternion.Inverse(Tooltip.rotation);
//         Vector3 axisAngle = QuaternionToAxisAngle(qError);

//         // Weight the orientation error if desired
//         axisAngle *= orientationErrorScale;

//         err[3] = axisAngle.x;
//         err[4] = axisAngle.y;
//         err[5] = axisAngle.z;

//         return err;
//     }

//     /// <summary>
//     /// Converts a quaternion to axis-angle form (axis * angle in radians).
//     /// The angle is in [0..π].
//     /// </summary>
//     Vector3 QuaternionToAxisAngle(Quaternion q)
//     {
//         q.ToAngleAxis(out float angleDeg, out Vector3 axis);
//         if (axis == Vector3.zero)
//             return Vector3.zero;

//         float angleRad = angleDeg * Mathf.Deg2Rad;
//         axis.Normalize();
//         return axis * angleRad;
//     }

//     /// <summary>
//     /// Ensures we get a delta in [-180, 180].
//     /// </summary>
//     float AngleDifference(float a, float b)
//     {
//         float diff = a - b;
//         while (diff > 180f) diff -= 360f;
//         while (diff < -180f) diff += 360f;
//         return diff;
//     }
// }
