using System;
using Unity.VRTemplate;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Single; // For single-precision (float) matrices

public class CCDIK : MonoBehaviour
{
    [Header("Assign your end-effector (the IK tip)")]
    public Transform Tooltip;

    [Header("Your chain of joints (each with a CCDIKJoint component)")]
    public CCDIKJoint[] joints;

    [Header("If you have UI knobs or another angle readout, assign them here")]
    public XRKnobAlt[] knobs;

    [Header("Number of CCD iterations per frame/update")]
    public int iterations = 1000;
    public float positionThreshold = 0.1f;
    public float rotationThreshold = 1.0f;
    public float delta = 0.01f;
    public float alpha = 0.1f;

    /// <summary>
    /// Call this to run CCD on the chain, trying to match the target.
    /// </summary>
    public void InverseKinematics(Transform target)
    {
        // We do multiple passes over the entire chain
        for (int i = 0; i < iterations; i++)
        {
            // Get the error between the current position and the target
            Vector3 error = target.position - Tooltip.position;

            // Check if we're close enough to the target
            if (error.magnitude < positionThreshold)
            {
                Debug.Log("Reached target position after " + i + " iterations.");
                break;
            }

            // Compute the Jacobian
            float[,] jacobian = computeJacobian();

            // Compute the pseudoinverse of the Jacobian
            float[,] pseudoInverse = MoorePenrosePseudoInverse(jacobian);

            // Compute the delta angles
            float[] deltaAngles = new float[joints.Length];
            for (int j = 0; j < joints.Length; j++)
            {
                deltaAngles[j] = 0f;
                for (int k = 0; k < 6; k++)
                {
                    deltaAngles[j] += pseudoInverse[k, j] * error[k];
                }
                deltaAngles[j] *= alpha;
            }

            // Apply the delta angles to the joints and update the knobs
            for (int j = 0; j < joints.Length; j++)
            {
                joints[j].transform.localEulerAngles += new Vector3(0f, deltaAngles[j], 0f);
                if (knobs != null && j < knobs.Length && knobs[j] != null)
                {
                    knobs[j].jointAngle += deltaAngles[j];
                }
            }

            // for (int j = 0; j < joints.Length; j++)
            // {
            //     // Make the joint try to align the Tooltip with the target
            //     joints[j].Evaluate(Tooltip, target, true);

            //     // Calculate how much that joint's local Y angle changed
            //     float diff = angleDifference(joints[j].prevAngle, joints[j].transform.localEulerAngles.y);

            //     // Adjust any external "knob" or angle readout accordingly
            //     if (knobs != null && j < knobs.Length && knobs[j] != null)
            //     {
            //         knobs[j].jointAngle -= diff;
            //     }
            // }
        }
    }

    protected float[,] computeJacobian()
    {
        float[,] jacobian = new float[3, joints.Length];

        // Get the current end-effector transform
        Transform endEffector = joints[joints.Length - 1].transform;

        for (int i = 0; i < joints.Length; i++)
        {
            joints[i].Purturb(delta);

            // Get the new end-effector transform
            Transform newEndEffector = joints[joints.Length - 1].transform;

            // Compute the difference in position (ignore rotation for now)
            Vector3 diff = newEndEffector.position - endEffector.position;

            // Store the difference in the Jacobian
            jacobian[0, i] = diff.x;
            jacobian[1, i] = diff.y;
            jacobian[2, i] = diff.z;

            // Revert the perturbation
            joints[i].Purturb(-delta);
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
