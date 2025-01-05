using Unity.VRTemplate;
using UnityEngine;

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

    /// <summary>
    /// Call this to run CCD on the chain, trying to match the target.
    /// </summary>
    public void InverseKinematics(Transform target)
    {
        // We do multiple passes over the entire chain
        for (int i = 0; i < iterations; i++)
        {
            // Check if we're close enough to the target
            if (Vector3.Distance(Tooltip.position, target.position) < positionThreshold &&
                Quaternion.Angle(Tooltip.rotation, target.rotation) < rotationThreshold)
            {
                Debug.Log("Target reached after " + i + " iterations");
                break;
            }

            for (int j = 0; j < joints.Length; j++)
            {
                // Make the joint try to align the Tooltip with the target
                joints[j].Evaluate(Tooltip, target, false);

                // Calculate how much that joint's local Y angle changed
                float diff = angleDifference(joints[j].prevAngle, joints[j].transform.localEulerAngles.y);

                // Adjust any external "knob" or angle readout accordingly
                if (knobs != null && j < knobs.Length && knobs[j] != null)
                {
                    knobs[j].jointAngle -= diff;
                }
            }
        }
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
}
