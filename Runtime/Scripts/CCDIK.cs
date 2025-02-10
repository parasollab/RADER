using Unity.VRTemplate;
using UnityEngine;

public class CCDIK : IKSolver {
    public Transform Tooltip;
    public CCDIKJoint[] joints;
    public XRKnobAlt[] knobs;
    bool m_active = false;
    public bool active {
        get => m_active;
        set => m_active = value;
    }

    // TODO this is deprecated until it can be updated - needs to be cleaned up to work with new blackbox structure
    // ex. It need to be able to work with position and rotation inputs, not the transform of the target
    // and it needs to be able to work with the input pos and ori being relative to the robot's base.

    public override float[] InverseKinematics(Vector3 pos, Quaternion ori, float[] initialAngles, Transform baseTransform)
    {
        float[] angles = new float[joints.Length];

        // for (int j = 0; j < joints.Length; j++)
        // {
        //     joints[j].Evaluate(Tooltip, target, false);
        //     float diff = AngleDifference(joints[j].prevAngle, joints[j].transform.localEulerAngles.y);
        //     knobs[j].jointAngle = knobs[j].jointAngle - diff;
        //     angles[j] = knobs[j].jointAngle;
        // }

        return angles;
    }

    /// <summary>
    /// Ensures we get a delta in [-180, 180].
    /// </summary>
    private float AngleDifference(float a, float b)
    {
        float diff = a - b;
        while (diff > 180f) diff -= 360f;
        while (diff < -180f) diff += 360f;
        return diff;
    }
}