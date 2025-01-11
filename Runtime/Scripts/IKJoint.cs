using UnityEngine;

public class CCDIKJoint : MonoBehaviour
{
    [Header("Axis about which this joint rotates (in local space)")]
    public Vector3 axis = Vector3.right;

    [Header("Local Euler offset to re-apply after the hinge/limit constraints")]
    public Vector3 originalRotation = Vector3.zero;

    [Header("Maximum angular deviation from parent's orientation")]
    public float maxAngle = 360f;

    [HideInInspector] public float prevAngle = 0f;

    private Vector3 perpendicular; 

    void Start()
    {
        // Precompute a 'perpendicular' vector to our hinge axis for use in constraints
        perpendicular = Perpendicular(axis);
    }

    /// <summary>
    /// Returns some vector perpendicular to 'vec' (used for hinge-limit constraints).
    /// </summary>
    public static Vector3 Perpendicular(Vector3 vec)
    {
        // Just a quick utility to pick a safe perpendicular
        return Mathf.Abs(vec.x) > Mathf.Abs(vec.z)
            ? new Vector3(-vec.y, vec.x, 0f)
            : new Vector3(0f, -vec.z, vec.y);
    }

    /// <summary>
    /// Constrains 'direction' so that the angle between it and 'normalDirection' doesn't exceed maxAngle.
    /// </summary>
    public static Vector3 ConstrainToNormal(Vector3 direction, Vector3 normalDirection, float maxAngle)
    {
        if (maxAngle <= 0f)    // No movement allowed
            return normalDirection.normalized * direction.magnitude;

        if (maxAngle >= 180f) // Full range allowed
            return direction;

        // Compute angle between direction and normal
        float angle = Mathf.Acos(Mathf.Clamp(Vector3.Dot(direction.normalized, normalDirection.normalized), -1f, 1f)) * Mathf.Rad2Deg;

        // If beyond maxAngle, we slerp it closer to normalDirection
        if (angle > maxAngle)
        {
            float t = (angle - maxAngle) / angle;
            return Vector3.Slerp(direction.normalized, normalDirection.normalized, t) * direction.magnitude;
        }

        // Otherwise no constraint needed
        return direction;
    }

    public void Evaluate(Transform ToolTip, Transform Target, bool rotateToDirection = false) {
        prevAngle = transform.localEulerAngles.y;

        //Rotate the assembly so the tooltip better matches the target position/direction
        transform.rotation = (rotateToDirection ? Quaternion.FromToRotation(ToolTip.up, Target.forward) : Quaternion.FromToRotation(ToolTip.position - transform.position, Target.position - transform.position)) * transform.rotation;

        //Enforce only rotating with the hinge
        transform.rotation = Quaternion.FromToRotation(transform.rotation * axis, transform.parent.rotation * axis) * transform.rotation;
        
        //Enforce Joint Limits
        transform.rotation = Quaternion.FromToRotation(transform.rotation * perpendicular, ConstrainToNormal(transform.rotation * perpendicular, transform.parent.rotation * perpendicular, maxAngle)) * transform.rotation;

        // Align the rotation with the original orientation of the joint
        transform.rotation = transform.rotation * Quaternion.Euler(originalRotation);
    }
}