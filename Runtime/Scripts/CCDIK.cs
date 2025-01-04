using Unity.VRTemplate;
using UnityEngine;

public class CCDIK : MonoBehaviour {
  public Transform Tooltip;
  public CCDIKJoint[] joints;
  public XRKnobAlt[] knobs;

  public void InverseKinematics(Transform target) {
    for (int j = 0; j < joints.Length; j++) {
      joints[j].Evaluate(Tooltip, target, false);
      float diff = angleDifference(joints[j].prevAngle, joints[j].transform.localEulerAngles.y);
      knobs[j].jointAngle = knobs[j].jointAngle - diff;
    }
  }

  protected float angleDifference(float a, float b) {
    float diff = a - b;
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;
    return diff;
  }
}