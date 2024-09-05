using Unity.VRTemplate;
using UnityEngine;
public class CCDIK : MonoBehaviour {
  public Transform Tooltip;
  public Transform Target;
  public CCDIKJoint[] joints;
  public XRKnobAlt[] knobs;
  bool m_active = false;
  public bool active {
      get => m_active;
      set => m_active = value;
  }
  void Update() {
    if (m_active) {
      for (int j = 0; j < joints.Length; j++) {
        joints[j].Evaluate(Tooltip, Target, false);
        float diff = angleDifference(joints[j].prevAngle, joints[j].transform.localEulerAngles.y);
        knobs[j].jointAngle = knobs[j].jointAngle - diff;
      }
    }
  }

  protected float angleDifference(float a, float b) {
    float diff = a - b;
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;
    return diff;
  }
}