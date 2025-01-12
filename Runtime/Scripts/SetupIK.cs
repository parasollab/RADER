using System.Collections.Generic;
using Unity.VRTemplate;
using Unity.XR.CoreUtils;
using UnityEngine;

public class SetupIK : MonoBehaviour
{
    public IKSolver ikSolver;
    private List<CCDIKJoint> ikJoints = new List<CCDIKJoint>();
    private List<XRKnobAlt> xrKnobs = new List<XRKnobAlt>();
    private GameObject lastChild;

    public void Initialize()
    {
        TraverseAndAnalyze(this.gameObject);
        if(lastChild == null)
        {
            Debug.LogError("No XRKnob found in children, error in prefabSetup");
            return;
        }
        if(ikJoints.Count == 0)
        {
            Debug.LogError("No IKJoint found in children, error in prefabSetup");
            return;
        }
        setupIK(lastChild);
    }


    void TraverseAndAnalyze(GameObject obj)
    {
        if (obj == null) return;
        if(obj.GetComponent<XRKnobAlt>() != null)
        {
            lastChild = obj.gameObject;
            xrKnobs.Add(obj.GetComponent<XRKnobAlt>());
        }
        if(obj.GetComponent<CCDIKJoint>() != null)
        {
            ikJoints.Add(obj.GetComponent<CCDIKJoint>());
        }
        
        // Recursively process each child
        foreach (Transform child in obj.transform)
        {
            TraverseAndAnalyze(child.gameObject);
        }
    }

    void setupIK(GameObject lastChild)
    {
        if (ikSolver.GetType() == typeof(CCDIK))
        {
            // ccdIK
            // CCDIK ccdIK = lastChild.AddComponent<CCDIK>();

            CCDIK ccdIK = (CCDIK)ikSolver;
            
            ccdIK.joints = ikJoints.ToArray();
            ccdIK.knobs = xrKnobs.ToArray();
            ccdIK.Tooltip = findRealLastChild(lastChild.transform);
        }
    }

    public void SetJointAngles(float[] angles)
    {
        if (xrKnobs != null && xrKnobs.Count == ikJoints.Count && angles.Length == ikJoints.Count)
        {
            for (int j = 0; j < angles.Length; j++)
            {
                float oldAngle = ikJoints[j].transform.localEulerAngles.y;
                float diff = AngleDifference(oldAngle, angles[j]);

                // Subtract the difference from the knob angle
                xrKnobs[j].jointAngle -= diff;
            }
        } else {
            Debug.LogError("Invalid number of angles or knobs");
        }
    }

    public void SetJointAngle(int jointIndex, float jointAngle)
    {
        if (xrKnobs != null && xrKnobs.Count == ikJoints.Count)
        {
            float oldAngle = ikJoints[jointIndex].transform.localEulerAngles.y;
            float diff = AngleDifference(oldAngle, jointAngle);

            // Subtract the difference from the knob angle
            xrKnobs[jointIndex].jointAngle -= diff;
        } else {
            Debug.LogError("Invalid number of angles or knobs");
        }
    }

    public float[] GetJointAngles()
    {
        float[] angles = new float[xrKnobs.Count];
        for (int j = 0; j < xrKnobs.Count; j++)
        {
            angles[j] = xrKnobs[j].jointAngle;
        }
        return angles;
    }

    public float GetJointAngle(int jointIndex)
    {
        return xrKnobs[jointIndex].jointAngle;
    }

    Transform findRealLastChild(Transform lastChild) {
        foreach (Transform child in lastChild) {
            if (child.gameObject.GetNamedChild("Collisions") != null && child.gameObject.GetNamedChild("Visuals") != null) {
                return findRealLastChild(child);
            }
        }
        return lastChild;
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
