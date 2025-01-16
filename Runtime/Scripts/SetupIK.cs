using System.Collections.Generic;
using System;
using Unity.VRTemplate;
using Unity.XR.CoreUtils;
using UnityEngine;

public class SetupIK : MonoBehaviour
{
    public IKSolver ikSolver;
    public Transform Tooltip;
    private List<CCDIKJoint> ikJoints = new List<CCDIKJoint>();
    private List<XRKnobAlt> xrKnobs = new List<XRKnobAlt>();
    private GameObject lastChild;

    public Dictionary<string, List<string>> mimicJointMap = new Dictionary<string, List<string>>();
    public Dictionary<string, double> mimicJointOffsetMap = new Dictionary<string, double>();
    public Dictionary<string, double> mimicJointMultiplierMap = new Dictionary<string, double>();

    private List<Tuple<string, int>> nameToNumber = new List<Tuple<string, int>>();

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
        setupIK();
    }


    void TraverseAndAnalyze(GameObject obj)
    {
        if (obj == null) return;
        if(obj.GetComponent<XRKnobAlt>() != null)
        {
            lastChild = obj.gameObject;
            xrKnobs.Add(obj.GetComponent<XRKnobAlt>());
            nameToNumber.Add(new Tuple<string, int>(obj.name, xrKnobs.Count - 1));
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

    void setupIK()
    {
        if (ikSolver != null && ikSolver.GetType() == typeof(CCDIK))
        {
            // ccdIK
            // CCDIK ccdIK = lastChild.AddComponent<CCDIK>();

            CCDIK ccdIK = (CCDIK)ikSolver;
            
            ccdIK.joints = ikJoints.ToArray();
            ccdIK.knobs = xrKnobs.ToArray();
            ccdIK.Tooltip = Tooltip;
        }
    }

    public void SetJointAngles(float[] angles)
    {
        if (xrKnobs != null && xrKnobs.Count == ikJoints.Count && angles.Length == ikJoints.Count)
        {
            for (int j = 0; j < angles.Length; j++)
            {
                SetJointAngle(j, angles[j]);
            }
        } else {
            Debug.LogError("Invalid number of angles or knobs");
        }
    }

    public void SetJointAngle(int jointIndex, float jointAngle)
    {
        if (xrKnobs != null && xrKnobs.Count == ikJoints.Count)
        {
            float oldAngle;
            switch (xrKnobs[jointIndex].rotationAxis)
            {
                case KnobAxis.X:
                    oldAngle = ikJoints[jointIndex].transform.localEulerAngles.x;
                    break;
                case KnobAxis.Y:
                    oldAngle = ikJoints[jointIndex].transform.localEulerAngles.y;
                    break;
                case KnobAxis.Z:
                    oldAngle = ikJoints[jointIndex].transform.localEulerAngles.z;
                    break;
                case KnobAxis.NegativeX:
                    oldAngle = -ikJoints[jointIndex].transform.localEulerAngles.x;
                    break;
                case KnobAxis.NegativeY:
                    oldAngle = -ikJoints[jointIndex].transform.localEulerAngles.y;
                    break;
                case KnobAxis.NegativeZ:
                    oldAngle = -ikJoints[jointIndex].transform.localEulerAngles.z;
                    break;
                default:
                    oldAngle = ikJoints[jointIndex].transform.localEulerAngles.y;
                    break;
            }
            float diff = AngleDifference(oldAngle, jointAngle);

            // Subtract the difference from the knob angle
            xrKnobs[jointIndex].jointAngle -= diff;
        } else {
            Debug.LogError("Invalid number of angles or knobs");
        }
    }

    public void SetJointAngle(string jointName, float jointAngle, bool ignoreNotFound=true)
    {
        // Try to set the joint if it exists
        try {
            int jointIndex = nameToNumber.Find(x => x.Item1 == jointName).Item2;
            SetJointAngle(jointIndex, jointAngle);
        } catch (Exception)
        {
            if (!ignoreNotFound) {
                // Try prepending "KnobParent_" to the joint name
                jointName = "KnobParent_" + jointName;
                try {
                    int jointIndex = nameToNumber.Find(x => x.Item1 == jointName).Item2;
                    SetJointAngle(jointIndex, jointAngle);
                } catch (Exception)
                {
                    Debug.LogError("Joint not found: " + jointName);
                }
            }
        }

        // Check for joints that mimic this joint
        if (mimicJointMap.ContainsKey(jointName))
        {
            foreach (string mimicJoint in mimicJointMap[jointName])
            {
                double offset = mimicJointOffsetMap[mimicJoint];
                double multiplier = mimicJointMultiplierMap[mimicJoint];
                SetJointAngle(mimicJoint, (float)(jointAngle * multiplier + offset), false);
            }
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
