using System.Collections.Generic;
using Unity.VRTemplate;
using Unity.XR.CoreUtils;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;


public class SetupIK : MonoBehaviour
{
    private List<CCDIKJoint> ccdikJoints = new List<CCDIKJoint>();
    private List<XRKnobAlt> xrKnobs = new List<XRKnobAlt>();
    private GameObject lastChild;

    void Start()
    {
        TraverseAndAnalyze(this.gameObject);
        if(lastChild == null)
        {
            Debug.LogError("No XRKnob found in children, error in prefabSetup");
            return;
        }
        if(ccdikJoints.Count == 0)
        {
            Debug.LogError("No CCDIKJoint found in children, error in prefabSetup");
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
            ccdikJoints.Add(obj.GetComponent<CCDIKJoint>());
        }
        
        // Recursively process each child
        foreach (Transform child in obj.transform)
        {
            TraverseAndAnalyze(child.gameObject);
        }
    }

    void setupIK(GameObject lastChild)
    {
        // ccdIK
        CCDIK ccdIK = lastChild.AddComponent<CCDIK>();
        
        ccdIK.joints = ccdikJoints.ToArray();
        ccdIK.knobs = xrKnobs.ToArray();
        ccdIK.Tooltip = findRealLastChild(lastChild.transform);
    }

    public void InverseKinematics(Transform target)
    {
        lastChild.GetComponent<CCDIK>().InverseKinematics(target);
    }

    Transform findRealLastChild(Transform lastChild) {
        foreach (Transform child in lastChild) {
            if (child.gameObject.GetNamedChild("Collisions") != null && child.gameObject.GetNamedChild("Visuals") != null) {
                return findRealLastChild(child);
            }
        }
        return lastChild;
    }
}
