
using System.Collections.Generic;
using Unity.VRTemplate;
using UnityEngine;
using UnityEditor;
using System;
using UnityEngine.XR.Interaction.Toolkit.AffordanceSystem.State;
using UnityEngine.XR.Interaction.Toolkit.AffordanceSystem.Receiver.Rendering;
using UnityEngine.XR.Interaction.Toolkit.AffordanceSystem.Rendering;
using UnityEngine.XR.Interaction.Toolkit.AffordanceSystem.Theme.Primitives;
using Unity.XR.CoreUtils;

public class ProcessUrdf : MonoBehaviour
{
    private List<KeyValuePair<GameObject, GameObject>> reparentingList = new List<KeyValuePair<GameObject, GameObject>>();

    private List<Tuple<float, float>> jointLimits = new List<Tuple<float, float>>();
    
    private List<bool> clampedMotionList = new List<bool>();

    protected List<Transform> knobs = new List<Transform>();
    protected List<XRKnobAlt> knobObjs = new List<XRKnobAlt>();

    private List<double> jointPositions = new List<double>();

    private List<string> jointNames = new List<string>();
    private List<GameObject> jointList = new List<GameObject>();
    public bool saveAsPrefab = false;
    private GameObject grabJoint;

    private GameObject lastLink;

    // Getter for the last link
    public GameObject LastLink
    {
        get { return lastLink; }
    }

    public void ProcessModel(GameObject urdfModel, GameObject gripper, GameObject graspedObject, 
        Vector3 graspedObjectPosition, Vector3 graspedObjectRotation,
        ColorAffordanceThemeDatumProperty affordanceThemeDatum,
        IKSolver ikSolver)
    {
        if (urdfModel == null)
        {
            Debug.LogError("No URDF model found, error in prefabSetup");
            return;
        }

        TraverseAndModify(urdfModel);
        reParent(affordanceThemeDatum);

        GameObject lastChild = reparentingList[reparentingList.Count - 1].Key;
        lastLink = findRealLastChild(lastChild);

        urdfModel.AddComponent<SetupGrabBase>();
        SetupGrabBase setupBase = urdfModel.GetComponent<SetupGrabBase>();
        setupBase.Base = grabJoint;

        // Instantiate and attach the gripper prefab as a child to the last link
        if (gripper != null)
        {
            GameObject gripperInstance = AddGripper(gripper, lastLink);
            TraverseAndModify(gripperInstance);
        }

        // Instantiate and attach the grasped object prefab as a child to the last link
        if (graspedObject != null)
        {
            AddGraspedObject(graspedObject, lastLink, graspedObjectPosition, graspedObjectRotation);
        }

        urdfModel.AddComponent<SetupIK>();
        SetupIK setupIK = urdfModel.GetComponent<SetupIK>();
        setupIK.ikSolver = ikSolver;
        setupIK.Initialize();

        #if UNITY_EDITOR
        savePrefab(urdfModel, urdfModel.name);
        #endif
    }

    GameObject AddGripper(GameObject gripper, GameObject lastLink)
    {
        // Instantiate the gripper prefab and attach it to the last link
        GameObject gripperInstance = Instantiate(gripper, lastLink.transform.position, lastLink.transform.rotation);
        gripperInstance.name = "Gripper"; // Optional: Rename the gripper instance
        gripperInstance.transform.SetParent(lastLink.transform);
        gripperInstance.transform.localPosition = Vector3.zero;

        // Rotate the gripper by 90 degrees around the y-axis
        gripperInstance.transform.localRotation = Quaternion.Euler(0, 90, 0);
        return gripperInstance;
    }

    void AddGraspedObject(GameObject graspedObject, GameObject lastLink, Vector3 graspedObjectPosition, Vector3 graspedObjectRotation)
    {
        // Instantiate the grasped object prefab and attach it to the last link
        GameObject graspedObjectInstance = Instantiate(graspedObject, lastLink.transform.position, lastLink.transform.rotation);
        graspedObjectInstance.name = "GraspedObject"; // Optional: Rename the grasped object instance
        graspedObjectInstance.transform.SetParent(lastLink.transform);
        graspedObjectInstance.transform.localPosition = graspedObjectPosition;  // These fields can be edited in the Unity Inspector
        graspedObjectInstance.transform.localRotation = Quaternion.Euler(graspedObjectRotation);
    }

    void TraverseAndModify(GameObject obj)
    {
        if (obj == null) return;

        string name = obj.name;
        // check if link in name
        if (name.Contains("link"))
        {
            obj.name = name.Replace("link", "joint");
        }
        // Process the current object
        RemoveAndModifyComponents(obj);
        
        // Recursively process each child
        foreach (Transform child in obj.transform)
        {
            TraverseAndModify(child.gameObject);
        }
    }

    void RemoveAndModifyComponents(GameObject obj)
    {
        var scripts = new List<MonoBehaviour>(obj.GetComponents<MonoBehaviour>());
        bool fixedJoint = false;
        foreach (var script in scripts)
        {   
            fixedJoint = script.GetType().Name == "UrdfJointFixed";
            
            // Do not delete scripts of type RobotManager
            if (script.GetType().Name == "RobotManager") continue;

            // Do not delete scripts that inherit from IKSolver
            if (script.GetType().IsSubclassOf(typeof(IKSolver))) continue;

            DestroyImmediate(script); 
        }

        var articulationBody = obj.GetComponent<ArticulationBody>();

        if (articulationBody != null)
        {
            bool isClampedMotion = articulationBody.xDrive.upperLimit - articulationBody.xDrive.lowerLimit < 360;
            // bool isClampedMotion = (articulationBody.xDrive.upperLimit != 0) && (articulationBody.xDrive.lowerLimit != 0);
            Tuple<float, float> jointLimit = new Tuple<float, float>(articulationBody.xDrive.lowerLimit, articulationBody.xDrive.upperLimit);
            // Debug.LogAssertion("Joint " + obj.name + " has 0 range of motion, setting to 360");
            // Debug.LogError("Joint " + obj.name + " upper limit: " + articulationBody.xDrive.upperLimit + " lower limit: " + articulationBody.xDrive.lowerLimit);
            // Debug.LogError("Joint " + obj.name + " isClampedMotion: " + isClampedMotion);


            if (articulationBody.xDrive.upperLimit - articulationBody.xDrive.lowerLimit == 0 && articulationBody.jointType == ArticulationJointType.RevoluteJoint) {
                // Debug.LogError("Joint " + obj.name + " has 0 range of motion, setting to 360");
                isClampedMotion = false;
                jointLimit = new Tuple<float, float>(0, 360);
            }
            
            DestroyImmediate(articulationBody);

            // add rigidbody
            var rb = obj.AddComponent<Rigidbody>();
            rb.mass = 1.0f;
            rb.useGravity = false;
            rb.isKinematic = true;
            // if fixedJoint we dont add XRGrabInteractable
            if(!fixedJoint)
            {
                GameObject originalParent = obj.transform.parent.gameObject;
                GameObject knobParent = new GameObject("KnobParent_" + obj.name);

                knobParent.transform.parent = originalParent.transform;

                // Store the object and its new parent for later re-parenting
                reparentingList.Add(new KeyValuePair<GameObject, GameObject>(obj, knobParent));
                clampedMotionList.Add(isClampedMotion);
                jointLimits.Add(jointLimit);
            }

            if (grabJoint == null) {
                MeshCollider meshCollider = obj.GetComponentInChildren<MeshCollider>();
                if (meshCollider != null) {
                    grabJoint = obj;
                }

            }
        }
    }

    void reParent(ColorAffordanceThemeDatumProperty affordanceThemeDatum)
    {
        for (int i = reparentingList.Count - 1; i >= 0; i--)
        {
            var pair = reparentingList[i];
            GameObject child = pair.Key;
            GameObject knobParent = pair.Value;
            jointNames.Add(child.name);
            jointList.Add(child);

            knobParent.transform.position = child.transform.position;
            knobParent.transform.rotation = child.transform.rotation;

            // // Set the new parent
            child.transform.parent = knobParent.transform;

            // zero out child's local position and rotation
            // child.transform.localPosition = Vector3.zero;
            // child.transform.localRotation = Quaternion.identity;

            // // Add IK components to the child, and add references to the list
            CCDIKJoint ik = child.AddComponent<CCDIKJoint>();
            ik.axis = new Vector3(0, 1, 0);

            // // Add the XRKnobAlt
            XRKnobAlt knob = knobParent.AddComponent<XRKnobAlt>();
            knob.uniqueID = i;
            // knob.clampedMotion = clampedMotionList[i];
            knob.jointMinAngle = jointLimits[i].Item1;
            knob.jointMaxAngle = jointLimits[i].Item2;

            // Debug.LogError("Parent " + knobParent.name + " Joint " + knob.name + " upper limit: " + knob.maxAngle + " lower limit: " + knob.minAngle);


            knob.handle = child.transform;
            
            createInteractionAffordance(child, knob, knobParent, affordanceThemeDatum);

            // Use .Prepend to reverse the joint order
            knobObjs.Insert(0, knob);
            knobs.Add(child.transform);
            jointPositions.Add(child.transform.localRotation.eulerAngles.y);

            // Debug.Log(child.name + " " + child.transform.localRotation.eulerAngles.y);

            // // Check for MeshCollider on the child or its descendants
            MeshCollider meshCollider = child.GetComponent<MeshCollider>();
            if (meshCollider == null)
            {
                meshCollider = child.GetComponentInChildren<MeshCollider>();
            }

            // Clear existing colliders and add the found one if any
            knob.colliders.Clear();
            if (meshCollider != null)
            {
                knob.colliders.Add(meshCollider);
            }
        }
        jointNames.Reverse();
    }

    public void SetHomePosition() // set current joint positions as home position
    {
        for (int i = 0; i < jointList.Count; i++)
        {
            jointPositions[i] = jointList[i].transform.localRotation.eulerAngles.y;
        }
    }

    public void ResetHomePosition() // reset joint positions to home position
    {
        for (int i = 0; i < jointList.Count; i++)
        {
            jointList[i].transform.localRotation = Quaternion.Euler(0, (float)jointPositions[i], 0);
        }
    }

    public List<string> GetJointNames()
    {
        return jointNames;
    }

    public List<Tuple<float, float>> GetJointLimits()
    {
        return jointLimits;
    }

    void createInteractionAffordance(GameObject child, XRKnobAlt knob, GameObject knobParent, ColorAffordanceThemeDatumProperty affordanceThemeDatum)
    {
        // create interaction affordance
            GameObject knobAffordance = new GameObject("KnobAffordance");
            knobAffordance.transform.parent = knobParent.transform;
            XRInteractableAffordanceStateProvider affordanceProvider = knobAffordance.AddComponent<XRInteractableAffordanceStateProvider>();
            affordanceProvider.interactableSource = knob;
            affordanceProvider.activateClickAnimationMode = XRInteractableAffordanceStateProvider.ActivateClickAnimationMode.Activated;



            GameObject colorAffordance = new GameObject("ColorAffordance");
            colorAffordance.transform.parent = knobAffordance.transform;

            // add xr interaction affordance receiver
            
            ColorMaterialPropertyAffordanceReceiver colorMaterialPropertyAffordanceReceiver = colorAffordance.AddComponent<ColorMaterialPropertyAffordanceReceiver>();
            colorMaterialPropertyAffordanceReceiver.replaceIdleStateValueWithInitialValue = true;
            MaterialPropertyBlockHelper materialPropertyBlockHelper = colorAffordance.GetComponent<MaterialPropertyBlockHelper>();
            colorMaterialPropertyAffordanceReceiver.affordanceThemeDatum = affordanceThemeDatum;
            MeshRenderer[] meshRenderers = child.GetComponentsInChildren<MeshRenderer>();
            materialPropertyBlockHelper.rendererTarget = meshRenderers[0];
            materialPropertyBlockHelper.enabled = true;
    }

    GameObject findRealLastChild(GameObject lastChild) {
        foreach (Transform child in lastChild.transform) {
            if (child.gameObject.GetNamedChild("Collisions") != null && child.gameObject.GetNamedChild("Visuals") != null) {
                return findRealLastChild(child.gameObject);
            }
        }
        return lastChild;
    }

    void savePrefab(GameObject urdfModel, string name)
    {
        // Save the prefab
        string prefabPath = "Assets/Prefabs/"+name+".prefab";
        #if UNITY_EDITOR
        GameObject prefab = PrefabUtility.SaveAsPrefabAssetAndConnect(urdfModel, prefabPath, InteractionMode.AutomatedAction);
        #endif
    }
}