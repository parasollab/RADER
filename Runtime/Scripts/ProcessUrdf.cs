
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
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
    public GameObject urdfModel;  // Reference to the base of the robot's URDF model
    public GameObject gripper;  // Reference to the gripper object if any
    public GameObject graspedObject;  // Reference to the object being grasped if any
    public Vector3 graspedObjectPosition = Vector3.zero;  // Local position of the grasped object
    public Vector3 graspedObjectRotation = Vector3.zero;  // Local rotation of the grasped object in Euler angles

    public GameObject target;  // Reference to the target object for the CCDIK
    public GameObject menuUI;  // Reference to the meanu UI prefab
    private List<KeyValuePair<GameObject, GameObject>> reparentingList = new List<KeyValuePair<GameObject, GameObject>>();

    private List<Tuple<float, float>> jointLimits = new List<Tuple<float, float>>();
    
    private List<bool> clampedMotionList = new List<bool>();

    private List<CCDIKJoint> ccdikJoints = new List<CCDIKJoint>();
    public ColorAffordanceThemeDatumProperty affordanceThemeDatum;

    // variables for sending messages to ROS
    public ROSConnection ros;
    public string queryTopic = "/joint_query";
    public string trajectoryTopic = "/joint_trajectory";
    public string mirrorInputTopic = "/physical_joint_state";
    public string stateOutputTopic = "/virtual_joint_state";
    public string interactionTopic = "/interaction";
    public float demoRecordRate = 0.01f;
    public float publishStateRate = 0.2f;
    protected List<Transform> knobs = new List<Transform>();
    protected List<XRKnobAlt> knobObjs = new List<XRKnobAlt>();

    private List<double> jointPositions = new List<double>();

    private List<string> jointNames = new List<string>();
    public bool saveAsPrefab = false;
    private int jointCount = 0;
    private GameObject grabJoint;

    private SetupUI setupUI;

    void Awake()
    {
        Debug.Log("ProcessUrdf Awake");
        if (urdfModel != null)
        {
            TraverseAndModify(urdfModel);
            reParent();

            GameObject lastChild = reparentingList[reparentingList.Count - 1].Key;
            GameObject lastLink = findRealLastChild(lastChild);
            createTarget(lastLink);

            // Instantiate and attach the gripper prefab as a child to the last link
            if (gripper != null)
            {
                GameObject gripperInstance = AddGripper(lastLink);
                TraverseAndModify(gripperInstance);
            }

            // Instantiate and attach the grasped object prefab as a child to the last link
            if (graspedObject != null)
            {
                AddGraspedObject(lastLink);
            }

            SetupGrabBase setupBase = urdfModel.AddComponent<SetupGrabBase>();
            setupBase.Base = grabJoint;

            urdfModel.AddComponent<SetupIK>();
            urdfModel.AddComponent<SetupUI>();

            setupUI = urdfModel.GetComponent<SetupUI>();
            setupUI.ros = ros;
            setupUI.trajTopicName = trajectoryTopic;
            setupUI.queryTopicName = queryTopic;
            setupUI.inputStateTopicName = mirrorInputTopic;
            setupUI.outputStateTopicName = stateOutputTopic;
            setupUI.interactionTopicName = interactionTopic;
            setupUI.knobTransforms = knobs;
            setupUI.knobs = knobObjs;
            setupUI.jointNames = jointNames;
            setupUI.menuUI = menuUI;
            setupUI.recordInterval = demoRecordRate;
            setupUI.publishStateInterval = publishStateRate;
            
            Debug.Log("SetupUI done");

            #if UNITY_EDITOR
            savePrefab(urdfModel.name);
            #endif
        }
    }

    GameObject AddGripper(GameObject lastLink)
    {
        // Instantiate the gripper prefab and attach it to the last link
        GameObject gripperInstance = Instantiate(gripper, lastLink.transform.position, lastLink.transform.rotation);
        gripperInstance.name = "Gripper"; // Optional: Rename the gripper instance
        gripperInstance.transform.SetParent(lastLink.transform);
        gripperInstance.transform.localPosition = Vector3.zero;
        gripperInstance.transform.localRotation = Quaternion.identity;

        return gripperInstance;
    }

    void AddGraspedObject(GameObject lastLink)
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
            DestroyImmediate(script); 
        }

        var articulationBody = obj.GetComponent<ArticulationBody>();

        if (articulationBody != null)
        {
            jointCount++;
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
            if(jointCount == 2)  grabJoint = obj;

        }
    }


    void reParent()
    {
        for (int i = reparentingList.Count - 1; i >= 0; i--)
        {
            var pair = reparentingList[i];
            GameObject child = pair.Key;
            GameObject knobParent = pair.Value;
            jointNames.Add(child.name);

            knobParent.transform.position = child.transform.position;
            knobParent.transform.rotation = child.transform.rotation;

            // // Set the new parent
            child.transform.parent = knobParent.transform;

            // zero out child's local position and rotation
            // child.transform.localPosition = Vector3.zero;
            // child.transform.localRotation = Quaternion.identity;

            // // Add CCDIK components to the child, and add references to the list
            CCDIKJoint ccdik = child.AddComponent<CCDIKJoint>();
            ccdik.axis = new Vector3(0, 1, 0);
            ccdikJoints.Add(ccdik);


            // // Add the XRKnobAlt
            XRKnobAlt knob = knobParent.AddComponent<XRKnobAlt>();
            knob.uniqueID = i;
            // knob.clampedMotion = clampedMotionList[i];
            knob.jointMinAngle = jointLimits[i].Item1;
            knob.jointMaxAngle = jointLimits[i].Item2;

            // Debug.LogError("Parent " + knobParent.name + " Joint " + knob.name + " upper limit: " + knob.maxAngle + " lower limit: " + knob.minAngle);


            knob.handle = child.transform;
            
            createInteractionAffordance(child, knob, knobParent);

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

    void createInteractionAffordance(GameObject child, XRKnobAlt knob, GameObject knobParent)
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

    void createTarget(GameObject lastLink)
    {
        // create target object for the last child
        GameObject target = Instantiate(this.target, lastLink.transform.position, lastLink.transform.rotation);
        target.name = "target";
        target.transform.SetParent(lastLink.transform);
        target.transform.localPosition = Vector3.zero;
        target.transform.localRotation = Quaternion.identity;

    }

    GameObject findRealLastChild(GameObject lastChild) {
        foreach (Transform child in lastChild.transform) {
            if (child.gameObject.GetNamedChild("Collisions") != null && child.gameObject.GetNamedChild("Visuals") != null) {
                return findRealLastChild(child.gameObject);
            }
        }
        return lastChild;
    }


    void savePrefab(string name)
    {
        // Save the prefab
        string prefabPath = "Assets/Prefabs/"+name+".prefab";
        #if UNITY_EDITOR
        GameObject prefab = PrefabUtility.SaveAsPrefabAssetAndConnect(urdfModel, prefabPath, InteractionMode.AutomatedAction);
        #endif
    }
}