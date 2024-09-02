using System;
using System.Collections.Generic;
using TMPro;
using Unity.VRTemplate;
using Unity.XR.CoreUtils;
using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Trajectory;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.Hri;
using UnityEngine.Assertions;
// using System.Diagnostics;

public class SetupUI : MonoBehaviour
{
    public ROSConnection ros;
    public GameObject menuUI;
    public String trajTopicName = "/joint_trajectory";
    public String queryTopicName = "/joint_query";
    public String inputStateTopicName = "/joint_states";
    public String outputStateTopicName = "/virtual_joint_state";
    public String interactionTopicName = "/interaction";
    public List<Transform> knobTransforms;
    public List<XRKnobAlt> knobs;
    public List<double> jointPositions;
    public List<String> jointNames;
    public List<int> jointSigns = new List<int>();
    public float recordInterval = 0.1f;
    public float publishStateInterval = 0.2f;

    private bool recordROS = false;
    private List<double> startPositions;
    private List<double> goalPositions;
    private List<JointTrajectoryPointMsg> jointTrajectoryPoints = new List<JointTrajectoryPointMsg>();
    private bool mirrorInputState = false;
    // private bool mirrorInputState = true;
    private bool publishState = true;

    private GameObject startButtonObject;
    private GameObject goalButtonObject;
    private GameObject queryButtonObject;

    private GameObject recordButtonObject;
    private int recordStartTime;

    private GameObject mirrorButtonObject;
    private GameObject publishStateButtonObject;

    private List<double> jointTorques;
    private Dictionary<Transform, float> previousAngles = new Dictionary<Transform, float>();
    private Dictionary<Transform, float> momentsOfInertia = new Dictionary<Transform, float>();

    void Start()
    {
        Debug.Log("SetupUI Start");
        if (ros == null) ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointQueryMsg>(queryTopicName);
        ros.RegisterPublisher<JointTrajectoryMsg>(trajTopicName);
        ros.RegisterPublisher<JointStateMsg>(outputStateTopicName);
        ros.RegisterPublisher<BoolMsg>(interactionTopicName);
        ros.Subscribe<JointStateMsg>(inputStateTopicName, MirrorStateCallback);

        LoadUI();
        
        InitializeKnobData();
    }

    void InitializeKnobData()
    {
        foreach (var knob in knobTransforms)
        {
            previousAngles[knob] = knob.transform.localEulerAngles.y;
            momentsOfInertia[knob] = 0.0005f; // Example value, adjust as necessary
        }
    }

    public void startMirroring()
    {
        mirrorInputState = true;
        TextMeshProUGUI mirrorButtonText = mirrorButtonObject.GetNamedChild("Button Front").GetNamedChild("Text (TMP) ").GetComponent<TextMeshProUGUI>();
        mirrorButtonText.text = "Stop Mirroring";

        // Let the planner know that interaction is not happening
        sendInteractionMessage(false);
    }

    public void stopMirroring()
    {
        mirrorInputState = false;
        TextMeshProUGUI mirrorButtonText = mirrorButtonObject.GetNamedChild("Button Front").GetNamedChild("Text (TMP) ").GetComponent<TextMeshProUGUI>();
        mirrorButtonText.text = "Start Mirroring";

        // Let the planner know that interaction is starting
        sendInteractionMessage(true);
    }

    // TODO this is hardcoded for the UR5e, need to generalize
    void MirrorStateCallback(JointStateMsg jointState)
    {
        // Debug.Log("MirrorStateCallback");
        if (mirrorInputState)
        {
            knobs[0].jointAngle = -((float)jointState.position[5] * Mathf.Rad2Deg);
            knobs[1].jointAngle = -((float)jointState.position[0] * Mathf.Rad2Deg);
            knobs[2].jointAngle = -((float)jointState.position[1] * Mathf.Rad2Deg);
            knobs[3].jointAngle = -((float)jointState.position[2] * Mathf.Rad2Deg);
            knobs[4].jointAngle = -((float)jointState.position[3] * Mathf.Rad2Deg);
            knobs[5].jointAngle = -((float)jointState.position[4] * Mathf.Rad2Deg);
        }
    }

    void sendInteractionMessage(bool interaction)
    {
        BoolMsg interactionMsg = new BoolMsg
        {
            data = interaction
        };
        ros.Publish(interactionTopicName, interactionMsg);
    }

    void PublishState()
    {
        if (publishState)
        {
            for (int i = 0; i < knobTransforms.Count; i++)
            {
                jointPositions[i] = -knobs[i].jointAngle * Mathf.Deg2Rad;
            }
           
            // Publish the joint state message
            JointStateMsg jointState = new JointStateMsg
            {
                name = jointNames.ToArray(),
                position = jointPositions.ToArray()
            };
            ros.Publish(outputStateTopicName, jointState);
        }
    }

    void setStart()
    {
        startPositions = new List<double>();
        for (int i = 0; i < knobTransforms.Count; i++)
        {
            startPositions.Add(knobTransforms[i].transform.localRotation.eulerAngles.y * (float)Math.PI / 180.0);
        }

        CheckQueryButton();
    }

    void setGoal()
    {
        goalPositions = new List<double>();
        for (int i = 0; i < knobTransforms.Count; i++)
        {
            goalPositions.Add(knobTransforms[i].transform.localRotation.eulerAngles.y * (float)Math.PI / 180.0);
        }

        CheckQueryButton();
    }

    void CheckQueryButton()
    {
        if (startPositions != null && goalPositions != null)
        {
            Debug.Log("Both start and goal are set");

            // Enable the query send button
            Button queryButton = queryButtonObject.GetComponent<Button>();
            queryButton.interactable = true;
        }
    }

    void sendQueryMessage()
    {
        JointStateMsg start = new JointStateMsg
        {
            name = jointNames.ToArray(),
            position = startPositions.ToArray()
        };

        JointStateMsg goal = new JointStateMsg
        {
            name = jointNames.ToArray(),
            position = goalPositions.ToArray()
        };

        JointQueryMsg jointQuery = new JointQueryMsg
        {
            start = start,
            goal = goal
        };
        ros.Publish(queryTopicName, jointQuery);

        // Also enable the record button
        Button recordButton = recordButtonObject.GetComponent<Button>();
        recordButton.interactable = true;
    }

    void LoadUI()
    {
        if (menuUI == null)
        {
            Debug.LogError("error loading UI");
        }
        menuUI = Instantiate(menuUI, transform);

        // Load the interface for recording demos and setting joint angles
        GameObject contentGameObject = menuUI.GetNamedChild("Spatial Panel Scroll").GetNamedChild("Robot Scroll View").GetNamedChild("Viewport").GetNamedChild("Content");

        // Record button
        recordButtonObject = contentGameObject.GetNamedChild("Record Button").GetNamedChild("Text Poke Button");
        Button button = recordButtonObject.GetComponent<Button>();
        TextMeshProUGUI buttonText = recordButtonObject.GetNamedChild("Button Front").GetNamedChild("Text (TMP) ").GetComponent<TextMeshProUGUI>();

        // Discard button
        GameObject discardButtonObject = contentGameObject.GetNamedChild("Discard Button").GetNamedChild("Text Poke Button");
        Button discardButton = discardButtonObject.GetComponent<Button>();
        
        discardButton.onClick.AddListener(() =>
        {
            resetJointPositionMessage();
            if (recordROS == true)
            {
                recordROS = false;
                buttonText.text = "Start Recording";
            }
        });

        discardButton.interactable = false;

        // Record button functionality

        button.onClick.AddListener(() =>
        {
            if (recordROS == true)
            {
                recordROS = false;
                discardButton.interactable = false;
                buttonText.text = "Start Recording";
                sendJointPositionMessage();

                // Let the planner know that interaction is starting
                sendInteractionMessage(true);
            }
            else
            {
                recordROS = true;
                discardButton.interactable = true;
                buttonText.text = "Send Recording";
            }
        });

        // (Don't) Disable the button until a query is sent
        button.interactable = true;

        // dropdown and slider
        TMP_Dropdown dropdown = contentGameObject.GetNamedChild("List Item Dropdown").GetNamedChild("Dropdown").GetComponent<TMP_Dropdown>();
        Slider slider = contentGameObject.GetNamedChild("List Item Slider").GetNamedChild("MinMax Slider").GetComponent<Slider>();
        TextMeshProUGUI sliderText = slider.gameObject.GetNamedChild("Value Text").GetComponent<TextMeshProUGUI>();

        // Populate the dropdown with the joint names reversed without changing the jointNames list
        List<string> reversedJointNames = new List<string>(jointNames);
        reversedJointNames.Reverse();
        dropdown.AddOptions(reversedJointNames);
        
        int dropdownIndex = 0;
        int knobID = knobs[dropdownIndex].uniqueID;
        slider.value = knobTransforms[dropdownIndex].GetComponentInParent<XRKnobAlt>().jointAngle;
        sliderText.text = (-knobTransforms[dropdownIndex].GetComponentInParent<XRKnobAlt>().jointAngle).ToString();
        slider.minValue = knobs[dropdownIndex].jointMinAngle;
        slider.maxValue = knobs[dropdownIndex].jointMaxAngle;

        dropdown.onValueChanged.AddListener(delegate
        {
            dropdownIndex = dropdown.value;
            knobID = knobs[dropdownIndex].uniqueID;
            
            slider.value = knobTransforms[dropdownIndex].GetComponentInParent<XRKnobAlt>().jointAngle;
            // Debug.Log("minAngle: " + knobs[dropdownIndex].jointMinAngle + ", maxAngle: " + knobs[dropdownIndex].jointMaxAngle);
            slider.minValue = knobs[dropdownIndex].jointMinAngle;
            slider.maxValue = knobs[dropdownIndex].jointMaxAngle;
        });

        slider.onValueChanged.AddListener(delegate
        {
            knobTransforms[dropdownIndex].GetComponentInParent<XRKnobAlt>().jointAngle = slider.value;
            sliderText.text = (-knobTransforms[dropdownIndex].GetComponentInParent<XRKnobAlt>().jointAngle).ToString();
        });

        InvokeRepeating("addJointPosition", 1.0f, recordInterval);

        // Load the query interface
        contentGameObject = menuUI.GetNamedChild("Spatial Panel Scroll").GetNamedChild("Query Scroll View").GetNamedChild("Viewport").GetNamedChild("Content");

        startButtonObject = contentGameObject.GetNamedChild("Set Start Button").GetNamedChild("Text Poke Button");
        Button startButton = startButtonObject.GetComponent<Button>();
        TextMeshProUGUI startButtonText = startButtonObject.GetNamedChild("Button Front").GetNamedChild("Text (TMP) ").GetComponent<TextMeshProUGUI>();

        startButton.onClick.AddListener(() =>
        {
            setStart();
            startButtonText.text = "Start is Set!";
        });

        goalButtonObject = contentGameObject.GetNamedChild("Set Goal Button").GetNamedChild("Text Poke Button");
        Button goalButton = goalButtonObject.GetComponent<Button>();
        TextMeshProUGUI goalButtonText = goalButtonObject.GetNamedChild("Button Front").GetNamedChild("Text (TMP) ").GetComponent<TextMeshProUGUI>();

        goalButton.onClick.AddListener(() =>
        {
            setGoal();
            goalButtonText.text = "Goal is Set!";
        });

        queryButtonObject = contentGameObject.GetNamedChild("Send Query Button").GetNamedChild("Text Poke Button");
        Button queryButton = queryButtonObject.GetComponent<Button>();
        TextMeshProUGUI queryButtonText = queryButtonObject.GetNamedChild("Button Front").GetNamedChild("Text (TMP) ").GetComponent<TextMeshProUGUI>();

        queryButton.onClick.AddListener(() =>
        {
            sendQueryMessage();
            queryButtonText.text = "Query Sent!";
        });

        // Disable the query button until both start and goal are set
        queryButton.interactable = false;

        // Load the mirror interface
        contentGameObject = menuUI.GetNamedChild("Spatial Panel Scroll").GetNamedChild("Mirror Scroll View").GetNamedChild("Viewport").GetNamedChild("Content");

        // Mirror input button
        mirrorButtonObject = contentGameObject.GetNamedChild("Mirror Input Button").GetNamedChild("Text Poke Button");
        Button mirrorButton = mirrorButtonObject.GetComponent<Button>();
        TextMeshProUGUI mirrorButtonText = mirrorButtonObject.GetNamedChild("Button Front").GetNamedChild("Text (TMP) ").GetComponent<TextMeshProUGUI>();

        mirrorButton.onClick.AddListener(() =>
        {
            Debug.Log("mirrorButton.onClick");
            if (mirrorInputState)
            {
                stopMirroring();
            }
            else
            {
                Debug.Log("startMirroring");
                startMirroring();
            }
        });

        // Publish state button
        publishStateButtonObject = contentGameObject.GetNamedChild("Publish State Button").GetNamedChild("Text Poke Button");
        Button publishStateButton = publishStateButtonObject.GetComponent<Button>();
        TextMeshProUGUI publishStateButtonText = publishStateButtonObject.GetNamedChild("Button Front").GetNamedChild("Text (TMP) ").GetComponent<TextMeshProUGUI>();

        publishStateButton.onClick.AddListener(() =>
        {
            Debug.Log("publishStateButton.onClick");
            if (publishState)
            {
                publishState = false;
                publishStateButtonText.text = "Start Publishing";
            }
            else
            {
                publishState = true;
                publishStateButtonText.text = "Stop Publishing";
            }
        });

        InvokeRepeating("PublishState", 1.0f, publishStateInterval);
    }

    void addJointPosition()
    {
        if (recordROS)
        {
            if (recordStartTime == 0)
            {
                recordStartTime = (int)Time.time;
            }

            // List<double> jointAnglesWithinLimits = getJointAnglesWithinLimits();
            jointTorques = new List<double>();
            for (int i = 0; i < knobTransforms.Count; i++)
            {
                jointPositions[i] = -knobs[i].jointAngle * Mathf.Deg2Rad;
                
                // Get the torque from the knob
                float torque = CalculateTorque(knobTransforms[i]);
                jointTorques.Add(torque);
            }

            JointTrajectoryPointMsg jointTrajectoryPoint = new JointTrajectoryPointMsg
            {
                positions = jointPositions.ToArray(),
                effort = jointTorques.ToArray(),
                time_from_start = new DurationMsg((int)Time.time - recordStartTime, 0),
            };
            jointTrajectoryPoints.Add(jointTrajectoryPoint);
        }
    }

    float CalculateTorque(Transform knob)
    {
        float currentAngle = knob.transform.localEulerAngles.y;
        float previousAngle = previousAngles[knob];
        float deltaAngle = Mathf.DeltaAngle(previousAngle, currentAngle);
        float angularVelocity = deltaAngle / Time.deltaTime;
        float torque = momentsOfInertia[knob] * angularVelocity;
        previousAngles[knob] = currentAngle;
        return torque;
    }

    void sendJointPositionMessage()
    {
        JointTrajectoryMsg jointTrajectory = new JointTrajectoryMsg();

        HeaderMsg header = new HeaderMsg
        {
            frame_id = gameObject.name,
            stamp = new TimeMsg
            {
                sec = (int)Time.time,
                nanosec = (uint)((Time.time - (int)Time.time) * 1e9)
            }
        };
        jointTrajectory.header = header;
        jointTrajectory.joint_names = jointNames.ToArray();
        jointTrajectory.points = jointTrajectoryPoints.ToArray();
        ros.Publish(trajTopicName, jointTrajectory);

        // Clear the jointTrajectoryPoints list
        resetJointPositionMessage();

        // Let the planner know that interaction is done
        sendInteractionMessage(false);
    }

    List<double> getJointAnglesWithinLimits()
    {
        List<double> jointAnglesWithinLimits = new List<double>();
        for (int i = 0; i < knobs.Count; i++)
        {
            // Scale and shift the value to be within the joint limits
            double value = knobs[i].jointAngle;
            double jointAngle = knobs[i].jointMinAngle + value * (knobs[i].jointMaxAngle - knobs[i].jointMinAngle);
            jointAnglesWithinLimits.Add(jointAngle);
        }
        return jointAnglesWithinLimits;
    }

    void resetJointPositionMessage()
    {
        jointTrajectoryPoints.Clear();
        recordStartTime = 0;
    }
}