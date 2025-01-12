using System;
using System.Collections.Generic;
using TMPro;
using Unity.VRTemplate;
using Unity.XR.CoreUtils;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Trajectory;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.Hri;
using System.Collections;

// TODO: Currently, only the first robot is used, need to generalize for multiple robots.

public class SetupUI : MonoBehaviour
{
    public GameObject menuUI;
    public List<GameObject> robotModels;
    public string trajTopicName = "/joint_trajectory";
    public string queryTopicName = "/joint_query";
    public string inputStateTopicName = "/joint_states";
    public string outputStateTopicName = "/virtual_joint_state";
    public string interactionTopicName = "/interaction";
    public float recordInterval = 0.1f;
    public float publishStateInterval = 0.2f;

    private List<string> jointNames;
    private List<Tuple<float, float>> jointLimits;

    private XRIDefaultInputActions inputActions;

    private ROSConnection ros;
    private bool recordROS = false;
    private List<double> startPositions;
    private List<double> goalPositions;
    private List<JointTrajectoryPointMsg> jointTrajectoryPoints = new List<JointTrajectoryPointMsg>();
    private bool mirrorInputState = false;
    private bool publishState = true;
    private List<JointTrajectoryMsg> trajectoryLog = new List<JointTrajectoryMsg>();

    private GameObject startButtonObject;
    private GameObject goalButtonObject;
    private GameObject queryButtonObject;

    private GameObject recordButtonObject;
    private float recordStartTime;

    private GameObject mirrorButtonObject;
    private GameObject publishStateButtonObject;

    private GameObject setHomeButtonObject;
    private GameObject goHomeButtonObject;
    private List<double> jointTorques;
    private Dictionary<int, float> previousAngles = new Dictionary<int, float>();
    private Dictionary<int, float> momentsOfInertia = new Dictionary<int, float>();
    private JointTrajectoryMsg lastTrajectory = null;

    private Button recordButton;
    private Button replayButton;
    private Button sendButton;
    private Button mirrorButton;

    void Start()
    {
        Debug.Log("SetupUI Start");
        if (ros == null) ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointQueryMsg>(queryTopicName);
        ros.RegisterPublisher<JointTrajectoryMsg>(trajTopicName);
        ros.RegisterPublisher<JointStateMsg>(outputStateTopicName);
        ros.RegisterPublisher<BoolMsg>(interactionTopicName);
        ros.Subscribe<JointStateMsg>(inputStateTopicName, MirrorStateCallback);

        inputActions = new XRIDefaultInputActions();
        inputActions.XRILeftHand.Enable();
        inputActions.XRIRightHand.Enable();

        inputActions.XRIRightHand.PressA.performed += SimulateRecordButtonClick;
        inputActions.XRIRightHand.PressB.performed += SimulateReplayButtonClick;
        inputActions.XRILeftHand.PressX.performed += SimulateMirrorButtonClick;

        // Load the joint information
        RobotManager robotManager = robotModels[0].GetComponent<RobotManager>();
        jointNames = robotManager.GetJointNames();
        jointLimits = robotManager.GetJointLimits();
        
        LoadUI();
        
        InitializeKnobData();
    }


    void InitializeKnobData()
    {
        RobotManager robotManager = robotModels[0].GetComponent<RobotManager>();
        float[] jointAngles = robotManager.GetJointAngles();

        for(int i = 0; i < jointAngles.Length; i++)
        {
            previousAngles[i] = jointAngles[i];
            momentsOfInertia[i] = 0.0005f; // Example value, adjust as necessary
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
        if (mirrorInputState)
        {
            float[] jointAngles = new float[jointState.position.Length];
            jointAngles[0] = -((float)jointState.position[5] * Mathf.Rad2Deg);
            jointAngles[1] = -((float)jointState.position[0] * Mathf.Rad2Deg);
            jointAngles[2] = -((float)jointState.position[1] * Mathf.Rad2Deg);
            jointAngles[3] = -((float)jointState.position[2] * Mathf.Rad2Deg);
            jointAngles[4] = -((float)jointState.position[3] * Mathf.Rad2Deg);
            jointAngles[5] = -((float)jointState.position[4] * Mathf.Rad2Deg);

            RobotManager robotManager = robotModels[0].GetComponent<RobotManager>();
            robotManager.SetJointAngles(jointAngles);
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
            RobotManager robotManager = robotModels[0].GetComponent<RobotManager>();
            float[] jointPositions = robotManager.GetJointAngles();


            jointTorques = new List<double>();
            for (int i = 0; i < jointPositions.Length; i++)
            {
                // Get the torque from the knob
                float torque = CalculateTorque(i, jointPositions[i] * Mathf.Deg2Rad);
                jointTorques.Add(torque);
            }
           
            // Publish the joint state message
            JointStateMsg jointState = new JointStateMsg
            {
                name = jointNames.ToArray(),
                position = Array.ConvertAll(jointPositions, item => (double)item),
                effort = jointTorques.ToArray()
            };
            ros.Publish(outputStateTopicName, jointState);
        }
    }

    void setStart()
    {
        RobotManager robotManager = robotModels[0].GetComponent<RobotManager>();
        float[] jointAngles = robotManager.GetJointAngles();

        startPositions = new List<double>();
        for (int i = 0; i < jointAngles.Length; i++)
        {
            startPositions.Add(jointAngles[i] * (float)Math.PI / 180.0);
        }

        CheckQueryButton();
    }

    void setGoal()
    {
        RobotManager robotManager = robotModels[0].GetComponent<RobotManager>();
        float[] jointAngles = robotManager.GetJointAngles();

        goalPositions = new List<double>();
        for (int i = 0; i < jointAngles.Length; i++)
        {
            goalPositions.Add(jointAngles[i] * (float)Math.PI / 180.0);
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
        recordButton = recordButtonObject.GetComponent<Button>();
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
        GameObject contentGameObject = menuUI.GetNamedChild("Spatial Panel Scroll")
            .GetNamedChild("Robot Scroll View")
            .GetNamedChild("Viewport")
            .GetNamedChild("Content");

        // Record button
        recordButtonObject = contentGameObject.GetNamedChild("Record Button")
            .GetNamedChild("Text Poke Button");
        recordButton = recordButtonObject.GetComponent<Button>();
        TextMeshProUGUI recordButtonText = recordButtonObject
            .GetNamedChild("Button Front")
            .GetNamedChild("Text (TMP) ")
            .GetComponent<TextMeshProUGUI>();

        // Send button
        GameObject sendButtonObject = contentGameObject.GetNamedChild("Send Button")
            .GetNamedChild("Text Poke Button");
        sendButton = sendButtonObject.GetComponent<Button>();
        TextMeshProUGUI sendButtonText = sendButtonObject
            .GetNamedChild("Button Front")
            .GetNamedChild("Text (TMP) ")
            .GetComponent<TextMeshProUGUI>();


        // Send button functionality
        sendButton.onClick.AddListener(() =>
        {
            if (lastTrajectory != null)
            {
                sendJointPositionMessage(lastTrajectory);
                // Reset lastTrajectory to null after sending
                lastTrajectory = null;
                sendButton.interactable = false; // Disable the send button since there's no trajectory to send
            }
        });

        sendButton.interactable = false; // Initially disabled

        // Replay Button
        GameObject replayButtonObject = contentGameObject.GetNamedChild("Replay Button")
            .GetNamedChild("Text Poke Button");
        replayButton = replayButtonObject.GetComponent<Button>();

        replayButton.onClick.AddListener(() =>
        {
            if (lastTrajectory != null)
            {
                StartCoroutine(playTrajectory(lastTrajectory));
            }
        });

        // Record button functionality
        recordButton.onClick.AddListener(() =>
        {
            if (recordROS == true)
            {
                // Stop recording
                recordROS = false;
                recordButtonText.text = "Start Recording";
                saveJointPositionMessage();

                // Enable the send button since we have a trajectory to send
                sendButton.interactable = true;

                // Let the planner know that interaction is done ||||||||| as the sending logic is now on the send button, don't know if i should do this anymore
                sendInteractionMessage(false);
            }
            else
            {
                // Start recording
                recordROS = true;
                recordButtonText.text = "Stop Recording";
                sendButton.interactable = false; // Disable send button while recording
                resetJointPositionMessage(); // Reset any previous recording data
                lastTrajectory = null; // Clear previous trajectory

                // Let the planner know that interaction is starting
                sendInteractionMessage(true);
                mirrorButton.onClick.Invoke();

            }
        });

        recordButton.interactable = true;

        // Dropdown and slider
        TMP_Dropdown dropdown = contentGameObject.GetNamedChild("List Item Dropdown")
            .GetNamedChild("Dropdown")
            .GetComponent<TMP_Dropdown>();
        Slider slider = contentGameObject.GetNamedChild("List Item Slider")
            .GetNamedChild("MinMax Slider")
            .GetComponent<Slider>();
        TextMeshProUGUI sliderText = slider.gameObject
            .GetNamedChild("Value Text")
            .GetComponent<TextMeshProUGUI>();

        // Populate the dropdown with the joint names
        dropdown.AddOptions(jointNames);

        RobotManager robotManager = robotModels[0].GetComponent<RobotManager>();
        int dropdownIndex = 0;
        slider.value = robotManager.GetJointAngle(dropdownIndex);
        sliderText.text = (-slider.value).ToString();
        slider.minValue = jointLimits[dropdownIndex].Item1;
        slider.maxValue = jointLimits[dropdownIndex].Item2;

        dropdown.onValueChanged.AddListener(delegate
        {
            dropdownIndex = dropdown.value;

            slider.value = robotManager.GetJointAngle(dropdownIndex);
            slider.minValue = jointLimits[dropdownIndex].Item1;
            slider.maxValue = jointLimits[dropdownIndex].Item2;
        });

        slider.onValueChanged.AddListener(delegate
        {
            robotManager.SetJointAngle(dropdownIndex, slider.value);
            sliderText.text = (-slider.value).ToString();
        });

        InvokeRepeating("addJointPosition", 1.0f, recordInterval);

        // Load the query interface
        contentGameObject = menuUI.GetNamedChild("Spatial Panel Scroll")
            .GetNamedChild("Query Scroll View")
            .GetNamedChild("Viewport")
            .GetNamedChild("Content");

        startButtonObject = contentGameObject.GetNamedChild("Set Start Button")
            .GetNamedChild("Text Poke Button");
        Button startButton = startButtonObject.GetComponent<Button>();
        TextMeshProUGUI startButtonText = startButtonObject
            .GetNamedChild("Button Front")
            .GetNamedChild("Text (TMP) ")
            .GetComponent<TextMeshProUGUI>();

        startButton.onClick.AddListener(() =>
        {
            setStart();
            startButtonText.text = "Start is Set!";
        });

        goalButtonObject = contentGameObject.GetNamedChild("Set Goal Button")
            .GetNamedChild("Text Poke Button");
        Button goalButton = goalButtonObject.GetComponent<Button>();
        TextMeshProUGUI goalButtonText = goalButtonObject
            .GetNamedChild("Button Front")
            .GetNamedChild("Text (TMP) ")
            .GetComponent<TextMeshProUGUI>();

        goalButton.onClick.AddListener(() =>
        {
            setGoal();
            goalButtonText.text = "Goal is Set!";
        });

        queryButtonObject = contentGameObject.GetNamedChild("Send Query Button")
            .GetNamedChild("Text Poke Button");
        Button queryButton = queryButtonObject.GetComponent<Button>();
        TextMeshProUGUI queryButtonText = queryButtonObject
            .GetNamedChild("Button Front")
            .GetNamedChild("Text (TMP) ")
            .GetComponent<TextMeshProUGUI>();

        queryButton.onClick.AddListener(() =>
        {
            sendQueryMessage();
            queryButtonText.text = "Query Sent!";
        });

        // Disable the query button until both start and goal are set
        queryButton.interactable = false;

        // Load the mirror interface
        contentGameObject = menuUI.GetNamedChild("Spatial Panel Scroll")
            .GetNamedChild("Mirror Scroll View")
            .GetNamedChild("Viewport")
            .GetNamedChild("Content");

        // Mirror input button
        mirrorButtonObject = contentGameObject.GetNamedChild("Mirror Input Button")
            .GetNamedChild("Text Poke Button");
        mirrorButton = mirrorButtonObject.GetComponent<Button>();
        TextMeshProUGUI mirrorButtonText = mirrorButtonObject
            .GetNamedChild("Button Front")
            .GetNamedChild("Text (TMP) ")
            .GetComponent<TextMeshProUGUI>();

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
        publishStateButtonObject = contentGameObject.GetNamedChild("Publish State Button")
            .GetNamedChild("Text Poke Button");
        Button publishStateButton = publishStateButtonObject.GetComponent<Button>();
        TextMeshProUGUI publishStateButtonText = publishStateButtonObject
            .GetNamedChild("Button Front")
            .GetNamedChild("Text (TMP) ")
            .GetComponent<TextMeshProUGUI>();

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

        // Set Home and Go Home buttons
        setHomeButtonObject = contentGameObject.GetNamedChild("Set Home Button")
            .GetNamedChild("Text Poke Button");
        Button setHomeButton = setHomeButtonObject.GetComponent<Button>();
        TextMeshProUGUI setHomeButtonText = setHomeButtonObject
            .GetNamedChild("Button Front")
            .GetNamedChild("Text (TMP) ")
            .GetComponent<TextMeshProUGUI>();

        setHomeButton.onClick.AddListener(() =>
        {
            Debug.Log("setHomeButton.onClick");
            robotManager.SetHomePosition();
        });

        goHomeButtonObject = contentGameObject.GetNamedChild("Reset to Home Button")
            .GetNamedChild("Text Poke Button");
        Button goHomeButton = goHomeButtonObject.GetComponent<Button>();
        TextMeshProUGUI goHomeButtonText = goHomeButtonObject
            .GetNamedChild("Button Front")
            .GetNamedChild("Text (TMP) ")
            .GetComponent<TextMeshProUGUI>();

        goHomeButton.onClick.AddListener(() =>
        {
            Debug.Log("goHomeButton.onClick");
            robotManager.ResetHomePosition();
        });
    }

    void addJointPosition()
    {
        if (recordROS)
        {
            RobotManager robotManager = robotModels[0].GetComponent<RobotManager>();

            if (recordStartTime == 0f)
            {
                recordStartTime = Time.time;
            }

            float[] jointPositions = robotManager.GetJointAngles();
            jointTorques = new List<double>();
            for (int i = 0; i < jointPositions.Length; i++) 
            {
                // Get the torque from the knob
                float torque = CalculateTorque(i, jointPositions[i] * Mathf.Deg2Rad);
                jointTorques.Add(torque);
            }

            // Calculate the precise time from start
            float timeFromStart = Time.time - recordStartTime;
            int secs = (int)Math.Floor(timeFromStart);
            uint nsecs = (uint)((timeFromStart - secs) * 1e9);

            JointTrajectoryPointMsg jointTrajectoryPoint = new JointTrajectoryPointMsg
            {
                positions = Array.ConvertAll(jointPositions, item => (double)item),
                effort = jointTorques.ToArray(),
                time_from_start = new DurationMsg(secs, nsecs),
            };
            jointTrajectoryPoints.Add(jointTrajectoryPoint);
        }
    }

    float CalculateTorque(int i, float currentAngle)
    {
        float previousAngle = previousAngles[i];
        float deltaAngle = Mathf.DeltaAngle(previousAngle, currentAngle);
        float angularVelocity = deltaAngle / Time.deltaTime;
        float torque = momentsOfInertia[i] * angularVelocity;
        previousAngles[i] = currentAngle;
        return torque;
    }
    
    void saveJointPositionMessage()
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

        lastTrajectory = jointTrajectory;
    }

    void sendJointPositionMessage(JointTrajectoryMsg jointTrajectory)
    {
        ros.Publish(trajTopicName, jointTrajectory);
        trajectoryLog.Add(jointTrajectory);

        // Clear the jointTrajectoryPoints list
        resetJointPositionMessage();

        // Let the planner know that interaction is done
        sendInteractionMessage(false);
    }

    void resetJointPositionMessage()
    {
        jointTrajectoryPoints.Clear();
        recordStartTime = 0;
    }

    IEnumerator playTrajectory(JointTrajectoryMsg trajectory) {
        JointTrajectoryPointMsg[] points = trajectory.points;
        double prevTime = durationToDouble(points[0].time_from_start);

 
        double[] prevPos = new double[points[0].positions.Length];
        for (int i = 0; i < prevPos.Length; i++) {
            prevPos[i] = -1 * (points[0].positions[i] * Mathf.Rad2Deg);
        }

        for (int i = 1; i < points.Length; i++) {
            double[] positions = points[i].positions;

  
            double[] modifiedPositions = new double[positions.Length];
            for (int j = 0; j < positions.Length; j++) {
                modifiedPositions[j] = -1 * (positions[j] * Mathf.Rad2Deg);
            }

            double currTime = durationToDouble(points[i].time_from_start);
            double movingTime = currTime - prevTime;

            if (positions.Length != jointNames.Count) {
                Debug.LogError("Positions array length does not match knobs count.");
                yield break;
            }

            yield return StartCoroutine(MoveKnobsOverTime(prevPos, modifiedPositions, movingTime));

            prevPos = modifiedPositions;
            prevTime = currTime;
        }
    }

    IEnumerator MoveKnobsOverTime(double[] startPositions, double[] endPositions, double duration) {
        float elapsedTime = 0f;

        if (duration <= 0f) duration = 0.000001f; 

        RobotManager robotManager = robotModels[0].GetComponent<RobotManager>();
        while (elapsedTime < duration) {
            elapsedTime += Time.deltaTime;
            float t = Mathf.Clamp01((float)(elapsedTime / duration)); // scaling to 0-1 range

            for (int j = 0; j < jointNames.Count; j++) {
                float newPos = Mathf.Lerp((float)startPositions[j], (float)endPositions[j], t);
                robotManager.SetJointAngle(j, newPos);
            }
            yield return null;
        }
    }

    double durationToDouble(DurationMsg duration)
    {
        return duration.sec + (duration.nanosec * 0.000000001);
    }


    void resetTrajectoryLog()
    {
        trajectoryLog.Clear();
    }

    private void SimulateRecordButtonClick(InputAction.CallbackContext context)
    {

        recordButton.onClick.Invoke();
    }
    private void SimulateReplayButtonClick(InputAction.CallbackContext context)
    {

        replayButton.onClick.Invoke();

    }
    private void SimulateMirrorButtonClick(InputAction.CallbackContext context)
    {

        mirrorButton.onClick.Invoke();

    }
}