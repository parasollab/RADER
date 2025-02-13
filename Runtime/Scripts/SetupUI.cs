using System;
using System.Collections.Generic;
using TMPro;
using Unity.XR.CoreUtils;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Trajectory;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using System.Collections;

public class SetupUI : MonoBehaviour
{
    public GameObject menuUI;
    public List<GameObject> robotModels;
    public string trajTopicName = "/joint_trajectory";
    public string inputStateTopicName = "/joint_states";
    public string outputStateTopicName = "/virtual_joint_state";
    public string interactionTopicName = "/interaction";
    public float recordInterval = 0.1f;
    public float publishStateInterval = 0.2f;

    // A simple class to store per-robot data.
    public class RobotInfo {
        public GameObject robotObject;
        public RobotManager manager;
        public List<string> jointNames;
        public List<Tuple<float, float>> jointLimits;
        // For calculating torque per joint.
        public Dictionary<int, float> previousAngles = new Dictionary<int, float>();
        public Dictionary<int, float> momentsOfInertia = new Dictionary<int, float>();
    }

    // List of robot data.
    private List<RobotInfo> robots = new List<RobotInfo>();
    // selectedRobotIndex: -1 means “All Robots”, otherwise index into robots list.
    private int selectedRobotIndex = 0;

    private XRIDefaultInputActions inputActions;
    private ROSConnection ros;
    private bool recordROS = false;
    // When recording, we store a list of trajectory points per robot (keyed by robot index).
    private Dictionary<int, List<JointTrajectoryPointMsg>> jointTrajectoryPointsDict = new Dictionary<int, List<JointTrajectoryPointMsg>>();
    private Dictionary<int, JointTrajectoryMsg> lastTrajectories = new Dictionary<int, JointTrajectoryMsg>();
    private Dictionary<int, float> recordStartTimes = new Dictionary<int, float>();

    private bool mirrorInputState = false;
    private bool publishState = true;
    private List<JointTrajectoryMsg> trajectoryLog = new List<JointTrajectoryMsg>();

    private GameObject recordButtonObject;
    private GameObject mirrorButtonObject;
    private GameObject publishStateButtonObject;
    private GameObject setHomeButtonObject;
    private GameObject goHomeButtonObject;

    private Button recordButton;
    private Button replayButton;
    private Button sendButton;
    private Button mirrorButton;

    // UI elements for joint selection.
    private TMP_Dropdown jointDropdown;
    private Slider jointSlider;
    private TextMeshProUGUI jointSliderText;

    void Start()
    {
        if (ros == null) ros = ROSConnection.GetOrCreateInstance();
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

        // Build our robot data list from robotModels.
        foreach (GameObject robot in robotModels)
        {
            RobotManager manager = robot.GetComponent<RobotManager>();
            RobotInfo info = new RobotInfo();
            info.robotObject = robot;
            info.manager = manager;
            info.jointNames = new List<string>(manager.GetJointNames());
            info.jointLimits = new List<Tuple<float, float>>(manager.GetJointLimits());
            // Initialize previous angles and moments (using an example inertia value).
            float[] jointAngles = manager.GetJointAngles();
            for (int i = 0; i < jointAngles.Length; i++)
            {
                info.previousAngles[i] = jointAngles[i];
                info.momentsOfInertia[i] = 0.0005f;
            }
            robots.Add(info);
        }
        // Default to the first robot (i.e. not “All Robots”).
        selectedRobotIndex = 0;

        LoadUI();
    }

    // Helper to compute torque for a given robot’s joint.
    float CalculateTorqueForRobot(RobotInfo robot, int i, float currentAngle)
    {
        float previousAngle = robot.previousAngles[i];
        float deltaAngle = Mathf.DeltaAngle(previousAngle, currentAngle);
        float angularVelocity = deltaAngle / Time.deltaTime;
        float torque = robot.momentsOfInertia[i] * angularVelocity;
        robot.previousAngles[i] = currentAngle;
        return torque;
    }

    public void startMirroring()
    {
        mirrorInputState = true;
        TextMeshProUGUI mirrorButtonText = mirrorButtonObject.GetNamedChild("Button Front").GetNamedChild("Text (TMP) ").GetComponent<TextMeshProUGUI>();
        mirrorButtonText.text = "Stop Mirroring";
        sendInteractionMessage(false);
    }

    public void stopMirroring()
    {
        mirrorInputState = false;
        TextMeshProUGUI mirrorButtonText = mirrorButtonObject.GetNamedChild("Button Front").GetNamedChild("Text (TMP) ").GetComponent<TextMeshProUGUI>();
        mirrorButtonText.text = "Start Mirroring";
        sendInteractionMessage(true);
    }

    // Mirror callback now directs the incoming joint state either to the selected robot or to all.
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

            if (selectedRobotIndex == -1)
            {
                foreach (var robot in robots)
                {
                    robot.manager.SetJointAngles(jointAngles);
                }
            }
            else
            {
                robots[selectedRobotIndex].manager.SetJointAngles(jointAngles);
            }
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

    // Publish state for either the selected robot or all robots.
    void PublishState()
    {
        if (publishState)
        {
            if (selectedRobotIndex == -1)
            {
                foreach (var robot in robots)
                {
                    float[] jointPositions = robot.manager.GetJointAngles();
                    List<double> jointTorques = new List<double>();
                    for (int i = 0; i < jointPositions.Length; i++)
                    {
                        float torque = CalculateTorqueForRobot(robot, i, jointPositions[i] * Mathf.Deg2Rad);
                        jointTorques.Add(torque);
                    }
                    JointStateMsg jointState = new JointStateMsg
                    {
                        name = robot.jointNames.ToArray(),
                        position = Array.ConvertAll(jointPositions, item => (double)item),
                        effort = jointTorques.ToArray()
                    };
                    ros.Publish(outputStateTopicName, jointState);
                }
            }
            else
            {
                var robot = robots[selectedRobotIndex];
                float[] jointPositions = robot.manager.GetJointAngles();
                List<double> jointTorques = new List<double>();
                for (int i = 0; i < jointPositions.Length; i++)
                {
                    float torque = CalculateTorqueForRobot(robot, i, jointPositions[i] * Mathf.Deg2Rad);
                    jointTorques.Add(torque);
                }
                JointStateMsg jointState = new JointStateMsg
                {
                    name = robot.jointNames.ToArray(),
                    position = Array.ConvertAll(jointPositions, item => (double)item),
                    effort = jointTorques.ToArray()
                };
                ros.Publish(outputStateTopicName, jointState);
            }
        }
    }

    // LoadUI now sets up the new robot-selection dropdown (located in the Header Interactable)
    // as well as reconfigures the joint dropdown/slider to update based on the selected robot.
    void LoadUI()
    {
        if (menuUI == null)
        {
            Debug.LogError("error loading UI");
        }
        menuUI = Instantiate(menuUI, transform);

        // Set up the robot selection dropdown.
        GameObject headerInteractable = menuUI.GetNamedChild("Spatial Panel Scroll")
            .GetNamedChild("Header Interactable");
        TMP_Dropdown robotDropdown = headerInteractable.GetNamedChild("Robot Dropdown")
            .GetNamedChild("Dropdown")
            .GetComponent<TMP_Dropdown>();

        // Populate options: first option is "All Robots", then one for each robot.
        List<string> robotOptions = new List<string>();
        robotOptions.Add("All Robots");
        foreach (var robot in robots)
        {
            robotOptions.Add(robot.robotObject.name);
        }
        robotDropdown.ClearOptions();
        robotDropdown.AddOptions(robotOptions);
        // Default: select the first robot (dropdown value 1) so that joint controls are enabled.
        robotDropdown.value = 1;
        selectedRobotIndex = 0;

        // Set up joint selection dropdown and slider (from the Robot Scroll View).
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

        sendButton.onClick.AddListener(() =>
        {
            if (selectedRobotIndex == -1)
            {
                foreach (var traj in lastTrajectories.Values)
                {
                    sendJointPositionMessage(traj);
                }
                lastTrajectories.Clear();
                sendButton.interactable = false;
            }
            else
            {
                if (lastTrajectories.ContainsKey(selectedRobotIndex))
                {
                    sendJointPositionMessage(lastTrajectories[selectedRobotIndex]);
                    lastTrajectories.Remove(selectedRobotIndex);
                    sendButton.interactable = false;
                }
            }
        });
        sendButton.interactable = false;

        // Replay button
        GameObject replayButtonObject = contentGameObject.GetNamedChild("Replay Button")
            .GetNamedChild("Text Poke Button");
        replayButton = replayButtonObject.GetComponent<Button>();

        replayButton.onClick.AddListener(() =>
        {
            if (selectedRobotIndex == -1)
            {
                foreach (var traj in lastTrajectories.Values)
                {
                    StartCoroutine(playTrajectory(traj));
                }
            }
            else
            {
                if (lastTrajectories.ContainsKey(selectedRobotIndex))
                {
                    StartCoroutine(playTrajectory(lastTrajectories[selectedRobotIndex]));
                }
            }
        });

        // Record button functionality.
        recordButton.onClick.AddListener(() =>
        {
            if (recordROS == true)
            {
                // Stop recording and package the trajectory for each robot.
                recordROS = false;
                recordButtonText.text = "Start Recording";
                if (selectedRobotIndex == -1)
                {
                    for (int i = 0; i < robots.Count; i++)
                    {
                        JointTrajectoryMsg trajectory = new JointTrajectoryMsg();
                        HeaderMsg header = new HeaderMsg
                        {
                            frame_id = robots[i].robotObject.name,
                            stamp = new TimeMsg
                            {
                                sec = (int)Time.time,
                                nanosec = (uint)((Time.time - (int)Time.time) * 1e9)
                            }
                        };
                        trajectory.header = header;
                        trajectory.joint_names = robots[i].jointNames.ToArray();
                        trajectory.points = jointTrajectoryPointsDict.ContainsKey(i) ? jointTrajectoryPointsDict[i].ToArray() : new JointTrajectoryPointMsg[0];
                        lastTrajectories[i] = trajectory;
                    }
                    sendButton.interactable = true;
                }
                else
                {
                    var robot = robots[selectedRobotIndex];
                    JointTrajectoryMsg trajectory = new JointTrajectoryMsg();
                    HeaderMsg header = new HeaderMsg
                    {
                        frame_id = robot.robotObject.name,
                        stamp = new TimeMsg
                        {
                            sec = (int)Time.time,
                            nanosec = (uint)((Time.time - (int)Time.time) * 1e9)
                        }
                    };
                    trajectory.header = header;
                    trajectory.joint_names = robot.jointNames.ToArray();
                    trajectory.points = jointTrajectoryPointsDict.ContainsKey(selectedRobotIndex) ? jointTrajectoryPointsDict[selectedRobotIndex].ToArray() : new JointTrajectoryPointMsg[0];
                    lastTrajectories[selectedRobotIndex] = trajectory;
                    sendButton.interactable = true;
                }
                sendInteractionMessage(false);
            }
            else
            {
                // Start recording.
                recordROS = true;
                recordButtonText.text = "Stop Recording";
                sendButton.interactable = false;
                if (selectedRobotIndex == -1)
                {
                    for (int i = 0; i < robots.Count; i++)
                    {
                        jointTrajectoryPointsDict[i] = new List<JointTrajectoryPointMsg>();
                        recordStartTimes[i] = Time.time;
                    }
                }
                else
                {
                    jointTrajectoryPointsDict[selectedRobotIndex] = new List<JointTrajectoryPointMsg>();
                    recordStartTimes[selectedRobotIndex] = Time.time;
                    if(lastTrajectories.ContainsKey(selectedRobotIndex))
                        lastTrajectories.Remove(selectedRobotIndex);
                }
                sendInteractionMessage(true);
                mirrorButton.onClick.Invoke();
            }
        });
        recordButton.interactable = true;

        // Joint selection dropdown and slider.
        jointDropdown = contentGameObject.GetNamedChild("List Item Dropdown")
            .GetNamedChild("Dropdown")
            .GetComponent<TMP_Dropdown>();
        jointSlider = contentGameObject.GetNamedChild("List Item Slider")
            .GetNamedChild("MinMax Slider")
            .GetComponent<Slider>();
        jointSliderText = jointSlider.gameObject.GetNamedChild("Value Text")
            .GetComponent<TextMeshProUGUI>();

        // Initialize joint dropdown based on the currently selected robot.
        if (selectedRobotIndex != -1)
        {
            jointDropdown.ClearOptions();
            jointDropdown.AddOptions(robots[selectedRobotIndex].jointNames);
            int jointIndex = jointDropdown.value;
            float angle = robots[selectedRobotIndex].manager.GetJointAngle(jointIndex);
            jointSlider.value = angle;
            jointSlider.minValue = robots[selectedRobotIndex].jointLimits[jointIndex].Item1;
            jointSlider.maxValue = robots[selectedRobotIndex].jointLimits[jointIndex].Item2;
        }
        else
        {
            // When "All Robots" is selected, disable joint controls.
            jointDropdown.interactable = false;
            jointSlider.interactable = false;
        }

        jointDropdown.onValueChanged.AddListener(delegate {
            int jointIndex = jointDropdown.value;
            if (selectedRobotIndex != -1)
            {
                float angle = robots[selectedRobotIndex].manager.GetJointAngle(jointIndex);
                jointSlider.value = angle;
                jointSlider.minValue = robots[selectedRobotIndex].jointLimits[jointIndex].Item1;
                jointSlider.maxValue = robots[selectedRobotIndex].jointLimits[jointIndex].Item2;
            }
        });

        jointSlider.onValueChanged.AddListener(delegate {
            int jointIndex = jointDropdown.value;
            if (selectedRobotIndex != -1)
            {
                robots[selectedRobotIndex].manager.SetJointAngle(jointIndex, jointSlider.value);
                jointSliderText.text = (-jointSlider.value).ToString();
            }
        });

        // When the user changes the robot selection...
        robotDropdown.onValueChanged.AddListener(delegate {
            if (robotDropdown.value == 0)
            {
                selectedRobotIndex = -1;
                jointDropdown.interactable = false;
                jointSlider.interactable = false;
            }
            else
            {
                selectedRobotIndex = robotDropdown.value - 1;
                jointDropdown.interactable = true;
                jointSlider.interactable = true;
                // Refresh joint dropdown options.
                jointDropdown.ClearOptions();
                jointDropdown.AddOptions(robots[selectedRobotIndex].jointNames);
                int jointIndex = jointDropdown.value;
                float angle = robots[selectedRobotIndex].manager.GetJointAngle(jointIndex);
                jointSlider.value = angle;
                jointSlider.minValue = robots[selectedRobotIndex].jointLimits[jointIndex].Item1;
                jointSlider.maxValue = robots[selectedRobotIndex].jointLimits[jointIndex].Item2;
            }
        });

        // Set up periodic recording and state publishing.
        InvokeRepeating("addJointPosition", 1.0f, recordInterval);
        InvokeRepeating("PublishState", 1.0f, publishStateInterval);

        // Mirror interface.
        contentGameObject = menuUI.GetNamedChild("Spatial Panel Scroll")
            .GetNamedChild("Mirror Scroll View")
            .GetNamedChild("Viewport")
            .GetNamedChild("Content");

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

        // Set Home and Go Home buttons.
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
            if (selectedRobotIndex == -1)
            {
                foreach (var robot in robots)
                {
                    robot.manager.SetHomePosition();
                }
            }
            else
            {
                robots[selectedRobotIndex].manager.SetHomePosition();
            }
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
            if (selectedRobotIndex == -1)
            {
                foreach (var robot in robots)
                {
                    robot.manager.ResetHomePosition();
                }
            }
            else
            {
                robots[selectedRobotIndex].manager.ResetHomePosition();
            }
        });
    }

    // Records joint positions over time for the selected robot or for all robots.
    void addJointPosition()
    {
        if (recordROS)
        {
            if (selectedRobotIndex == -1)
            {
                for (int i = 0; i < robots.Count; i++)
                {
                    if (!recordStartTimes.ContainsKey(i))
                        recordStartTimes[i] = Time.time;
                    if (!jointTrajectoryPointsDict.ContainsKey(i))
                        jointTrajectoryPointsDict[i] = new List<JointTrajectoryPointMsg>();
                    float[] jointPositions = robots[i].manager.GetJointAngles();
                    List<double> robotTorques = new List<double>();
                    for (int j = 0; j < jointPositions.Length; j++)
                    {
                        float torque = CalculateTorqueForRobot(robots[i], j, jointPositions[j] * Mathf.Deg2Rad);
                        robotTorques.Add(torque);
                    }
                    float timeFromStart = Time.time - recordStartTimes[i];
                    int secs = (int)Math.Floor(timeFromStart);
                    uint nsecs = (uint)((timeFromStart - secs) * 1e9);
                    JointTrajectoryPointMsg point = new JointTrajectoryPointMsg
                    {
                        positions = Array.ConvertAll(jointPositions, item => (double)item),
                        effort = robotTorques.ToArray(),
                        time_from_start = new DurationMsg(secs, nsecs),
                    };
                    jointTrajectoryPointsDict[i].Add(point);
                }
            }
            else
            {
                int i = selectedRobotIndex;
                if (!recordStartTimes.ContainsKey(i))
                    recordStartTimes[i] = Time.time;
                if (!jointTrajectoryPointsDict.ContainsKey(i))
                    jointTrajectoryPointsDict[i] = new List<JointTrajectoryPointMsg>();
                float[] jointPositions = robots[i].manager.GetJointAngles();
                List<double> robotTorques = new List<double>();
                for (int j = 0; j < jointPositions.Length; j++)
                {
                    float torque = CalculateTorqueForRobot(robots[i], j, jointPositions[j] * Mathf.Deg2Rad);
                    robotTorques.Add(torque);
                }
                float timeFromStart = Time.time - recordStartTimes[i];
                int secs = (int)Math.Floor(timeFromStart);
                uint nsecs = (uint)((timeFromStart - secs) * 1e9);
                JointTrajectoryPointMsg point = new JointTrajectoryPointMsg
                {
                    positions = Array.ConvertAll(jointPositions, item => (double)item),
                    effort = robotTorques.ToArray(),
                    time_from_start = new DurationMsg(secs, nsecs),
                };
                jointTrajectoryPointsDict[i].Add(point);
            }
        }
    }

    void sendJointPositionMessage(JointTrajectoryMsg jointTrajectory)
    {
        ros.Publish(trajTopicName, jointTrajectory);
        trajectoryLog.Add(jointTrajectory);
        sendInteractionMessage(false);
    }

    // Replay coroutine (unchanged aside from using the selected robot’s joint names when needed).
    IEnumerator playTrajectory(JointTrajectoryMsg trajectory)
    {
        JointTrajectoryPointMsg[] points = trajectory.points;
        double prevTime = durationToDouble(points[0].time_from_start);
        double[] prevPos = new double[points[0].positions.Length];
        for (int i = 0; i < prevPos.Length; i++)
            prevPos[i] = -1 * (points[0].positions[i] * Mathf.Rad2Deg);
        for (int i = 1; i < points.Length; i++)
        {
            double[] positions = points[i].positions;
            double[] modifiedPositions = new double[positions.Length];
            for (int j = 0; j < positions.Length; j++)
                modifiedPositions[j] = -1 * (positions[j] * Mathf.Rad2Deg);
            double currTime = durationToDouble(points[i].time_from_start);
            double movingTime = currTime - prevTime;
            if (positions.Length != (selectedRobotIndex == -1 ? robots[0].jointNames.Count : robots[selectedRobotIndex].jointNames.Count))
            {
                Debug.LogError("Positions array length does not match knobs count.");
                yield break;
            }
            yield return StartCoroutine(MoveKnobsOverTime(prevPos, modifiedPositions, movingTime));
            prevPos = modifiedPositions;
            prevTime = currTime;
        }
    }

    IEnumerator MoveKnobsOverTime(double[] startPositions, double[] endPositions, double duration)
    {
        float elapsedTime = 0f;
        if (duration <= 0f) duration = 0.000001f;
        if (selectedRobotIndex == -1)
        {
            // Update all robots.
            while (elapsedTime < duration)
            {
                elapsedTime += Time.deltaTime;
                float t = Mathf.Clamp01(elapsedTime / (float)duration);
                for (int j = 0; j < robots[0].jointNames.Count; j++)
                {
                    float newPos = Mathf.Lerp((float)startPositions[j], (float)endPositions[j], t);
                    foreach (var robot in robots)
                    {
                        robot.manager.SetJointAngle(j, newPos);
                    }
                }
                yield return null;
            }
        }
        else
        {
            RobotManager robotManager = robots[selectedRobotIndex].manager;
            while (elapsedTime < duration)
            {
                elapsedTime += Time.deltaTime;
                float t = Mathf.Clamp01(elapsedTime / (float)duration);
                for (int j = 0; j < robots[selectedRobotIndex].jointNames.Count; j++)
                {
                    float newPos = Mathf.Lerp((float)startPositions[j], (float)endPositions[j], t);
                    robotManager.SetJointAngle(j, newPos);
                }
                yield return null;
            }
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
