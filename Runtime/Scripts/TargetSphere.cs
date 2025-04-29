using System.Collections;
using System.Collections.Generic;
using System.Data.Common;
using RosMessageTypes.Sensor;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
public class TargetSphere : MonoBehaviour
{
    public GameObject target;
    public GameObject robot;
    private bool m_active = false;
    private RobotManager robotManager;
    private Vector3 targetPos;
    private Vector3 oldPos;
    private bool canMove = true;
    private Coroutine activeTimer;
    // Start is called before the first frame update
    void Start()
    {
        robotManager = robot.GetComponent<RobotManager>();
        if (robotManager == null) {
            Debug.LogError("RobotManager not found");
            return;
        }
        if (target == null) {
            Debug.LogError("Target not found");
            return;
        }
        // xr events
        UnityEngine.XR.Interaction.Toolkit.Interactables.XRGrabInteractable grabInteractable = target.GetComponent<UnityEngine.XR.Interaction.Toolkit.Interactables.XRGrabInteractable>();
        if (grabInteractable == null) {
            Debug.LogError("XRGrabInteractable not found");
            return;
        }
        // On select enter set CCDIK activ
        grabInteractable.selectEntered.AddListener((SelectEnterEventArgs interactor) => {
            Debug.Log("TargetSphere: Target selected");
            m_active = true;
        });
        grabInteractable.selectExited.AddListener((SelectExitEventArgs interactor) => {
            m_active = false;
        });
        // Set the target to the end effector
        Transform ee = robotManager.GetEEPose();
        targetPos = ee.position;
        target.transform.position = targetPos;
        oldPos = targetPos;
        // Move the target a known transformation
        // target.transform.position = new Vector3(-0.47659859f, -0.15102799f, 0.49082398f);
        // target.transform.position = new Vector3(-0.5f, 0.5f, 0.4f);
        // target.transform.rotation = new Quaternion(0.6922559f, 0.7213566f, -0.0004294f, -0.0206427f);
    }
    // private IEnumerator timer() {
    //     yield return new WaitForSeconds(0.01f);
    //     canMove = true;
    //     activeTimer = null;
    // }
    // Update is called once per frame
    void Update()
    {
        targetPos = target.transform.position;
        if (robotManager == null || target == null) {
            return;
        }
        if (!m_active) {
            Transform ee = robotManager.GetEEPose();
            targetPos = ee.position;
            target.transform.position = targetPos;
        } else {
            if (targetPos != oldPos){ // REVIEW; whenever the targetPos gets out of reach the arm starts shaking.
                robotManager.SetTargetEEPose(target.transform);
                oldPos = targetPos;
                // canMove = false;
                // if (activeTimer == null) {
                //     activeTimer = StartCoroutine(timer());
                // }
            }
        }
    }
}