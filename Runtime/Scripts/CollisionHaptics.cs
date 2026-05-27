using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;
using UnityEngine.XR.Interaction.Toolkit.Inputs;

public class CollisionHaptics : MonoBehaviour
{
    [SerializeField] private float maxIntensity = 1.0f;
    [SerializeField] private float minDistance = 0.3f; // Haptics start below this distance

    public XRBaseController leftController;
    public XRBaseController rightController;

    private void Start()
    {
        InitializeControllers();
    }

    private void InitializeControllers()
    {
        if (leftController == null)
        {
            GameObject leftControllerObject = GameObject.Find("Left Controller");
            if (leftControllerObject != null)
            {
                leftController = leftControllerObject.GetComponent<ActionBasedController>();
                if (leftController == null)
                    Debug.LogWarning("Left Controller found, but it does not have an ActionBasedController component.");
            }
            else
            {
                Debug.LogWarning("Left Controller GameObject not found. Assign it manually in the Inspector.");
            }
        }

        if (rightController == null)
        {
            GameObject rightControllerObject = GameObject.Find("Right Controller");
            if (rightControllerObject != null)
            {
                rightController = rightControllerObject.GetComponent<ActionBasedController>();
                if (rightController == null)
                    Debug.LogWarning("Right Controller found, but it does not have an ActionBasedController component.");
            }
            else
            {
                Debug.LogWarning("Right Controller GameObject not found. Assign it manually in the Inspector.");
            }
        }
    }

    private void Update()
    {
        GameObject[] objects = GameObject.FindGameObjectsWithTag("Selectable");
        if (objects.Length == 0) return;

        float closestDistance = float.MaxValue;

        foreach (var obj in objects)
        {
            float distance = CalculateClosestDistance(obj);
            if (distance < closestDistance)
                closestDistance = distance;
        }

        if (closestDistance >= minDistance) return;

        float intensity = CalculateIntensity(closestDistance);
        TriggerHaptics(intensity);
    }

    private float CalculateClosestDistance(GameObject obj)
    {
        float closestDistance = float.MaxValue;

        foreach (Transform robotPart in GetComponentsInChildren<Transform>())
        {
            float distance = Vector3.Distance(obj.transform.position, robotPart.position);
            if (distance < closestDistance)
                closestDistance = distance;
        }

        return closestDistance;
    }

    private float CalculateIntensity(float distance)
    {
        // Smooth quadratic ramp: 0 at minDistance, maxIntensity at contact
        float t = Mathf.Clamp01(1f - (distance / minDistance));
        return maxIntensity * t * t;
    }

    private void TriggerHaptics(float intensity)
    {
        if (intensity < 0.01f) return;

        if (leftController != null)
            leftController.SendHapticImpulse(intensity, 0.1f);

        if (rightController != null)
            rightController.SendHapticImpulse(intensity, 0.1f);
    }
}
