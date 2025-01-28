using System;
using UnityEngine;
using UnityEngine.XR.Interaction.Toolkit;

public class CollisionHaptics : MonoBehaviour
{
    [SerializeField] private float maxIntensity = 1.0f;
    [SerializeField] private float scaleFactor = 5.0f; // Determines the sharpness of the exponential increase
    [SerializeField] private float minDistance = 0.3f; // Minimum distance to trigger haptics

    public XRBaseController leftController;
    public XRBaseController rightController;

    private void Start()
    {
        // InitializeControllers();
    }

    private void InitializeControllers()
    {
        GameObject leftControllerObject = GameObject.Find("Left Controller");
        if (leftControllerObject != null)
        {
            leftController = leftControllerObject.GetComponent<XRController>();
            if (leftController == null)
            {
                Debug.LogWarning("Left Controller found, but it does not have an XRController component.");
            }
        }
        else
        {
            Debug.LogWarning("Left Controller GameObject not found.");
        }

        GameObject rightControllerObject = GameObject.Find("Right Controller");
        if (rightControllerObject != null)
        {
            rightController = rightControllerObject.GetComponent<XRController>();
            if (rightController == null)
            {
                Debug.LogWarning("Right Controller found, but it does not have an XRController component.");
            }
        }
        else
        {
            Debug.LogWarning("Right Controller GameObject not found.");
        }
    }

    private void Update()
    {
        GameObject[] objects = GameObject.FindGameObjectsWithTag("object");
        if (objects.Length == 0) return;

        float closestDistance = float.MaxValue;

        foreach (var obj in objects)
        {
            float distance = CalculateClosestDistance(obj);
            if (distance < closestDistance)
            {
                closestDistance = distance;
            }
        }

        if (closestDistance > minDistance) return;

        float intensity = CalculateIntensity(closestDistance);
        TriggerHaptics(intensity);

        // Debug.Log($"Closest Distance: {closestDistance}, Intensity: {intensity}");
    }

    private float CalculateClosestDistance(GameObject obj)
    {
        float closestDistance = float.MaxValue;

        foreach (Transform robotPart in GetComponentsInChildren<Transform>())
        {
            float distance = Vector3.Distance(obj.transform.position, robotPart.position);
            if (distance < closestDistance)
            {
                closestDistance = distance;
            }
        }

        return closestDistance;
    }

    private float CalculateIntensity(float distance)
    {
        if (distance <= 0.1f) return maxIntensity; // Cap intensity if very close
        return maxIntensity * Mathf.Exp(-scaleFactor * distance);
    }

    private void TriggerHaptics(float intensity)
    {
        if (intensity < 0.01f) return; // Ignore very low intensities

        if (leftController != null)
        {
            leftController.SendHapticImpulse(intensity, 0.1f);
        }

        if (rightController != null)
        {
            rightController.SendHapticImpulse(intensity, 0.1f);
        }
    }
}
