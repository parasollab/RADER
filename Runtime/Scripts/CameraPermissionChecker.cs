using UnityEngine;
using UnityEngine.Android;

/// <summary>
/// Checks and requests camera permissions for Quest 3
/// Attach this to a GameObject in your scene
/// </summary>
public class CameraPermissionChecker : MonoBehaviour
{
    private const string CAMERA_PERMISSION = "android.permission.CAMERA";
    private const string HEADSET_CAMERA_PERMISSION = "horizonos.permission.HEADSET_CAMERA";
    
    void Start()
    {
        CheckPermissions();
    }
    
    void CheckPermissions()
    {
        Debug.Log("=== Camera Permission Check ===");
        
        #if UNITY_ANDROID && !UNITY_EDITOR
        
        // Check CAMERA permission
        bool hasCameraPermission = Permission.HasUserAuthorizedPermission(CAMERA_PERMISSION);
        Debug.Log($"CAMERA permission: {(hasCameraPermission ? "GRANTED ✓" : "DENIED ✗")}");
        
        // Check HEADSET_CAMERA permission
        bool hasHeadsetPermission = Permission.HasUserAuthorizedPermission(HEADSET_CAMERA_PERMISSION);
        Debug.Log($"HEADSET_CAMERA permission: {(hasHeadsetPermission ? "GRANTED ✓" : "DENIED ✗")}");
        
        // Request if not granted
        if (!hasCameraPermission || !hasHeadsetPermission)
        {
            Debug.LogWarning("Requesting missing camera permissions...");
            
            if (!hasCameraPermission)
            {
                Permission.RequestUserPermission(CAMERA_PERMISSION);
            }
            
            if (!hasHeadsetPermission)
            {
                Permission.RequestUserPermission(HEADSET_CAMERA_PERMISSION);
            }
        }
        else
        {
            Debug.Log("All camera permissions granted!");
        }
        
        #else
        Debug.LogWarning("Permission check only works on Android device (Quest 3)");
        #endif
        
        Debug.Log("==============================");
    }
    
    // Check permissions again after a few seconds
    void Update()
    {
        if (Time.frameCount == 180) // After 3 seconds at 60fps
        {
            CheckPermissions();
        }
    }
}
