using UnityEngine;
using UnityEngine.Android;

/// <summary>
/// Checks and requests environment / plane (Scene API) permissions for Meta Quest 3
/// Attach this to a GameObject in your scene
/// </summary>
public class ScenePermissionChecker : MonoBehaviour
{
    // Required for planes, walls, floors, scene anchors
    private const string USE_SCENE_PERMISSION = "com.oculus.permission.USE_SCENE";

    // Sometimes implicitly required by system spatial mapping
    private const string FINE_LOCATION_PERMISSION = "android.permission.ACCESS_FINE_LOCATION";

    void Start()
    {
        CheckPermissions();
    }

    void CheckPermissions()
    {
        Debug.Log("=== Scene / Plane Permission Check ===");

#if UNITY_ANDROID && !UNITY_EDITOR

        bool hasScenePermission =
            Permission.HasUserAuthorizedPermission(USE_SCENE_PERMISSION);

        bool hasLocationPermission =
            Permission.HasUserAuthorizedPermission(FINE_LOCATION_PERMISSION);

        Debug.Log($"USE_SCENE permission: {(hasScenePermission ? "GRANTED ✓" : "DENIED ✗")}");
        Debug.Log($"FINE_LOCATION permission: {(hasLocationPermission ? "GRANTED ✓" : "DENIED ✗")}");

        if (!hasScenePermission || !hasLocationPermission)
        {
            Debug.LogWarning("Requesting missing scene permissions...");

            if (!hasScenePermission)
            {
                Permission.RequestUserPermission(USE_SCENE_PERMISSION);
            }

            if (!hasLocationPermission)
            {
                Permission.RequestUserPermission(FINE_LOCATION_PERMISSION);
            }
        }
        else
        {
            Debug.Log("All scene permissions granted!");
        }

#else
        Debug.LogWarning("Permission check only works on Android device (Quest 3)");
#endif

        Debug.Log("====================================");
    }

    void Update()
    {
        if (Time.frameCount == 180)
        {
            #if UNITY_ANDROID && !UNITY_EDITOR
            bool hasScene = Permission.HasUserAuthorizedPermission(USE_SCENE_PERMISSION);
            bool hasLoc   = Permission.HasUserAuthorizedPermission(FINE_LOCATION_PERMISSION);
            Debug.Log($"[ScenePermissionChecker] frame 180 — USE_SCENE:{hasScene} FINE_LOCATION:{hasLoc}");
            #endif
        }
    }
}
