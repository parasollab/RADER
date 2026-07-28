// MIT License

// Copyright (c) 2025 Takashi Yoshinaga

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
#if ERUPT_USE_META_XR && UNITY_ANDROID && !UNITY_VISIONOS
using Meta.XR;
#endif

public class ArUcoTrackingManager : MonoBehaviour
{
    /// <summary>
    /// Serializable class for mapping marker IDs to GameObjects in the Inspector.
    /// </summary>
    [Serializable]
    public class MarkerGameObjectPair
    {
        /// <summary>
        /// The unique ID of the AR marker to track.
        /// </summary>
        public int markerId;
        
        /// <summary>
        /// The GameObject to associate with this marker.
        /// </summary>
        public GameObject gameObject;
    }

    [Header("Passthrough Camera")]
    [SerializeField]
#if ERUPT_USE_META_XR && UNITY_ANDROID && !UNITY_VISIONOS
    private PassthroughCameraAccess m_passthroughCameraAccess;
#else
    private MonoBehaviour m_passthroughCameraAccess;
#endif

    [Header("Marker Tracking")]
    [SerializeField] private ArUcoMarkerTracking m_arucoMarkerTracking;
    [SerializeField, Tooltip("List of marker IDs mapped to their corresponding GameObjects")]
    private List<MarkerGameObjectPair> m_markerGameObjectPairs = new List<MarkerGameObjectPair>();
    [SerializeField] MeshRenderer m_debugRenderer;
    [SerializeField, Tooltip("Optional generic input action for toggling the debug recognition view.")]
    private InputActionReference m_toggleVisualizationAction;

    private Dictionary<int, GameObject> m_markerGameObjectDictionary = new Dictionary<int, GameObject>();
    
    private Texture2D m_resultTexture;

    private Transform m_cameraAnchor;

    
    private bool m_showRecogResult = false;

    private void OnEnable()
    {
        m_toggleVisualizationAction?.action?.Enable();
    }

    private void OnDisable()
    {
        m_toggleVisualizationAction?.action?.Disable();
    }

    /// <summary>
    /// Initializes the camera anchor, camera, and marker tracking system.
    /// </summary>
    private IEnumerator Start()
    {
#if !(ERUPT_USE_META_XR && UNITY_ANDROID && !UNITY_VISIONOS)
        Debug.LogWarning("[ArUcoTrackingManager] Raw passthrough camera access is only enabled for Quest builds with ERUPT_USE_META_XR. Marker tracking is disabled for this build.");
        yield break;
#else
    
        if(m_passthroughCameraAccess==null)
        {
            Debug.LogError("PassthroughCameraAccess reference is missing.");
            yield break;
        }

        // Create camera anchor dynamically
        CreateCameraAnchor();

        // Initialize camera
        yield return InitializeCamera();

            
        //======================================================================================
        // CORE SETUP: Initialize the marker tracking system with camera parameters
        // This configures the ArUco detection with proper camera calibration values
        // and prepares the marker-to-GameObject mapping dictionary
        //======================================================================================
        InitializeMarkerTracking();
        
        // Set initial visibility states
        if(m_debugRenderer!=null){
            m_debugRenderer.gameObject.SetActive(m_showRecogResult);
        }
        SetMarkerObjectsVisibility(!m_showRecogResult);
#endif
    }



    /// <summary>
    /// Creates a camera anchor GameObject dynamically at runtime.
    /// </summary>
    private void CreateCameraAnchor()
    {
        GameObject anchorObject = new GameObject("CameraAnchor");
        m_cameraAnchor = anchorObject.transform;
    }

    /// <summary>
    /// Initializes the camera with appropriate resolution and waits until ready.
    /// </summary>
    private IEnumerator InitializeCamera()
    {
#if ERUPT_USE_META_XR && UNITY_ANDROID && !UNITY_VISIONOS
        while (!m_passthroughCameraAccess.IsPlaying)
        {
            yield return null;
        }
        yield return null; // Wait one frame to ensure camera is fully initialized
#else
        yield break;
#endif
    }

    /// <summary>
    /// Updates camera poses, processes marker tracking, and handles input for toggling visualization mode.
    /// </summary>
    private void Update()
    {
#if ERUPT_USE_META_XR && UNITY_ANDROID && !UNITY_VISIONOS
        // Skip if camera or tracking system isn't ready
        if(m_passthroughCameraAccess==null || !m_passthroughCameraAccess.IsPlaying || !m_arucoMarkerTracking.IsReady)
            return;

        // Toggle between camera view and AR visualization on button press
        HandleVisualizationToggle();
        
        // Update tracking and visualization
        UpdateCameraPoses();
        
        //======================================================================================
        // CORE FUNCTIONALITY: Process marker detection and positioning of 3D objects
        // This is where ArUco markers are detected in the camera frame and 3D objects
        // are positioned in the scene according to marker positions
        //======================================================================================
        ProcessMarkerTracking();
#endif
    }

    /// <summary>
    /// Handles button input to toggle between recognition result display and AR marker objects.
    /// </summary>
    private void HandleVisualizationToggle()
    {
        if(m_debugRenderer==null)
            return;

        if (m_toggleVisualizationAction != null && m_toggleVisualizationAction.action != null &&
            m_toggleVisualizationAction.action.WasPressedThisFrame())
        {
            m_showRecogResult = !m_showRecogResult;
            m_debugRenderer.gameObject.SetActive(m_showRecogResult);
            SetMarkerObjectsVisibility(!m_showRecogResult);
        }
    }

    /// <summary>
    /// Performs marker detection and pose estimation.
    /// This is the core functionality that processes camera frames to detect markers
    /// and position virtual objects in 3D space.
    /// </summary>
    private void ProcessMarkerTracking()
    {
#if ERUPT_USE_META_XR && UNITY_ANDROID && !UNITY_VISIONOS
        // Step 1: Detect ArUco markers in the current camera frame
        m_arucoMarkerTracking.DetectMarker(m_passthroughCameraAccess.GetTexture(), m_resultTexture);
        
        // Step 2: Estimate the pose of markers and position 3D objects accordingly
        // This maps the 2D marker positions to 3D space using the camera parameters
        m_arucoMarkerTracking.EstimatePoseCanonicalMarker(m_markerGameObjectDictionary, m_cameraAnchor);
#endif
    }

    /// <summary>
    /// Toggles the visibility of all marker-associated GameObjects in the dictionary.
    /// </summary>
    /// <param name="isVisible">Whether the marker objects should be visible or not.</param>
    private void SetMarkerObjectsVisibility(bool isVisible)
    {
        // Toggle visibility for all GameObjects in the marker dictionary
        foreach (var markerObject in m_markerGameObjectDictionary.Values)
        {
            if (markerObject != null)
            {
                var rendererList = markerObject.GetComponentsInChildren<Renderer>(true);
                foreach (var meshRenderer in rendererList)
                {
                    meshRenderer.enabled = isVisible;
                }
            }
        }
    }

    /// <summary>
    /// Initializes the marker tracking system with camera parameters and builds the marker dictionary.
    /// This method configures the ArUco marker detection system with the correct camera parameters
    /// for accurate pose estimation.
    /// </summary>
    private void InitializeMarkerTracking()
    {
#if ERUPT_USE_META_XR && UNITY_ANDROID && !UNITY_VISIONOS
        // Step 1: Get camera intrinsic parameters
        // These intrinsic parameters are essential for accurate marker pose estimation
        var intrinsics = m_passthroughCameraAccess.Intrinsics;
        var cx = intrinsics.PrincipalPoint.x;  // Principal point X (optical center)
        var cy = intrinsics.PrincipalPoint.y;  // Principal point Y (optical center)
        var fx = intrinsics.FocalLength.x;     // Focal length X
        var fy = intrinsics.FocalLength.y;     // Focal length Y
        var width = intrinsics.SensorResolution.x;   // Image width
        var height = intrinsics.SensorResolution.y;  // Image height
        
        // Step 2: Scale parameters to match current camera resolution
        var currentResolution = m_passthroughCameraAccess.CurrentResolution;
        Debug.Log($"Camera Intrinsics - fx: {fx}, fy: {fy}, cx: {cx}, cy: {cy}, width: {width}, height: {height}");
        Debug.Log($"Current Camera Resolution - width: {currentResolution.x}, height: {currentResolution.y}");
        
        if (currentResolution.x != width || currentResolution.y != height)
        {
            float scaleX = (float)currentResolution.x / width;
            float scaleY = (float)currentResolution.y / height;
            fx *= scaleX;
            fy *= scaleY;
            cx *= scaleX;
            cy *= scaleY;
            width = currentResolution.x;
            height = currentResolution.y;
        }

        // Step 3: Initialize the ArUco tracking with camera parameters
        m_arucoMarkerTracking.Initialize(width, height, cx, cy, fx, fy);
        
        // Step 4: Build marker dictionary from serialized list
        // This maps marker IDs to the GameObjects that should be positioned at each marker
        BuildMarkerDictionary();
        
        // Step 5: Set up texture for visualization
        ConfigureResultTexture(width, height);
#endif
    }

    /// <summary>
    /// Builds the dictionary mapping marker IDs to GameObjects.
    /// </summary>
    private void BuildMarkerDictionary()
    {
        m_markerGameObjectDictionary.Clear();
        foreach (var pair in m_markerGameObjectPairs)
        {
            if (pair.gameObject != null)
            {
                m_markerGameObjectDictionary[pair.markerId] = pair.gameObject;
            }
        }
    }

    /// <summary>
    /// Configures the texture for displaying camera and tracking results.
    /// </summary>
    /// <param name="width">Width of the camera resolution</param>
    /// <param name="height">Height of the camera resolution</param>
    private void ConfigureResultTexture(int width, int height)
    {
        int divideNumber = m_arucoMarkerTracking.DivideNumber;
        m_resultTexture = new Texture2D(width/divideNumber, height/divideNumber, TextureFormat.RGB24, false);
        if (m_debugRenderer != null)
        {
            m_debugRenderer.material.mainTexture = m_resultTexture;
        }
    }

    /// <summary>
    /// Updates the camera anchor position and rotation based on the camera pose.
    /// </summary>
    private void UpdateCameraPoses()
    {
#if ERUPT_USE_META_XR && UNITY_ANDROID && !UNITY_VISIONOS
        // Update camera anchor position and rotation
        var cameraPose = m_passthroughCameraAccess.GetCameraPose();
        m_cameraAnchor.position = cameraPose.position;
        m_cameraAnchor.rotation = cameraPose.rotation;
#endif
    }
}
