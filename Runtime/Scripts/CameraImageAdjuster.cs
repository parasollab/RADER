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

using UnityEngine;
#if ERUPT_USE_META_XR && UNITY_ANDROID && !UNITY_VISIONOS
using Meta.XR;
#endif

public class CameraImageAduster : MonoBehaviour
{
#if ERUPT_USE_META_XR && UNITY_ANDROID && !UNITY_VISIONOS
    [SerializeField] private PassthroughCameraAccess _passthroughCameraAccess;
#else
    [SerializeField] private Camera _fallbackCamera;
#endif
    [SerializeField] private float _distanceFromCamera = 1.0f;
   
    void Awake()
    {
    }
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
#if ERUPT_USE_META_XR && UNITY_ANDROID && !UNITY_VISIONOS
        if (_passthroughCameraAccess == null || !_passthroughCameraAccess.IsPlaying)
            return;

        var pose = _passthroughCameraAccess.GetCameraPose();
        var intrinsics = _passthroughCameraAccess.Intrinsics;
        var image = _passthroughCameraAccess.CurrentResolution;
        var principalPoint = intrinsics.PrincipalPoint;
        var focalLength = intrinsics.FocalLength;
        var resolution = intrinsics.SensorResolution;
        transform.SetPositionAndRotation(pose.position + pose.rotation * Vector3.forward * _distanceFromCamera, pose.rotation);

        var scaleX = 2.0f * _distanceFromCamera * focalLength.x / resolution.x;
        var scaleY = scaleX * (image.y / (float)image.x);

        transform.localScale = new Vector3(scaleX, scaleY, 1.0f);
#else
        _fallbackCamera ??= Camera.main;
        if (_fallbackCamera == null)
            return;

        transform.SetPositionAndRotation(
            _fallbackCamera.transform.position + _fallbackCamera.transform.forward * _distanceFromCamera,
            _fallbackCamera.transform.rotation);

        float height = 2f * _distanceFromCamera * Mathf.Tan(_fallbackCamera.fieldOfView * Mathf.Deg2Rad * 0.5f);
        transform.localScale = new Vector3(height * _fallbackCamera.aspect, height, 1f);
#endif
    }
}
