using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.XR.ARFoundation;

[RequireComponent(typeof(ARPlaneManager))]
public class SceneController : MonoBehaviour
{
    [SerializeField]
    private InputActionReference _togglePlanesAction;

    private ARPlaneManager _arPlaneManager;
    private bool _isVisible = true;
    private int _numPlanesAddedOccurred = 0;

    // Start is called before the first frame update
    void Start()
    {
        _arPlaneManager = GetComponent<ARPlaneManager>();

        if (_arPlaneManager == null)
        {
            Debug.LogError("ARPlaneManager component not found");
        }

        _togglePlanesAction.action.performed += OnTogglePlanesAction;
        _arPlaneManager.trackablesChanged.AddListener(OnTrackablesChanged);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnTogglePlanesAction(InputAction.CallbackContext context)
    {
        _isVisible = !_isVisible;
        float fillAlpha = _isVisible ? 0.33f : 0.0f;
        float lineAlpha = _isVisible ? 1.0f : 0.0f;

        foreach (var plane in _arPlaneManager.trackables)
        {
            SetPlaneAlpha(plane, fillAlpha, lineAlpha);
        }
    }

    private void SetPlaneAlpha(ARPlane plane, float fillAlpha, float lineAlpha)
    {
        var meshRenderer = plane.GetComponent<MeshRenderer>();
        var lineRenderer = plane.GetComponent<LineRenderer>();

        if (meshRenderer != null)
        {
            var color = meshRenderer.material.color;
            color.a = fillAlpha;
            meshRenderer.material.color = color;
        }

        if (lineRenderer != null)
        {
            Color startColor = lineRenderer.startColor;
            Color endColor = lineRenderer.endColor;

            startColor.a = lineAlpha;
            endColor.a = lineAlpha;

            lineRenderer.startColor = startColor;
            lineRenderer.endColor = endColor;
        }
    }

    public void OnTrackablesChanged(ARTrackablesChangedEventArgs<ARPlane> changes)
    {
        foreach (var plane in changes.added)
        {
            _numPlanesAddedOccurred++;
            Debug.Log($"Plane added. Total planes added: {_numPlanesAddedOccurred}");
        }
    }

    void OnDestroy()
    {
        _togglePlanesAction.action.performed -= OnTogglePlanesAction;
        _arPlaneManager.trackablesChanged.RemoveListener(OnTrackablesChanged);
    }
}