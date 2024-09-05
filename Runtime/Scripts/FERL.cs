using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using Unity.XR.CoreUtils;
using UnityEngine.UI;

public class FERL : MonoBehaviour
{
    public ROSConnection ros;
    public string feedbackRequestTopic = "/feedback_request";
    public string satisfactionRequestTopic = "/req_satisfied";
    public string feedbackResponseTopic = "/feedback_response";
    public GameObject requestPopup;

    private Button resumeButton;
    private Button closeButton;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<BoolMsg>(feedbackRequestTopic, onRequestFeedback);
        ros.Subscribe<BoolMsg>(satisfactionRequestTopic, onRequestSatisfaction);
        ros.RegisterPublisher<BoolMsg>(feedbackResponseTopic);
        LoadPopupOptions();
    }

    void LoadPopupOptions() {
        if (requestPopup == null)
        {
            Debug.LogError("error loading UI");
        }
        requestPopup = Instantiate(requestPopup, transform);
        requestPopup.SetActive(false); // Disable the popup by default

        GameObject contentGameObject = requestPopup.GetNamedChild("Spatial Panel Scroll").GetNamedChild("Scroll View").GetNamedChild("Viewport").GetNamedChild("Content");
        GameObject resumeButtonObject = contentGameObject.GetNamedChild("Resume Button").GetNamedChild("Text Poke Button");
        resumeButton = resumeButtonObject.GetComponent<Button>();

        resumeButton.interactable = false; // Disable until we get a satisfaction request

        resumeButton.onClick.AddListener(() => {
            sendFeedbackResponse();
            requestPopup.SetActive(false);
        });

        GameObject closeButtonObject = contentGameObject.GetNamedChild("Close Button").GetNamedChild("Text Poke Button");
        closeButton = closeButtonObject.GetComponent<Button>();

        closeButton.onClick.AddListener(() => {
            requestPopup.SetActive(false);
        });
    }

    void onRequestFeedback(BoolMsg _request) {
        if (!_request.data) {
            return;
        }

        requestPopup.SetActive(true);
        resumeButton.interactable = false;
    }

    void onRequestSatisfaction(BoolMsg _request) {
        if (!_request.data) {
            return;
        }

        resumeButton.interactable = true;
    }

    void sendFeedbackResponse() {
        BoolMsg response = new BoolMsg
        {
            data = true
        };
        ros.Publish(feedbackResponseTopic, response);
    }
}
