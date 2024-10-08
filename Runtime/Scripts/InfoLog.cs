using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class InfoLog : MonoBehaviour
{
    public ROSConnection ros;
    public string topicName = "/user_info";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>(topicName, PrintLog);
    }

    void PrintLog(StringMsg msg)
    {
        Debug.Log(msg.data);
    }
}
