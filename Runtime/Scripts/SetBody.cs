using Unity.VRTemplate;
using UnityEngine;

public class SetBody : MonoBehaviour
{
    public ArticulationBody body;


    public void Set(float value){
        // set articulation body xDrive target to be the knob value
        var drive = body.xDrive;
        drive.target = value;
        body.xDrive = drive;
    }
}
