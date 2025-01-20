using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public abstract class IKSolver : MonoBehaviour
{
    public abstract float[] InverseKinematics(Vector3 pos, Quaternion ori, float[] initialAngles);
}
