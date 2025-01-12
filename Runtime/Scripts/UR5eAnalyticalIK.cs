using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// READ BEFORE WORKING ON THIS FILE:
// This file contains the analytical inverse kinematics solver for the UR5e robot.
// The solver is adapted from the ur_kinematics package in the ROS Industrial repository.
// The original code is written in C++ and can be found at:
// https://github.com/ros-industrial/universal_robot/tree/noetic-devel/ur_kinematics
// (we should include the license info from the original code when we finalize the implementation)

// The current version does not work as expected and needs to be fixed.
// Here are the issues:
// 1) Unity uses left-handed coordinate system, while the UR5e uses a right-handed coordinate system.
//   This means that the axes of the robot are different from the axes of Unity.
//   The solver needs to take this into account and convert the target position and orientation from Unity to the robot's coordinate system.
//   This file includes a forward kinematics function that also suffers from this issue (for use in testing the solver).
// 2) The solver does not handle the case when there is no solution for the given target pose.
//   In this case, the solver should find the closest reachable point to the target pose.
// 3) The base of the robot could be in a different position from the origin of the world in Unity.
//   The solver should take this into account and make the target pose relative to the robot's base.
// 4) The solver should return the joint angles in degrees, as Unity uses degrees for angles.
//   This might be already implemented, but it needs to be verified that eveything is in the units expected by respective components.
// 5) It is unclear whether this matches the desired end effector orientation.
//   If it doesn't, we should choose the solution that gets closest to the desired orientation.
//   Additionally, we might want to look into using IKFast to generate a more accurate solver.

public class UR5eAnalyticalIK : IKSolver
{
    private float d1 = 0.1625f;
    private float a2 = -0.425f;
    private float a3 = -0.3922f;
    private float d4 =  0.1333f;
    private float d5 =  0.0997f;
    private float d6 =  0.0996f;
    private float ZERO_THRESH = 0.00000001f;

    public override float[] InverseKinematics(Transform target, float[] initialAngles)
    {
        // Print target position
        Debug.Log("Target position: " + target.position);

        // Run forward kinematics to get the initial pose and print the result
        Matrix4x4 mInitial = Forward(initialAngles);
        string sInitial = "Initial pose: ";
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                sInitial += mInitial[i,j] + " ";
            }
        }
        Debug.Log(sInitial);

        // 1) Get Unity transform and 2) Convert from Unity coords to Robot coords
        // Swap the axes to -z, x, y (?)
        // Vector3 targetPos = new Vector3(-target.position.z, target.position.x, target.position.y);
        // Quaternion targetRot = new Quaternion(-target.rotation.z, target.rotation.x, target.rotation.y, target.rotation.w);
        Vector3 targetPos = target.position;
        Quaternion targetRot = target.rotation;

        Matrix4x4 mRobot = Matrix4x4.TRS(targetPos, targetRot, Vector3.one);

        // 3) Build T[] from mRobot in row-major order
        double[] T = new double[16];
        T[0]  = mRobot[0,0]; T[1]  = mRobot[0,1]; T[2]  = mRobot[0,2]; T[3]  = mRobot[0,3];
        T[4]  = mRobot[1,0]; T[5]  = mRobot[1,1]; T[6]  = mRobot[1,2]; T[7]  = mRobot[1,3];
        T[8]  = mRobot[2,0]; T[9]  = mRobot[2,1]; T[10] = mRobot[2,2]; T[11] = mRobot[2,3];
        T[12] = mRobot[3,0]; T[13] = mRobot[3,1]; T[14] = mRobot[3,2]; T[15] = mRobot[3,3];

        // Print T
        string sT = "T: ";
        for(int i = 0; i < 16; i++)
        {
            sT += T[i] + " ";
        }
        Debug.Log(sT);

        // 4) Call the inverse solver
        double[] qSols = new double[6*8];  // up to 8 solutions
        int nSols = Inverse(T, qSols, 0.0);

        // 5) Pick solution closest to 'initialAngles'
        if(nSols == 0)
        {
            Debug.LogWarning("No IK solutions found for this pose!");
            return null;
        }

        Debug.Log("Found " + nSols + " IK solutions");

        // Print all of the solutions
        for(int i = 0; i < nSols; i++)
        {
            string s = "Solution " + i + ": ";
            for(int j = 0; j < 6; j++)
            {
                s += qSols[i*6 + j] + " ";
            }
            Debug.Log(s);
        }

        float[] bestSolution = new float[6];
        double bestDist = double.MaxValue;
        int bestIdx = -1;

        for(int i = 0; i < nSols; i++)
        {
            double sumSq = 0.0;
            for(int j = 0; j < 6; j++)
            {
                double diff = qSols[i*6 + j] - initialAngles[j] * Math.PI / 180.0;
                sumSq += diff*diff;
            }
            if(sumSq < bestDist)
            {
                bestDist = sumSq;
                bestIdx = i;
            }
        }

        // For each solution, convert to degrees and then print the forward kinematics result using those angles
        for(int i = 0; i < nSols; i++)
        {
            for(int j = 0; j < 6; j++)
            {
                qSols[i*6 + j] = qSols[i*6 + j] * 180.0 / Math.PI;
            }

            Matrix4x4 m = Forward(new float[] { (float)qSols[i*6 + 0], (float)qSols[i*6 + 1], (float)qSols[i*6 + 2], (float)qSols[i*6 + 3], (float)qSols[i*6 + 4], (float)qSols[i*6 + 5] });
            string s = "Forward kinematics for solution " + i + ": ";
            for(int k = 0; k < 4; k++)
            {
                for(int j = 0; j < 4; j++)
                {
                    s += m[k,j] + " ";
                }
            }
            Debug.Log(s);
        }

        bestIdx = 0;  // For now, just use the first solution
        for(int j = 0; j < 6; j++)
        {
            bestSolution[j] = (float)qSols[bestIdx*6 + j] * 180.0f / (float)Math.PI;
        }

        // Print the best solution
        string best = "Best solution: ";
        for(int j = 0; j < 6; j++)
        {
            best += bestSolution[j] + " ";
        }
        Debug.Log(best);

        // Run forward kinematics to get the final pose and print the result
        Matrix4x4 mFinal = Forward(bestSolution);
        string sFinal = "Final pose: ";
        for(int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                sFinal += mFinal[i,j] + " ";
            }
        }
        Debug.Log(sFinal);

        return bestSolution;
    }

    private int Inverse(double[] T, double[] qSols, double q6_des = 0.0)
    {
        int num_sols = 0;

        double T02 = -T[0];
        double T00 =  T[1];
        double T01 =  T[2];
        double T03 = -T[3];

        double T12 = -T[4];
        double T10 =  T[5];
        double T11 =  T[6];
        double T13 = -T[7];

        double T22 =  T[8];
        double T20 = -T[9];
        double T21 = -T[10];
        double T23 =  T[11];

        ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
        double[] q1 = new double[2];
        {
            double A = d6 * T12 - T13;
            double B = d6 * T02 - T03;
            double R = A*A + B*B;

            if(Math.Abs(A) < ZERO_THRESH)
            {
                // handle arcsin
                double div;
                if(Math.Abs(Math.Abs(d4) - Math.Abs(B)) < ZERO_THRESH)
                    div = -SIGN(d4) * SIGN(B);
                else
                    div = -d4/B;

                double val = Math.Asin(div);
                if(Math.Abs(val) < ZERO_THRESH) val = 0.0;
                if(val < 0.0) q1[0] = val + 2.0*Math.PI; 
                else          q1[0] = val;
                q1[1] = Math.PI - val;
            }
            else if(Math.Abs(B) < ZERO_THRESH)
            {
                // handle arccos
                double div;
                if(Math.Abs(Math.Abs(d4) - Math.Abs(A)) < ZERO_THRESH)
                    div = SIGN(d4) * SIGN(A);
                else
                    div = d4/A;

                double val = Math.Acos(div);
                q1[0] = val;
                q1[1] = 2.0*Math.PI - val;
            }
            else if(d4*d4 > R)
            {
                // No solution
                return 0;
            }
            else
            {
                double val = d4 / Math.Sqrt(R);
                double arccos = Math.Acos(val);
                double arctan = Math.Atan2(-B, A);

                double pos = arccos + arctan;
                double neg = -arccos + arctan;
                if(Math.Abs(pos) < ZERO_THRESH) pos = 0.0;
                if(Math.Abs(neg) < ZERO_THRESH) neg = 0.0;

                q1[0] = (pos >= 0.0) ? pos : 2.0*Math.PI + pos;
                q1[1] = (neg >= 0.0) ? neg : 2.0*Math.PI + neg;
            }
        }

        ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
        // q5[i][j] for i in [0..1], j in [0..1]
        double[][] q5 = new double[2][] { new double[2], new double[2] };

        {
            for(int i=0; i<2; i++)
            {
                double numer = T03 * Math.Sin(q1[i]) - T13 * Math.Cos(q1[i]) - d4;
                double div;
                if(Math.Abs(Math.Abs(numer) - Math.Abs(d6)) < ZERO_THRESH)
                    div = SIGN(numer) * SIGN(d6);
                else
                    div = numer / d6;

                double val = Math.Acos(div);
                q5[i][0] = val;
                q5[i][1] = 2.0*Math.PI - val;
            }
        }

        {
            // big loop over possible i, j
            for(int i = 0; i < 2; i++)
            {
                for(int j = 0; j < 2; j++)
                {
                    double c1 = Math.Cos(q1[i]);
                    double s1 = Math.Sin(q1[i]);
                    double c5 = Math.Cos(q5[i][j]);
                    double s5 = Math.Sin(q5[i][j]);

                    ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
                    double q6;
                    if(Math.Abs(s5) < ZERO_THRESH)
                    {
                        q6 = q6_des;
                    }
                    else
                    {
                        // SIGN(s5)*-(T01*s1 - T11*c1) => same as -SIGN(s5)*(T01*s1 - T11*c1)
                        double y = -SIGN(s5)*(T01*s1 - T11*c1);
                        double x =  SIGN(s5)*(T00*s1 - T10*c1);
                        q6 = Math.Atan2(y, x);

                        if(Math.Abs(q6) < ZERO_THRESH) q6 = 0.0;
                        if(q6 < 0.0) q6 += 2.0*Math.PI;
                    }

                    // Now solve for q2, q3, q4
                    double c6 = Math.Cos(q6);
                    double s6 = Math.Sin(q6);

                    double x04x = -s5*(T02*c1 + T12*s1)
                                - c5*( s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1) );

                    double x04y = c5*(T20*c6 - T21*s6) - T22*s5;

                    double p13x = d5*( s6*(T00*c1 + T10*s1) 
                                    + c6*(T01*c1 + T11*s1) )
                                - d6*(T02*c1 + T12*s1)
                                + T03*c1 + T13*s1;

                    double p13y = T23 - d1 - d6*T22
                                + d5*(T21*c6 + T20*s6);

                    // solve for q2, q3
                    double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3)/(2.0*a2*a3);
                    // clamp if out of range
                    if(Math.Abs(Math.Abs(c3) - 1.0) < ZERO_THRESH) 
                    {
                        c3 = SIGN(c3); 
                    } 
                    else if(Math.Abs(c3) > 1.0)
                    {
                        // no solution
                        continue;
                    }

                    double arccos_c3 = Math.Acos(c3);
                    double[] q3vals = new double[2] { arccos_c3, 2.0*Math.PI - arccos_c3 };

                    /// here
                    /// 

                    double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
                    double s3 = Math.Sin(arccos_c3);
                    double A = a2 + a3*c3;
                    double B = a3*s3;

                    double q2val = Math.Atan2( (A*p13y - B*p13x)/denom,
                                            (A*p13x + B*p13y)/denom );

                    double q2alt = Math.Atan2( (A*p13y + B*p13x)/denom,
                                            (A*p13x - B*p13y)/denom );
                    
                    double[] q2vals = new double[2] { q2val, q2alt };

                    // compute q4 from q2+q3
                    double c23_0 = Math.Cos(q2vals[0] + q3vals[0]);
                    double s23_0 = Math.Sin(q2vals[0] + q3vals[0]);
                    double c23_1 = Math.Cos(q2vals[1] + q3vals[1]);
                    double s23_1 = Math.Sin(q2vals[1] + q3vals[1]);

                    double q4_0 = Math.Atan2(c23_0*x04y - s23_0*x04x,
                                            x04x*c23_0 + x04y*s23_0);
                    
                    double q4_1 = Math.Atan2(c23_1*x04y - s23_1*x04x,
                                            x04x*c23_1 + x04y*s23_1);

                    double[] q4vals = new double[2] { q4_0, q4_1 };
                    
                    for (int k = 0; k < 2; k++)
                    {
                        if (Math.Abs(q2vals[k]) < ZERO_THRESH) q2vals[k] = 0.0;
                        else if (q2vals[k] < 0.0) q2vals[k] += 2.0*Math.PI;

                        if (Math.Abs(q4vals[k]) < ZERO_THRESH) q4vals[k] = 0.0;
                        else if (q4vals[k] < 0.0) q4vals[k] += 2.0*Math.PI;

                        // store solution in q_sols
                        qSols[num_sols*6 + 0] = q1[i];
                        qSols[num_sols*6 + 1] = q2vals[k];
                        qSols[num_sols*6 + 2] = q3vals[k];
                        qSols[num_sols*6 + 3] = q4vals[k];
                        qSols[num_sols*6 + 4] = q5[i][j];
                        qSols[num_sols*6 + 5] = q6;

                        num_sols++;
                    }
                }
            }
        }

        return num_sols;
    }

    private Matrix4x4 Forward(float[] jointAngles)
    {
        double s1 = Math.Sin(jointAngles[0] * Math.PI / 180.0);
        double c1 = Math.Cos(jointAngles[0] * Math.PI / 180.0);

        double q23 = jointAngles[1];
        double q234 = jointAngles[1];
        double s2 = Math.Sin(jointAngles[1] * Math.PI / 180.0);
        double c2 = Math.Cos(jointAngles[1] * Math.PI / 180.0);

        double s3 = Math.Sin(jointAngles[2] * Math.PI / 180.0);
        double c3 = Math.Cos(jointAngles[2] * Math.PI / 180.0);
        q23 += jointAngles[2];
        q234 += jointAngles[2];

        double s4 = Math.Sin(jointAngles[3] * Math.PI / 180.0);
        double c4 = Math.Cos(jointAngles[3] * Math.PI / 180.0);
        q234 += jointAngles[3];

        double s5 = Math.Sin(jointAngles[4] * Math.PI / 180.0);
        double c5 = Math.Cos(jointAngles[4] * Math.PI / 180.0);

        double s6 = Math.Sin(jointAngles[5] * Math.PI / 180.0);
        double c6 = Math.Cos(jointAngles[5] * Math.PI / 180.0);

        double s23 = Math.Sin(q23);
        double c23 = Math.Cos(q23);

        double s234 = Math.Sin(q234);
        double c234 = Math.Cos(q234);

        Matrix4x4 T = new Matrix4x4();
        T[0,0] = (float)(c234*c1*s5 - c5*s1);
        T[0,1] = (float)(c6*(s1*s5 + c234*c1*c5) - s234*c1*s6);
        T[0,2] = (float)(-s6*(s1*s5 + c234*c1*c5) - s234*c1*c6);
        T[0,3] = (float)(d6*c234*c1*s5 - a3*c23*c1 - a2*c1*c2 - d6*c5*s1 - d5*s234*c1 - d4*s1);
        T[1,0] = (float)(c1*c5 + c234*s1*s5);
        T[1,1] = (float)(-c6*(c1*s5 - c234*c5*s1) - s234*s1*s6);
        T[1,2] = (float)(s6*(c1*s5 - c234*c5*s1) - s234*c6*s1);
        T[1,3] = (float)(d6*(c1*c5 + c234*s1*s5) + d4*c1 - a3*c23*s1 - a2*c2*s1 - d5*s234*s1);
        T[2,0] = (float)(-s234*s5);
        T[2,1] = (float)(-c234*s6 - s234*c5*c6);
        T[2,2] = (float)(s234*c5*s6 - c234*c6);
        T[2,3] = (float)(d1 + a3*s23 + a2*s2 - d5*(c23*c4 - s23*s4) - d6*s5*(c23*s4 + s23*c4));
        T[3,0] = 0.0f;
        T[3,1] = 0.0f;
        T[3,2] = 0.0f;
        T[3,3] = 1.0f;

        return T;
    }

    private int SIGN(double x) {
        int gt = (x > 0) ? 1 : 0;
        int lt = (x < 0) ? 1 : 0;
        return gt - lt;
    }
}
