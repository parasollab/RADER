using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UR5eAnalyticalIK : IKSolver
{
    private float d1 = 0.1625f;
    private float a2 = -0.425f;
    private float a3 = -0.3922f;
    private float d4 =  0.1333f;
    private float d5 =  0.0997f;
    private float d6 =  0.099f;
    private float ZERO_THRESH = 0.00000001f;

    // TODO handle the case of no solution by changing the target to the closest reachable point
    // TODO make the target relative to the robot base
    public override float[] InverseKinematics(Transform target, float[] initialAngles)
    {
        // 1) Build T[] from target's world (or local) transform
        //    This is robot-specific. You need to map Unity's Transform
        //    into your 4x4 matrix as your solver expects. For example:

        Matrix4x4 m = target.localToWorldMatrix; 
        // or target.localToWorldMatrix, or target.worldToLocalMatrix, 
        // depending on your robot’s reference frame

        // Convert to a double[16], row-major:
        double[] T = new double[16];
        // Unity's Matrix4x4 has m[row, col].
        // row-major means T = [m00, m01, m02, m03, m10, m11, ...].
        // Double-check your usage vs. solver's usage.
        T[0]  = m[0,0]; T[1]  = m[0,1]; T[2]  = m[0,2]; T[3]  = m[0,3];
        T[4]  = m[1,0]; T[5]  = m[1,1]; T[6]  = m[1,2]; T[7]  = m[1,3];
        T[8]  = m[2,0]; T[9]  = m[2,1]; T[10] = m[2,2]; T[11] = m[2,3];
        T[12] = m[3,0]; T[13] = m[3,1]; T[14] = m[3,2]; T[15] = m[3,3];

        // 2) Call the inverse solver
        double[] qSols = new double[6*8];  // up to 8 solutions, each 6 DOF
        int nSols = Inverse(T, qSols, 0.0); 
        if(nSols == 0)
        {
            // no valid IK solutions found!
            Debug.LogWarning("No IK solutions found for this pose!");
            return null;
        }

        // 3) Among all solutions, pick the one closest to 'initialAngles'.
        //    We'll measure distance in joint space using sum of squared diffs:
        float[] bestSolution = new float[6];
        double bestDist = double.MaxValue;
        int bestIdx = -1;

        for(int i = 0; i < nSols; i++)
        {
            double sumSq = 0.0;
            for(int j = 0; j < 6; j++)
            {
                double diff = qSols[i*6 + j] - initialAngles[j];
                sumSq += diff*diff;
            }
            if(sumSq < bestDist)
            {
                bestDist = sumSq;
                bestIdx = i;
            }
        }

        // copy that best solution to the output array
        for(int j = 0; j < 6; j++)
        {
            bestSolution[j] = (float)qSols[bestIdx*6 + j];
        }

        return bestSolution;
    }

    private int Inverse(double[] T, double[] qSols, double q6_des = 0.0)
    {
        int num_sols = 0;

        // The original C++ code is reading T in a peculiar order. 
        // We must replicate that EXACT reading order. The snippet was:
        //   double T02 = -*T; T++; double T00 =  *T; T++; double T01 =  *T; T++; double T03 = -*T; T++;
        //   double T12 = -*T; T++; double T10 =  *T; T++; double T11 =  *T; T++; double T13 = -*T; T++;
        //   double T22 =  *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 =  *T;

        // So T[0] ends up used as -T[0] => T02
        //    T[1] => T00
        //    T[2] => T01
        //    T[3] => -T[3] => T03
        //    T[4] => -T[4] => T12
        //    T[5] => T10
        //    T[6] => T11
        //    T[7] => -T[7] => T13
        //    T[8] =>  T22
        //    T[9] => -T[9] => T20
        //    T[10] => -T[10] => T21
        //    T[11] =>  T23
        // (Indices above are row-major if T was a 4x4. Just keep them consistent with your code.)

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

        for(int i=0; i<2; i++)
        {
            double numer = (T03 * Math.Sin(q1[i]) - T13 * Math.Cos(q1[i]) - d4);
            double div;
            if(Math.Abs(Math.Abs(numer) - Math.Abs(d6)) < ZERO_THRESH)
                div = SIGN(numer) * SIGN(d6);
            else
                div = numer / d6;

            double val = Math.Acos(div);
            q5[i][0] = val;
            q5[i][1] = 2.0*Math.PI - val;
        }

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

                for(int q3i = 0; q3i < 2; q3i++)
                {
                    double s3 = Math.Sin(q3vals[q3i]);
                    double A = a2 + a3*c3;
                    double B = a3*s3;
                    double denom = A*A + B*B;  // used inside the atan2 below

                    double q2val = Math.Atan2( (A*p13y - B*p13x)/denom,
                                            (A*p13x + B*p13y)/denom );

                    double q2alt = Math.Atan2( (A*p13y + B*p13x)/denom,
                                            (A*p13x - B*p13y)/denom );

                    double[] q2vals = new double[2] { q2val, q2alt };

                    // compute q4 from q2+q3
                    for(int q2i = 0; q2i < 2; q2i++)
                    {
                        double q2_ = q2vals[q2i];
                        double q3_ = q3vals[q3i];

                        double c23 = Math.Cos(q2_ + q3_);
                        double s23 = Math.Sin(q2_ + q3_);

                        double q4_ = Math.Atan2(c23*x04y - s23*x04x,
                                                x04x*c23 + x04y*s23);

                        // fix angle ranges
                        if(Math.Abs(q2_) < ZERO_THRESH)  q2_ = 0.0;
                        else if(q2_ < 0.0) q2_ += 2.0*Math.PI;

                        if(Math.Abs(q4_) < ZERO_THRESH)  q4_ = 0.0;
                        else if(q4_ < 0.0) q4_ += 2.0*Math.PI;

                        // store solution in q_sols
                        int idx = num_sols * 6;
                        qSols[idx + 0] = q1[i];
                        qSols[idx + 1] = q2_;
                        qSols[idx + 2] = q3_;
                        qSols[idx + 3] = q4_;
                        qSols[idx + 4] = q5[i][j];
                        qSols[idx + 5] = q6;

                        num_sols++;
                    }
                }
            }
        }

        return num_sols;
    }

    private int SIGN(double x) {
        int gt = (x > 0) ? 1 : 0;
        int lt = (x < 0) ? 1 : 0;
        return gt - lt;
    }
}
