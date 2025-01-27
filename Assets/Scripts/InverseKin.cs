using UnityEngine;
using System.Numerics;


public static class InverseKin
{
    //public GameObject cube_test;
    //public GameObject j0;
    //public float[] Get_pos()
    //{
    //    UnityEngine.Vector3 blockPosition = cube_test.transform.position;
    //    UnityEngine.Vector3 j0Position = j0.transform.position;
    //    float[] positionArray = new float[3];
    //    positionArray[0] = blockPosition.x - j0Position.x;
    //    positionArray[1] = blockPosition.y - j0Position.y;
    //    positionArray[2] = blockPosition.z - j0Position.z;
    //    return positionArray;
    //}

    public struct solution
    {
        public float[] angles;
        public bool allnan;
    }

    public static solution Inv_calculator(float[] cube_pos, float[] starting,float[] EulerAngle)
    {
        //转化成Matlab算法中的坐标
        float[] calibrateArray = new float[3];
        calibrateArray[0] = cube_pos[2] * 1000f;
        calibrateArray[1] = -cube_pos[0] * 1000f;
        calibrateArray[2] = cube_pos[1] * 1000f;
        //注意坐标系方向！！！！！！！！！！
        // 创建旋转四元数
        float rx = EulerAngle[0];
        float ry = EulerAngle[1];
        float rz = EulerAngle[2];
        UnityEngine.Quaternion rotationQuaternion = UnityEngine.Quaternion.Euler(rx, ry, rz);
        // 创建变换矩阵 T
        //UnityEngine.Matrix4x4 T_matrix = UnityEngine.Matrix4x4.identity;
        // 将旋转四元数转换为旋转矩阵
        UnityEngine.Matrix4x4 rotationMatrix = UnityEngine.Matrix4x4.Rotate(rotationQuaternion);


        float[,] T = new float[4, 4];

        for(int i=0;i<3;i++)
        {
            for(int j =0; j<3;j++)
            {
                T[i, j] = rotationMatrix[i, j];
            }
        }

        ////垂直向下
        //if(VerticalSign == true)
        //{
        //    T[0, 1] = -1f;
        //    T[1, 0] = -1f;
        //    T[2, 2] = -1f;
        //}

        T[3, 3] = 1f;
        T[0, 3] = calibrateArray[0];
        T[1, 3] = calibrateArray[1];
        T[2, 3] = calibrateArray[2];
        //print_array2D(T);
        float[,] theta = inverseKinematics(T);
        //Debug.Log("Theta");
        //print_array2D(theta);
        // float[] starting_defined = {-Mathf.PI/2 , -Mathf.PI/2 , 0.0f , -Mathf.PI/2 , -Mathf.PI/2 , -Mathf.PI/4};
        // Debug.Log("Origin:");
        // print_array1D(starting);
        starting = Mat_angle(starting);
        // Debug.Log("Modified:");
        // print_array1D(starting);
        //float[] optimal;
        solution mysolution = MinJointVar(theta, starting);
        //optimal = MinJointVar(theta, starting);
        //print_array1D(optimal);
        float[] Input_angle = UI_angle(mysolution.angles);
        mysolution.angles = Input_angle;
        return mysolution;
    }

    public static float[,] inverseKinematics(float[,] T)
    {
        // 变换矩阵T已知
        // SDH:使用标准DH矩阵
        float[] a = new float[] { 0, -247.9943f, -228.0397f, 0, 0, 0 };
        float[] d = new float[] { 150.55003f, 0, 0, 115f, 116.7504f, 264.8015f };
        float[] alpha = new float[] { Mathf.PI / 2, 0, 0, Mathf.PI / 2, -Mathf.PI / 2, 0 };

        float nx = T[0, 0], ny = T[1, 0], nz = T[2, 0];
        float ox = T[0, 1], oy = T[1, 1], oz = T[2, 1];
        float ax = T[0, 2], ay = T[1, 2], az = T[2, 2];
        float px = T[0, 3], py = T[1, 3], pz = T[2, 3];

        // 求解关节角1
        float m = d[5] * ay - py;
        float n = ax * d[5] - px;
        float[,] theta1 = new float[1, 2];
        theta1[0, 0] = Mathf.Atan2(m, n) - Mathf.Atan2(d[3], Mathf.Sqrt(m * m + n * n - d[3] * d[3]));
        theta1[0, 1] = Mathf.Atan2(m, n) - Mathf.Atan2(d[3], -Mathf.Sqrt(m * m + n * n - d[3] * d[3]));

        // 求解关节角5
        float[,] theta5 = new float[2, 2];
        theta5[0, 0] = Mathf.Acos(ax * Mathf.Sin(theta1[0, 0]) - ay * Mathf.Cos(theta1[0, 0]));
        theta5[0, 1] = Mathf.Acos(ax * Mathf.Sin(theta1[0, 1]) - ay * Mathf.Cos(theta1[0, 1]));
        theta5[1, 0] = -Mathf.Acos(ax * Mathf.Sin(theta1[0, 0]) - ay * Mathf.Cos(theta1[0, 0]));
        theta5[1, 1] = -Mathf.Acos(ax * Mathf.Sin(theta1[0, 1]) - ay * Mathf.Cos(theta1[0, 1]));

        // 求解关节角6
        float[] mm = new float[] { nx * Mathf.Sin(theta1[0, 0]) - ny * Mathf.Cos(theta1[0, 0]), nx * Mathf.Sin(theta1[0, 1]) - ny * Mathf.Cos(theta1[0, 1]) };
        float[] nn = new float[] { ox * Mathf.Sin(theta1[0, 0]) - oy * Mathf.Cos(theta1[0, 0]), ox * Mathf.Sin(theta1[0, 1]) - oy * Mathf.Cos(theta1[0, 1]) };
        float[,] theta6 = new float[2, 2];
        theta6[0, 0] = Mathf.Atan2(mm[0], nn[0]) - Mathf.Atan2(Mathf.Sin(theta5[0, 0]), 0);
        theta6[0, 1] = Mathf.Atan2(mm[1], nn[1]) - Mathf.Atan2(Mathf.Sin(theta5[0, 1]), 0);
        theta6[1, 0] = Mathf.Atan2(mm[0], nn[0]) - Mathf.Atan2(Mathf.Sin(theta5[1, 0]), 0);
        theta6[1, 1] = Mathf.Atan2(mm[1], nn[1]) - Mathf.Atan2(Mathf.Sin(theta5[1, 1]), 0);

        // 求解关节角3
        float[,] mmm = new float[2, 2];
        float[,] nnn = new float[2, 2];
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                mmm[i, j] = d[4] * (Mathf.Sin(theta6[i, j]) * (nx * Mathf.Cos(theta1[0, j]) + ny * Mathf.Sin(theta1[0, j])) + Mathf.Cos(theta6[i, j]) * (ox * Mathf.Cos(theta1[0, j]) + oy * Mathf.Sin(theta1[0, j]))) - d[5] * (ax * Mathf.Cos(theta1[0, j]) + ay * Mathf.Sin(theta1[0, j])) + px * Mathf.Cos(theta1[0, j]) + py * Mathf.Sin(theta1[0, j]);
                nnn[i, j] = pz - d[0] - az * d[5] + d[4] * (oz * Mathf.Cos(theta6[i, j]) + nz * Mathf.Sin(theta6[i, j]));
            }
        }

        Complex[,] theta3_complex = new Complex[4, 2];
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                float numerator = mmm[i, j] * mmm[i, j] + nnn[i, j] * nnn[i, j] - a[1] * a[1] - a[2] * a[2];
                float denominator = 2 * a[1] * a[2];
                theta3_complex[i, j] = Mathf.Acos(numerator / denominator);
                theta3_complex[i + 2, j] = -Mathf.Acos(numerator / denominator);
            }
        }

        // 把关节3复数部分变成nan
        for (int i = 0; i < theta3_complex.GetLength(0); i++)
        {
            for (int j = 0; j < theta3_complex.GetLength(1); j++)
            {
                if (!Mathf.Approximately((float)theta3_complex[i, j].Imaginary, 0f))
                {
                    theta3_complex[i, j] = float.NaN;
                }
            }
        }

        float[,] theta3 = new float[4, 2];
        for (int i = 0; i < theta3.GetLength(0); i++)
        {
            for (int j = 0; j < theta3.GetLength(1); j++)
            {
                theta3[i, j] = (float)theta3_complex[i, j].Real;
            }
        }

        // 求解关节角2
        float[,] s2 = new float[4, 2];
        float[,] c2 = new float[4, 2];
        float[,] theta2 = new float[4, 2];
        float[,] mmm_s2 = new float[4, 2];
        float[,] nnn_s2 = new float[4, 2];

        // 将 mmm 复制到 mmm_s2
        mmm_s2[0, 0] = mmm[0, 0];
        mmm_s2[0, 1] = mmm[0, 1];
        mmm_s2[1, 0] = mmm[1, 0];
        mmm_s2[1, 1] = mmm[1, 1];

        // 复制 mmm 到剩余的行
        mmm_s2[2, 0] = mmm[0, 0];
        mmm_s2[2, 1] = mmm[0, 1];
        mmm_s2[3, 0] = mmm[1, 0];
        mmm_s2[3, 1] = mmm[1, 1];

        // 将 nnn 复制到 nnn_s2
        nnn_s2[0, 0] = nnn[0, 0];
        nnn_s2[0, 1] = nnn[0, 1];
        nnn_s2[1, 0] = nnn[1, 0];
        nnn_s2[1, 1] = nnn[1, 1];

        // 复制 nnn 到剩余的行
        nnn_s2[2, 0] = nnn[0, 0];
        nnn_s2[2, 1] = nnn[0, 1];
        nnn_s2[3, 0] = nnn[1, 0];
        nnn_s2[3, 1] = nnn[1, 1];

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 2; j++)
            {
                s2[i, j] = ((a[2] * Mathf.Cos(theta3[i, j]) + a[1]) * nnn_s2[i, j] - a[2] * Mathf.Sin(theta3[i, j]) * mmm_s2[i, j]) / (a[1] * a[1] + a[2] * a[2] + 2 * a[1] * a[2] * Mathf.Cos(theta3[i, j]));
                c2[i, j] = (mmm_s2[i, j] + a[2] * Mathf.Sin(theta3[i, j]) * s2[i, j]) / (a[2] * Mathf.Cos(theta3[i, j]) + a[1]);
                theta2[i, j] = Mathf.Atan2(s2[i, j], c2[i, j]);
            }
        }

        // 整理关节角1 5 6 3 2
        float[,] theta = new float[8, 6];
        theta[0, 0] = theta1[0, 0];
        theta[1, 0] = theta1[0, 0];
        theta[2, 0] = theta1[0, 0];
        theta[3, 0] = theta1[0, 0];
        theta[4, 0] = theta1[0, 1];
        theta[5, 0] = theta1[0, 1];
        theta[6, 0] = theta1[0, 1];
        theta[7, 0] = theta1[0, 1];

        theta[0, 4] = theta5[0, 0];
        theta[1, 4] = theta5[0, 0];
        theta[2, 4] = theta5[1, 0];
        theta[3, 4] = theta5[1, 0];
        theta[4, 4] = theta5[0, 1];
        theta[5, 4] = theta5[0, 1];
        theta[6, 4] = theta5[1, 1];
        theta[7, 4] = theta5[1, 1];

        theta[0, 5] = theta6[0, 0];
        theta[1, 5] = theta6[0, 0];
        theta[2, 5] = theta6[1, 0];
        theta[3, 5] = theta6[1, 0];
        theta[4, 5] = theta6[0, 1];
        theta[5, 5] = theta6[0, 1];
        theta[6, 5] = theta6[1, 1];
        theta[7, 5] = theta6[1, 1];

        theta[0, 1] = theta2[0, 0];
        theta[1, 1] = theta2[2, 0];
        theta[2, 1] = theta2[1, 0];
        theta[3, 1] = theta2[3, 0];
        theta[4, 1] = theta2[0, 1];
        theta[5, 1] = theta2[2, 1];
        theta[6, 1] = theta2[1, 1];
        theta[7, 1] = theta2[3, 1];

        theta[0, 2] = theta3[0, 0];
        theta[1, 2] = theta3[2, 0];
        theta[2, 2] = theta3[1, 0];
        theta[3, 2] = theta3[3, 0];
        theta[4, 2] = theta3[0, 1];
        theta[5, 2] = theta3[2, 1];
        theta[6, 2] = theta3[1, 1];
        theta[7, 2] = theta3[3, 1];

        //求解关节角4
        for (int i = 0; i < 8; i++)
        {
            theta[i, 3] = Mathf.Atan2(-Mathf.Sin(theta[i, 5]) * (nx * Mathf.Cos(theta[i, 0]) + ny * Mathf.Sin(theta[i, 0])) - Mathf.Cos(theta[i, 5]) * (ox * Mathf.Cos(theta[i, 0]) + oy * Mathf.Sin(theta[i, 0])), oz * Mathf.Cos(theta[i, 5]) + nz * Mathf.Sin(theta[i, 5])) - theta[i, 1] - theta[i, 2];
        }

        // 返回所有解
        return theta;
    }

    public static void print_array2D(float[,] array2D)
    {
        string arrayString = "";
        int rowCount = array2D.GetLength(0);
        int colCount = array2D.GetLength(1);

        for (int i = 0; i < rowCount; i++)
        {
            for (int j = 0; j < colCount; j++)
            {
                arrayString += array2D[i, j] + " ";
            }
            arrayString += "\n";
        }

        Debug.Log(arrayString);
    }

    public static void print_array1D(float[] array1D)
    {
        string arrayString = "";
        for (int j = 0; j < array1D.GetLength(0); j++)
        {
            arrayString += array1D[j] + " ";
        }
        Debug.Log(arrayString);
    }

    public static solution MinJointVar(float[,] theta, float[] starting)
    {

        // 计算norm2和error
        solution mysolution;
        float[,] varNorm = new float[theta.GetLength(0), theta.GetLength(1)];
        for (int i = 0; i < theta.GetLength(0); i++)
        {
            for (int j = 0; j < theta.GetLength(1); j++)
            {
                varNorm[i, j] = theta[i, j] - starting[j];
                if (varNorm[i, j] > Mathf.PI)
                    varNorm[i, j] = 2 * Mathf.PI - varNorm[i, j];
                else if (varNorm[i, j] < -Mathf.PI)
                    varNorm[i, j] = 2 * Mathf.PI + varNorm[i, j];
            }
        }

        float[] varSolution = new float[theta.GetLength(0)];
        for (int i = 0; i < theta.GetLength(0); i++)
        {
            float sum = 0;
            bool hasNAN = false;
            for (int j = 0; j < theta.GetLength(1); j++)
            {
                if (float.IsNaN(varNorm[i, j]))
                {
                    hasNAN = true;
                }
                sum += Mathf.Abs(varNorm[i, j]);
            }
            if (!hasNAN)
            {
                varSolution[i] = sum;
            }
            else
            {
                varSolution[i] = float.NaN;
            }
        }

        float minSolution = 10000f;
        int index = 0;
        bool AllNAN = true;
        for (int i = 1; i < varSolution.Length; i++)
        {
            if (varSolution[i] < minSolution && !float.IsNaN(varSolution[i]))
            {
                minSolution = varSolution[i];
                AllNAN = false;
                index = i;
            }
        }

        float[] optimal = new float[theta.GetLength(1)];
        if (!AllNAN)
        {
            for (int j = 0; j < theta.GetLength(1); j++)
            {
                optimal[j] = theta[index, j];
            }
        }
        else
        {
            optimal = starting;

        }
        mysolution.angles = optimal;
        mysolution.allnan = AllNAN;
        return mysolution;
    }

    // Convert Matlab angle to UI angle
    public static float[] UI_angle(float[] IK_result)
    {
        float[] UI_result = new float[6];
        UI_result[0] = IK_result[0] * 180 / Mathf.PI;
        UI_result[1] = -IK_result[1] * 180 / Mathf.PI;
        UI_result[2] = -IK_result[2] * 180 / Mathf.PI;
        UI_result[3] = -IK_result[3] * 180 / Mathf.PI;
        UI_result[4] = (Mathf.PI + IK_result[4]) * 180 / Mathf.PI;
        UI_result[5] = (1 / 4 * Mathf.PI - IK_result[5]) * 180 / Mathf.PI;
        return UI_result;
    }

    // Convert UI angle to Matlab angle
    public static float[] Mat_angle(float[] UI_result)
    {
        float[] Mat_result = new float[6];
        Mat_result[0] = UI_result[0] * Mathf.PI / 180;
        Mat_result[1] = -UI_result[1] * Mathf.PI / 180;
        Mat_result[2] = -UI_result[2] * Mathf.PI / 180;
        Mat_result[3] = -UI_result[3] * Mathf.PI / 180;
        Mat_result[4] = UI_result[4] * Mathf.PI / 180 - Mathf.PI;
        Mat_result[5] = 1 / 4 * Mathf.PI - UI_result[5] * Mathf.PI / 180;
        return Mat_result;
    }

}
