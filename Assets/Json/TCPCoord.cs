[System.Serializable]
public class TCPcoord
{
    public double[] JointInfo = new double[12];
 

    public TCPcoord(double pointTime, float joint1, float joint2, float joint3, float joint4, float joint5, float joint6,  double tcpX, double tcpY, double tcpZ, double sgnCollision,double sgnGrip)
    {
        JointInfo[0]=pointTime;
        JointInfo[1] = joint1;
        JointInfo[2] = joint2;
        JointInfo[3] = joint3;
        JointInfo[4] = joint4;
        JointInfo[5] = joint5;
        JointInfo[6] = joint6;
        JointInfo[7] = tcpX;
        JointInfo[8] = tcpY;
        JointInfo[9] = tcpZ;
        JointInfo[10] = sgnCollision;
        JointInfo[11] = sgnGrip;
    }
}