using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

public class BodyScript_camera : MonoBehaviour
{
    public UDPPoseReceiver udpReceive;
    public GameObject head; // 传入模型
    public GameObject leftupperarm;
    public GameObject rightupperarm;
    public GameObject leftforearm;
    public GameObject rightforearm;
    public GameObject lefthand;
    public GameObject righthand;
    public GameObject leftthigh;
    public GameObject rightthigh;
    public GameObject leftshin;
    public GameObject rightshin;
    public GameObject leftfoot;
    public GameObject rightfoot;
    public GameObject leftindex_01;
    public GameObject rightindex_01;
    public GameObject leftindex_02;
    public GameObject rightindex_02;
    public GameObject leftindex_03;
    public GameObject rightindex_03;
    public GameObject leftthumb_01;
    public GameObject rightthumb_01;
    public GameObject leftthumb_02;
    public GameObject rightthumb_02;
    public GameObject leftthumb_03;
    public GameObject rightthumb_03;
    public GameObject leftmiddle_01;
    public GameObject rightmiddle_01;
    public GameObject leftmiddle_02;
    public GameObject rightmiddle_02;
    public GameObject leftmiddle_03;
    public GameObject rightmiddle_03;
    public GameObject leftring_01;
    public GameObject rightring_01;
    public GameObject leftring_02;
    public GameObject rightring_02;
    public GameObject leftring_03;
    public GameObject rightring_03;
    public GameObject leftpinky_01;
    public GameObject rightpinky_01;
    public GameObject leftpinky_02;
    public GameObject rightpinky_02;
    public GameObject leftpinky_03;
    public GameObject rightpinky_03;
    public GameObject model;
    public GameObject hips;
    public Vector3[] _poseDataArray; // 数据数组
    public float lerpSpeed;
    private float headangle;

    private float lastKeyTime = 0f;

    private void Start()
    {

    }

    private void Update()
    {
        headangle = 0;
        _poseDataArray = udpReceive._poseDataArray;

        //转身的动作----------------------------------------------------------------------------

        Vector3 middlePoint11_12 = (_poseDataArray[11] + _poseDataArray[12]) / 2;
        Vector3 projectedPoint0_11_12 = ProjectPointOnPlane(_poseDataArray[0], _poseDataArray[11] - _poseDataArray[12], middlePoint11_12);
        Vector3 projectedPoint0_11_12up = ProjectPointOnPlane(projectedPoint0_11_12, Vector3.up, hips.transform.position);
        Vector3 projectedPointmiddlePoint11_12 = ProjectPointOnPlane(middlePoint11_12, Vector3.up, hips.transform.position);
        hips.transform.forward = Vector3.Lerp(hips.transform.forward, (projectedPointmiddlePoint11_12 - projectedPoint0_11_12up).normalized, Time.deltaTime * lerpSpeed);
        Vector3 fixedRightchest = hips.transform.forward;
        float anglechest = Vector3.SignedAngle(hips.transform.right, Vector3.down, fixedRightchest);
        hips.transform.Rotate(fixedRightchest, anglechest, Space.World);

        //转身动作到此为止，接下来是头部的动作----------------------------------------------------------------------------

        Vector3 middlePoint7_8 = (_poseDataArray[7] + _poseDataArray[8]) / 2;
        head.transform.forward = Vector3.Lerp(head.transform.forward, (middlePoint7_8 - _poseDataArray[0]).normalized, Time.deltaTime * lerpSpeed);


        Vector3 vectorBetween9_10 = _poseDataArray[9] - _poseDataArray[10];
        Vector3 fixedForwardhead = head.transform.forward;
        float anglehead = Vector3.SignedAngle(head.transform.up, vectorBetween9_10, fixedForwardhead);
        head.transform.Rotate(fixedForwardhead, anglehead, Space.World);
        
        //头部动作到此为止，接下来是手臂的动作----------------------------------------------------------------------------

        leftupperarm.transform.right = Vector3.Lerp(leftupperarm.transform.right, (_poseDataArray[13] - _poseDataArray[11]).normalized, Time.deltaTime * lerpSpeed);
        Vector3 fixedRightleftupperarm = leftupperarm.transform.right;
        float angleleftupperarm = Vector3.SignedAngle(leftupperarm.transform.forward, -hips.transform.forward, fixedRightleftupperarm);
        leftupperarm.transform.Rotate(fixedRightleftupperarm, angleleftupperarm, Space.World);

        rightupperarm.transform.right = Vector3.Lerp(rightupperarm.transform.right, (_poseDataArray[14] - _poseDataArray[12]).normalized, Time.deltaTime * lerpSpeed);
        Vector3 fixedRightrightupperarm = rightupperarm.transform.right;
        float anglerightupperarm = Vector3.SignedAngle(rightupperarm.transform.forward, -hips.transform.forward, fixedRightrightupperarm);
        rightupperarm.transform.Rotate(fixedRightrightupperarm, anglerightupperarm, Space.World);

        leftforearm.transform.right = Vector3.Lerp(leftforearm.transform.right, (_poseDataArray[15] - _poseDataArray[13]).normalized, Time.deltaTime * lerpSpeed);
        Vector3 fixedRightleftforearm = leftforearm.transform.right;
        float angleleftforearm = Vector3.SignedAngle(leftforearm.transform.forward, -hips.transform.forward, fixedRightleftforearm);
        leftforearm.transform.Rotate(fixedRightleftforearm, angleleftforearm, Space.World);

        rightforearm.transform.right = Vector3.Lerp(rightforearm.transform.right, (_poseDataArray[16] - _poseDataArray[14]).normalized, Time.deltaTime * lerpSpeed);
        Vector3 fixedRightrightforearm = rightforearm.transform.right;
        float anglerightforearm = Vector3.SignedAngle(rightforearm.transform.forward, -hips.transform.forward, fixedRightrightforearm);
        rightforearm.transform.Rotate(fixedRightrightforearm, anglerightforearm, Space.World);

        //手臂动作到此为止，接下来是手掌的动作----------------------------------------------------------------------------

        Vector3 middlePoint38_50 = (_poseDataArray[38] + _poseDataArray[50]) / 2;
        lefthand.transform.right = Vector3.Lerp(lefthand.transform.right, (middlePoint38_50 - _poseDataArray[33]).normalized, Time.deltaTime * lerpSpeed);
        Vector3 fixedRightlefthand = lefthand.transform.right;
        float anglelefthand = Vector3.SignedAngle(lefthand.transform.forward, _poseDataArray[38] - _poseDataArray[50], fixedRightlefthand);
        lefthand.transform.Rotate(fixedRightlefthand, anglelefthand, Space.World);

        Vector3 middlePoint59_71 = (_poseDataArray[59] + _poseDataArray[71]) / 2;
        righthand.transform.right = Vector3.Lerp(righthand.transform.right, (middlePoint59_71 - _poseDataArray[54]).normalized, Time.deltaTime * lerpSpeed);
        Vector3 fixedRightrighthand = righthand.transform.right;
        float anglerighthand = Vector3.SignedAngle(righthand.transform.forward, _poseDataArray[59] - _poseDataArray[71], fixedRightrighthand);
        righthand.transform.Rotate(fixedRightrighthand, anglerighthand, Space.World);


        //手掌动作到此为止，接下来是腿的动作----------------------------------------------------------------------------

        leftthigh.transform.right = Vector3.Lerp(leftthigh.transform.right, (_poseDataArray[25] - _poseDataArray[23]).normalized, Time.deltaTime * lerpSpeed);
        Vector3 fixedRightleftthigh = leftthigh.transform.right;
        float angleleftthigh = Vector3.SignedAngle(leftthigh.transform.forward, -hips.transform.forward, fixedRightleftthigh);
        leftthigh.transform.Rotate(fixedRightleftthigh, angleleftthigh, Space.World);

        rightthigh.transform.right = Vector3.Lerp(rightthigh.transform.right, (_poseDataArray[26] - _poseDataArray[24]).normalized, Time.deltaTime * lerpSpeed);
        Vector3 fixedRightrightthigh = rightthigh.transform.right;
        float anglerightthigh = Vector3.SignedAngle(rightthigh.transform.forward, -hips.transform.forward, fixedRightrightthigh);
        rightthigh.transform.Rotate(fixedRightrightthigh, anglerightthigh, Space.World);

        Vector3 middlePoint27_25 = _poseDataArray[27] - _poseDataArray[25];
        Vector3 middlePoint27_25up = new Vector3(middlePoint27_25.x, middlePoint27_25.y, middlePoint27_25.z - 85f);
        leftshin.transform.right = Vector3.Lerp(leftshin.transform.right, middlePoint27_25up.normalized, Time.deltaTime * lerpSpeed);
        Vector3 fixedRightleftshin = leftshin.transform.right;
        float angleleftshin = Vector3.SignedAngle(leftshin.transform.forward, -hips.transform.forward, fixedRightleftshin);
        leftshin.transform.Rotate(fixedRightleftshin, angleleftshin, Space.World);

        Vector3 middlePoint28_26 = _poseDataArray[28] - _poseDataArray[26];
        Vector3 middlePoint28_26up = new Vector3(middlePoint28_26.x, middlePoint28_26.y, middlePoint28_26.z - 85f);
        rightshin.transform.right = Vector3.Lerp(rightshin.transform.right, middlePoint28_26up.normalized, Time.deltaTime * lerpSpeed);
        Vector3 fixedRightrightshin = rightshin.transform.right;
        float anglerightshin = Vector3.SignedAngle(rightshin.transform.forward, -hips.transform.forward, fixedRightrightshin);
        rightshin.transform.Rotate(fixedRightrightshin, anglerightshin, Space.World);

        //腿动作到此为止，接下来是脚的动作----------------------------------------------------------------------------
        /*
        Vector3 targetDirection32_28 = (_poseDataArray[32] - _poseDataArray[28]).normalized;
        Quaternion targetRotation32_28 = Quaternion.FromToRotation(leftfoot.transform.forward, targetDirection32_28) * leftfoot.transform.rotation;
        leftfoot.transform.rotation = Quaternion.Lerp(rightforearm.transform.rotation, targetRotation32_28, Time.deltaTime * lerpSpeed);
        */
        leftfoot.transform.right = Vector3.Lerp(leftfoot.transform.right, (_poseDataArray[31] - _poseDataArray[27]).normalized, Time.deltaTime * lerpSpeed);
        Vector3 fixedRightleftfoot = leftfoot.transform.right;
        float angleleftfoot = Vector3.SignedAngle(leftfoot.transform.forward, Vector3.down, fixedRightleftfoot);
        leftfoot.transform.Rotate(fixedRightleftfoot, angleleftfoot, Space.World);

        rightfoot.transform.right = Vector3.Lerp(rightfoot.transform.right, (_poseDataArray[32] - _poseDataArray[28]).normalized, Time.deltaTime * lerpSpeed);
        Vector3 fixedRightrightfoot = rightfoot.transform.right;
        float anglerightfoot = Vector3.SignedAngle(rightfoot.transform.forward, Vector3.down, fixedRightrightfoot);
        rightfoot.transform.Rotate(fixedRightrightfoot, anglerightfoot, Space.World);

        //脚动作到此为止，接下来是左手的动作----------------------------------------------------------------------------

        leftindex_01.transform.right = Vector3.Lerp(leftindex_01.transform.right, (_poseDataArray[39] - _poseDataArray[38]).normalized, Time.deltaTime * lerpSpeed);

        leftindex_02.transform.right = Vector3.Lerp(leftindex_02.transform.right, (_poseDataArray[40] - _poseDataArray[39]).normalized, Time.deltaTime * lerpSpeed);

        leftindex_03.transform.right = Vector3.Lerp(leftindex_03.transform.right, (_poseDataArray[41] - _poseDataArray[40]).normalized, Time.deltaTime * lerpSpeed);

        leftthumb_01.transform.right = Vector3.Lerp(leftthumb_01.transform.right, (_poseDataArray[35] - _poseDataArray[34]).normalized, Time.deltaTime * lerpSpeed);

        leftthumb_02.transform.right = Vector3.Lerp(leftthumb_02.transform.right, (_poseDataArray[36] - _poseDataArray[35]).normalized, Time.deltaTime * lerpSpeed);

        leftthumb_03.transform.right = Vector3.Lerp(leftthumb_03.transform.right, (_poseDataArray[37] - _poseDataArray[36]).normalized, Time.deltaTime * lerpSpeed);

        leftmiddle_01.transform.right = Vector3.Lerp(leftmiddle_01.transform.right, (_poseDataArray[43] - _poseDataArray[42]).normalized, Time.deltaTime * lerpSpeed);

        leftmiddle_02.transform.right = Vector3.Lerp(leftmiddle_02.transform.right, (_poseDataArray[44] - _poseDataArray[43]).normalized, Time.deltaTime * lerpSpeed);

        leftmiddle_03.transform.right = Vector3.Lerp(leftmiddle_03.transform.right, (_poseDataArray[45] - _poseDataArray[44]).normalized, Time.deltaTime * lerpSpeed);

        leftring_01.transform.right = Vector3.Lerp(leftring_01.transform.right, (_poseDataArray[47] - _poseDataArray[46]).normalized, Time.deltaTime * lerpSpeed);

        leftring_02.transform.right = Vector3.Lerp(leftring_02.transform.right, (_poseDataArray[48] - _poseDataArray[47]).normalized, Time.deltaTime * lerpSpeed);

        leftring_03.transform.right = Vector3.Lerp(leftring_03.transform.right, (_poseDataArray[49] - _poseDataArray[48]).normalized, Time.deltaTime * lerpSpeed);

        leftpinky_01.transform.right = Vector3.Lerp(leftpinky_01.transform.right, (_poseDataArray[51] - _poseDataArray[50]).normalized, Time.deltaTime * lerpSpeed);

        leftpinky_02.transform.right = Vector3.Lerp(leftpinky_02.transform.right, (_poseDataArray[52] - _poseDataArray[51]).normalized, Time.deltaTime * lerpSpeed);

        leftpinky_03.transform.right = Vector3.Lerp(leftpinky_03.transform.right, (_poseDataArray[53] - _poseDataArray[52]).normalized, Time.deltaTime * lerpSpeed);

        //左手动作到此为止，接下来是右手的动作----------------------------------------------------------------------------

        rightindex_01.transform.right = Vector3.Lerp(rightindex_01.transform.right, (_poseDataArray[60] - _poseDataArray[59]).normalized, Time.deltaTime * lerpSpeed);

        rightindex_02.transform.right = Vector3.Lerp(rightindex_02.transform.right, (_poseDataArray[61] - _poseDataArray[60]).normalized, Time.deltaTime * lerpSpeed);

        rightindex_03.transform.right = Vector3.Lerp(rightindex_03.transform.right, (_poseDataArray[62] - _poseDataArray[61]).normalized, Time.deltaTime * lerpSpeed);

        rightthumb_01.transform.right = Vector3.Lerp(rightthumb_01.transform.right, (_poseDataArray[56] - _poseDataArray[55]).normalized, Time.deltaTime * lerpSpeed);

        rightthumb_02.transform.right = Vector3.Lerp(rightthumb_02.transform.right, (_poseDataArray[57] - _poseDataArray[56]).normalized, Time.deltaTime * lerpSpeed);

        rightthumb_03.transform.right = Vector3.Lerp(rightthumb_03.transform.right, (_poseDataArray[58] - _poseDataArray[57]).normalized, Time.deltaTime * lerpSpeed);

        rightmiddle_01.transform.right = Vector3.Lerp(rightmiddle_01.transform.right, (_poseDataArray[64] - _poseDataArray[63]).normalized, Time.deltaTime * lerpSpeed);

        rightmiddle_02.transform.right = Vector3.Lerp(rightmiddle_02.transform.right, (_poseDataArray[65] - _poseDataArray[64]).normalized, Time.deltaTime * lerpSpeed);

        rightmiddle_03.transform.right = Vector3.Lerp(rightmiddle_03.transform.right, (_poseDataArray[66] - _poseDataArray[65]).normalized, Time.deltaTime * lerpSpeed);

        rightring_01.transform.right = Vector3.Lerp(rightring_01.transform.right, (_poseDataArray[68] - _poseDataArray[67]).normalized, Time.deltaTime * lerpSpeed);

        rightring_02.transform.right = Vector3.Lerp(rightring_02.transform.right, (_poseDataArray[69] - _poseDataArray[68]).normalized, Time.deltaTime * lerpSpeed);

        rightring_03.transform.right = Vector3.Lerp(rightring_03.transform.right, (_poseDataArray[70] - _poseDataArray[69]).normalized, Time.deltaTime * lerpSpeed);

        rightpinky_01.transform.right = Vector3.Lerp(rightpinky_01.transform.right, (_poseDataArray[72] - _poseDataArray[71]).normalized, Time.deltaTime * lerpSpeed);

        rightpinky_02.transform.right = Vector3.Lerp(rightpinky_02.transform.right, (_poseDataArray[73] - _poseDataArray[72]).normalized, Time.deltaTime * lerpSpeed);

        rightpinky_03.transform.right = Vector3.Lerp(rightpinky_03.transform.right, (_poseDataArray[74] - _poseDataArray[73]).normalized, Time.deltaTime * lerpSpeed);
    }

    public float AngleBetweenVectorAndPlane(Vector3 v, Vector3 planeNormal)
    {
        // Normalize the vectors
        v.Normalize();
        planeNormal.Normalize();

        // Calculate the dot product
        float dot = Vector3.Dot(v, planeNormal);

        // Calculate the angle in radians
        float angle = Mathf.Acos(dot);

        // Convert radians to degrees
        angle *= Mathf.Rad2Deg;

        // Determine the angle sign based on the direction of the vector relative to the plane's normal
        if (dot < 0)
        {
            angle = -angle;
        }

        return angle;
    }

    public static Vector3 ProjectPointOnPlane(Vector3 point, Vector3 planeNormal, Vector3 planePoint)
    {
        planeNormal.Normalize(); // 确保法线是单位向量
        var distance = Vector3.Dot(point - planePoint, planeNormal); // 从点到平面的距离
        return point - (planeNormal * distance); // 从点到平面的投影点
    }

}

