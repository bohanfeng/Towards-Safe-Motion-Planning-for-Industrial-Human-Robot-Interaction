//JournalModel
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System;
using Unity.Barracuda;
using System.Threading;
using LitJson;
using System.IO;
using UnityEngine.UI;


public class FinalAgent : Agent
{
    Client client;
    Client_CV client_cv;
    public GameObject client_obj;

    ////////////////////////////////////////////////////////
    // UI Testing
    public Button Pose_Initialization;
    public Button Establish_Connection;
    public Button Enable_Vision;
    public Button Start_Motion;
    public Button Pause_Motion;
    public Button Disconnect;
    public int RL_running = 0; // Flag to define whether RL is enabled
    private float[] Reset_Angle = new float[6]{70.0f,70.0f,-30.0f,70.0f,90.0f,40f};
    public int Reset_running = 0; // Flag to define whether reset is enabled
    public int connect_sign = 0;
    public int detect_sign = 0;
    int C3epiCount = 0;
    int classgripstay = 0;
    public Button End_Episode;

    float IK_RL_shiftDistance = 0.0f; // IK和RL切换的距离阈值
    float RL_LargeActionStep = 0.0f; // RL的动作步长
    float Collide_PenaltyDistance = 0.0f; // 碰撞惩罚距离
    float Collide_PenaltySmallDistance = 0.0f; // 小碰撞惩罚距离
    float DisScale = 0f; // 距离缩放比例

    // 定义状态枚举，替代原来的整数状态变量，增加代码可读性
    public enum ClassSign
    {
        Default = 0, // 默认状态
        Approach = 1, // 靠近状态
        Grip = 2, // 抓取状态
        Lift = 3, // 抬升状态
        Move = 4 // 移动状态
    }
    public ClassSign classSign = ClassSign.Default;

    public GameObject[] armsJoint; // 机械臂关节
    public GameObject endEffector; // 末端执行器
    public GameObject cube; // 目标物体
    public GameObject robot; // 机器人
    public GameObject table; // 桌子
    public GameObject goalBox_s; // 小目标区域
    public GameObject goal_s; // 小目标
    public GameObject finger_A; // 机械手指A
    public GameObject finger_B; // 机械手指B
    public GameObject obstacle2; // 障碍物
    public GameObject TCPpoint; // TCP点（工具中心点）
    public Transform parent; // 父物体，用于重置目标物体位置

    public GameObject RoboticArm; // 机器人手臂
    public GameObject rightHand; // 右手
    public GameObject leftHand; // 左手


    Collider smallCubeCollider; // 小目标物体的碰撞器
    RobotController robotController; // 机器人控制器
    TouchBottomDetector touchBottomDetector; // 底部接触检测器
    TableTouchDetector tableTouchDetector; // 桌子接触检测器
    TouchCubeDetector touchCubeDetector; // 目标物体接触检测器
    ObstacleTouchDetector obstacleTouchDetector2; // 障碍物接触检测器
    PincherController pincherController; // 夹爪控制器
    FingerTouchA fingertoucher_A; // 手指A接触检测器
    FingerTouchB fingertoucher_B; // 手指B接触检测器
    MovementControl obs_control; // 障碍物运动控制


    bool is_gripped = false; // 是否抓取标志
    bool is_released = false; // 是否释放标志

    Vector3 tcpPoint; // TCP点位置
    Vector3 init; // 初始位置
    int stay_step = 0; // 停留步数

    int approach_step = 0; // 靠近步数
    int is_gripped_step = 0; // 抓取步数
    int is_realease_step = 0; // 释放步数
    int C3move_step = 0; // 课程3移动步数


    int classSignMove = 0; // 课程标识移动

    int C1finishCount = 0; // Task 1 finished Count
    int SuccessCount = 0; // Task 3 all finsehed Count 

    private int episodeCount = 0; // 训练周期计数

    private float x = 0f; // 目标物体x位置
    private float z = 0f; // 目标物体z位置
    private float r = 0f; // 目标物体旋转
    int StepCount = 0; // 步数计数

    bool RLcontrol; // 是否采用RL算法控制

    float[] LastEulerAngle = new float[3]; // 上一个欧拉角
    float[] CurrentEulerAngle = new float[3]; // 当前欧拉角
    public GameObject[] roboJoints; // 机器人关节
    private Mesh mesh; // 障碍物的网格
    private Vector3[] vertices; // 网格顶点

    // 生成障碍物的顶点
    void GenerateVertices(Mesh mesh, GameObject obj, float pointInterval)
    {
        Vector3 size = obj.GetComponent<Renderer>().bounds.size; // 获取对象的尺寸
        //Debug.Log("size is " + size.x);
        List<Vector3> verticesList = new List<Vector3>(); // 顶点列表
        verticesList.AddRange(mesh.vertices); // 添加原始网格顶点

        // 根据对象的尺寸生成点
        for (float x = -0.5f; x <= 0.5f; x += pointInterval)
        {
            for (float y = -0.5f; y <= 0.5f; y += 1f)
            {
                for (float z = -0.5f; z <= 0.5f; z += pointInterval)
                {
                    verticesList.Add(new Vector3(x, y, z));
                }
            }
        }
        vertices = verticesList.ToArray(); // 转换为数组
        mesh.vertices = vertices; // 设置网格顶点
        mesh.RecalculateBounds(); // 重新计算边界
    }

    // 在项目启动时运行一次
    public void Awake()
    {
        client = client_obj.GetComponent<Client>();
        client_cv = client_obj.GetComponent<Client_CV>();

        // UI Test
        Pose_Initialization.onClick.AddListener(delegate
        {
            Reset_running = 1;
            RL_running = 0;
            if (connect_sign == 1)
            {
                client.SendMode(2);
                client.SendAngleByte(Reset_Angle);
            }
        }); // Initialize robotic arm angle

        Establish_Connection.onClick.AddListener(delegate
        {
            client.Start1();
            connect_sign = 1;
        });

        Enable_Vision.onClick.AddListener(delegate
        {
            // Comment out when not using CV testing!
            client_cv.StartDetection();
            detect_sign = 1;
        });  

        Start_Motion.onClick.AddListener(delegate
        {
            RL_running = 1;
            Reset_running = 0;

            // Comment out when not using vision-based testing!
            //// If vision detection button is clicked, get the initial position of the cube
            //if (detect_sign == 1)
            //{
            //    Vector3 Cube_pos_relative = new Vector3(client_cv.array[0, 1], 0.015f, -client_cv.array[0, 3]);
            //    // Vector3 Cube_pos_relative = new Vector3(client_cv.array[0,1]+0.038f,0.015f,-client_cv.array[0,3]+0.005f);
            //    Vector3 Cube_pos = Cube_pos_relative + robot.transform.position;
            //    cube.transform.localPosition = Cube_pos;
            //    cube.transform.localRotation = Quaternion.Euler(0, 0, 0);
            //    init = cube.transform.position;
            //}

            // If connected to the robotic arm and using the servo_mode script
            if (connect_sign == 1)
            {
                Debug.Log("Starting data transmission!");
                client.SendMode(3);
            }

        });    

        Pause_Motion.onClick.AddListener(delegate
        {
            Debug.Log("pause");
            RL_running = 0;
            Reset_running = 0;
        });    

        Disconnect.onClick.AddListener(delegate
        {
            try
            {
                client.Close1();
                client_cv.CloseDetection();
                detect_sign = 0;
                connect_sign = 0;
            }
            catch (Exception ex)
            {
                // Handle exception
                Debug.LogError("Exception occurred: " + ex.Message);
            }
        });    

        End_Episode.onClick.AddListener(delegate
        {
            RL_running = 0;
            Reset_running = 0;
            EndEpisode();
        });

        // 获取各个组件
        robotController = robot.GetComponent<RobotController>();
        touchBottomDetector = cube.GetComponent<TouchBottomDetector>();
        touchCubeDetector = cube.GetComponent<TouchCubeDetector>();
        tableTouchDetector = table.GetComponent<TableTouchDetector>();
        pincherController = endEffector.GetComponent<PincherController>();
        fingertoucher_A = cube.GetComponent<FingerTouchA>();
        fingertoucher_B = cube.GetComponent<FingerTouchB>();
        obstacleTouchDetector2 = obstacle2.GetComponent<ObstacleTouchDetector>();
        obs_control = obstacle2.GetComponent<MovementControl>();
        mesh = obstacle2.GetComponent<MeshFilter>().mesh;

        vertices = mesh.vertices;
        GenerateVertices(mesh, obstacle2, 0.5f);

    }

    // 计算机器人关节与障碍物顶点之间的最小距离
    float CalculateMinDistance()
    {
        float minJointDistance = Mathf.Infinity; // 初始化最小关节距离
        float minDistance = Mathf.Infinity; // 初始化最小距离

        // 遍历机器人关节和障碍物顶点，计算最小距离
        for (var i = 0; i < roboJoints.Length; i++)
        {
            for (var j = 0; j < vertices.Length; j++)
            {
                Vector3 worldVertex = obstacle2.transform.TransformPoint(vertices[j]); // 转换为世界坐标
                Vector3 closestOnObstacle = roboJoints[i].GetComponent<Collider>().ClosestPointOnBounds(worldVertex);

                Debug.DrawLine(worldVertex, closestOnObstacle, Color.green);

                float distance = (closestOnObstacle - worldVertex).magnitude; // 计算距离
                if (distance < minJointDistance)
                {
                    minJointDistance = distance;
                }
            }
            // Closest for one joint
            if (minJointDistance < minDistance)
            {
                minDistance = minJointDistance;
            }
            minJointDistance = Mathf.Infinity; // 重置最小关节距离
        }
        //Debug.Log(vertices.Length);
        return minDistance;
    }

    // 每个训练周期开始时运行一次，初始化场景
    public override void OnEpisodeBegin()
    {
        
        InitializeParameters(); // 初始化参数
        ResetScene(); // 重置场景
    }

    //  初始化参数
    private void InitializeParameters()
    {
        // 贝叶斯参数
        IK_RL_shiftDistance = 0.3f; //[0.1,0.3] IK和RL切换的距离阈值
        Collide_PenaltyDistance = 0.2f;// [0.1,0.3]  碰撞惩罚距离
        RL_LargeActionStep = 0.05f;//[0.02,0.05] RL的动作步长
        DisScale = 0.1f; // 距离缩放比例

        episodeCount++; // 增加训练周期计数
        Debug.Log("Episode count=" + episodeCount);
        LastEulerAngle[0] = -180; // 初始化上一个欧拉角
        LastEulerAngle[1] = 0;
        LastEulerAngle[2] = -180;

        is_realease_step = 0; // 初始化释放步数
        approach_step = 0; // 初始化靠近步数
        stay_step = 0; // 初始化停留步数

        //重置检测器状态：全部设为未碰撞
        touchBottomDetector.hasTouchedTarget = false;
        touchBottomDetector.hasTouchedTable = false;
        tableTouchDetector.hasTouchedTable = false;
        fingertoucher_A.isTouchingFingerA = false;
        fingertoucher_B.isTouchingFingerB = false;
        touchCubeDetector.hasTouchedTarget = false;
        obstacleTouchDetector2.hasTouchedObstacle = false;
        cube.GetComponent<Rigidbody>().isKinematic = false;

        is_gripped = false; // 重置抓取状态
        is_released = false; // 重置释放状态
        cube.transform.SetParent(parent); // 重置目标物体父物体

        // 障碍物速度调整
        obs_control.speed = UnityEngine.Random.Range(0.03f, 0.1f);

        Collide_PenaltySmallDistance = Collide_PenaltyDistance / 2; //是大阈值的一半
    }
    
    // 重置场景布局
    private void ResetScene()
    {
        float[] defaultRotations0 = { -250f, -20f, -30f, 70f, 90f, -50f }; // 机械臂默认旋转角度:机械臂转到初始位姿,j1原本是110，但是经过映射后初始转角变为-290，导致机械臂会反向旋转，所以需要提前110-360
        robotController.ForceJointsToRotations(defaultRotations0); // 设置机械臂到默认位置
        pincherController.ResetGripToOpen(); // 重置夹爪为打开状态

        // 随机设置目标物体的位置
        x = UnityEngine.Random.Range(0.4f, 0.5f);
        z = UnityEngine.Random.Range(-0.15f, 0.15f);
        r = 0f;
        cube.transform.localPosition = new Vector3(x, 0.798f, z);
        cube.transform.localRotation = Quaternion.Euler(0, r - 90, 0);
        init = cube.transform.position; // 记录初始位置
        cube.GetComponent<Rigidbody>().velocity = Vector3.zero;

        // 随机设置障碍物的位置
        float ObsZposrandom = UnityEngine.Random.Range(-0.4f, 0.4f);
        obstacle2.transform.localPosition = new Vector3(0.5f, 0.912f, ObsZposrandom);
        obstacle2.transform.localRotation = Quaternion.Euler(0, 0, 0);

        //课程3用到的计数标识符
        StepCount = 0; // 重置步数计数
        approach_step = 0; // 重置靠近步数
        C3epiCount = 0;
        int classgripstay = 0;

        C3move_step = 0; // 课程3移动步数
        classSign = ClassSign.Default;
        classSignMove = 0; // 课程3标识移动
    }

    //收集agent的Observations 观测值添加到sensor，供PPO训练
    public override void CollectObservations(VectorSensor sensor)
    {

        // 收集机器人手臂与左右手的距离
        float distanceToRight = Vector3.Distance(RoboticArm.transform.position, rightHand.transform.position);
        float distanceToLeft = Vector3.Distance(RoboticArm.transform.position, leftHand.transform.position);
        sensor.AddObservation(distanceToRight);
        sensor.AddObservation(distanceToLeft);

        // 根据当前课程状态，收集不同的观测值
        switch (classSign)
        {
            case ClassSign.Default:
                CollectDefaultObservations(sensor); // 收集默认状态的观测值
                break;
            case ClassSign.Grip:
                CollectGripObservations(sensor); // 收集抓取状态的观测值
                break;
            case ClassSign.Lift:
                CollectLiftObservations(sensor); // 收集抬升状态的观测值
                break;
                // 可根据需要添加其他情况
        }
    }

    // 收集默认状态的观测值
    private void CollectDefaultObservations(VectorSensor sensor)
    {
        sensor.AddObservation(approach_step); // 靠近步数
        Vector3 tcpPointPos = TCPpoint.GetComponent<Transform>().position - robot.transform.position;
        sensor.AddObservation(tcpPointPos); // TCP点位置
        Vector3 initPos = cube.transform.position - robot.transform.position;
        sensor.AddObservation(initPos); // 目标物体位置

        float noiseObstacleZ = UnityEngine.Random.Range(-0.02f, 0.02f);
        float distanceToCubeSurface = OriginDistanceDet() + noiseObstacleZ; // 到障碍物表面的距离
        sensor.AddObservation(distanceToCubeSurface);
    }

    public float OriginDistanceDet()
    {
        Vector3 TCPpositionInCubeSpace = obstacle2.transform.InverseTransformPoint(tcpPoint);
        Vector3 scale = obstacle2.transform.localScale / 2;
        // 将TCPpositionInCubeSpace中的每个坐标限制在立方体的范围内
        Vector3 nearestPointOnCubeInCubeSpace = new Vector3(
            Mathf.Clamp(TCPpositionInCubeSpace.x, -scale.x, scale.x),
            Mathf.Clamp(TCPpositionInCubeSpace.y, -scale.y, scale.y),
            Mathf.Clamp(TCPpositionInCubeSpace.z, -scale.z, scale.z)
        );

        //// 转换回世界坐标
        Vector3 nearestPointOnCubeInWorldSpace = obstacle2.transform.TransformPoint(nearestPointOnCubeInCubeSpace);

        //// 计算距离
        float distanceToCubeSurface = Vector3.Distance(tcpPoint, nearestPointOnCubeInWorldSpace);
        return (distanceToCubeSurface);
    }

    // 收集抓取状态的观测值
    private void CollectGripObservations(VectorSensor sensor)
    {
        sensor.AddObservation(stay_step); // 停留步数
        Vector3 tcpPointPos = TCPpoint.GetComponent<Transform>().position - robot.transform.position;
        sensor.AddObservation(tcpPointPos); // TCP点位置
        Vector3 initPos = cube.transform.position - robot.transform.position;
        sensor.AddObservation(initPos); // 目标物体位置
        sensor.AddObservation(0); // 固定值，便于区分不同状态
    }

    // 收集抬升状态的观测值
    private void CollectLiftObservations(VectorSensor sensor)
    {
        Vector3 tcpPointPos = TCPpoint.GetComponent<Transform>().position - robot.transform.position;
        sensor.AddObservation(tcpPointPos); // TCP点位置
        var goalPos = new Vector3(goal_s.transform.position.x - robot.transform.position.x, 0.777f, goal_s.transform.position.z - robot.transform.position.z);
        sensor.AddObservation(C3move_step); // 移动步数
        sensor.AddObservation(goalPos); // 目标位置
        sensor.AddObservation(is_realease_step); // 释放步数
    }
  
    // 处理agent接收到的动作
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        robotController.RotateJoint(6,0,0,false);
        if (detect_sign == 1 && client_cv.array[1,0] == 2)
        {
            Vector3 Obstacle_pos_relative = new Vector3(0.5f, 0.892f-robot.transform.position.y, -client_cv.array[1, 3]);
            //Vector3 Obstacle_pos_relative = new Vector3(client_cv.array[1,1],client_cv.array[1,2]+0.005f,-client_cv.array[1,3]);
            Vector3 Obstacle_pos = Obstacle_pos_relative+robot.transform.position;
            obstacle2.transform.position = Obstacle_pos;
            obstacle2.transform.eulerAngles = new Vector3(0.0f,0.0f,0.0f);
        }


        Debug.Log("Button parameter: "+ RL_running+" "+Reset_running);

        // 获取curriculumDetect参数的值（训练文件中设置为3）
        int curriculumDetectValue = (int) Academy.Instance.EnvironmentParameters.GetWithDefault("curriculumDetect", 0);
        // 根据课程编号执行不同的逻辑
        if (curriculumDetectValue == 0)  //parameter selection 3 for training 0 for running
        {   
            if(RL_running == 1 && Reset_running == 0)
            {
                Debug.Log("begin with"+ RL_running+" "+Reset_running);
                bool IsEnd = client.IsFinished;
                C3epiCount++;

                float distanceToCubeSurface = OriginDistanceDet(); // 计算到障碍物表面的距离

                // float test_line = CalculateMinDistance();

                float[] angles = new float[6]; // 存储关节角度

                // IK有无解的bool
                bool AllNAN; // 检查是否所有角度都为NAN
                if(IsEnd)
                {
                    // 根据距离判断使用IK还是RL控制
                    if (distanceToCubeSurface <= IK_RL_shiftDistance)
                    {
                        RLcontrol = true; // 使用RL控制

                        // 变步长x y z rx ry rz动作空间
                        // 计算TCP点到障碍物的距离
                        float distanceTcpToCube = Vector3.Distance(TCPpoint.GetComponent<Transform>().position, cube.transform.position);
                        // 获取当前TCP点相对于机器人的坐标
                        Vector3 cur_coor = TCPpoint.GetComponent<Transform>().position - robot.transform.position;
                        float[] goal_pos = { cur_coor.x, cur_coor.y, cur_coor.z };

                        // 获取动作空间中的连续动作
                        float actionX, actionY, actionZ, rx, ry, rz;
                        actionX = RL_LargeActionStep * Mathf.Clamp(actionBuffers.ContinuousActions[0], -1f, 1f);
                        actionY = RL_LargeActionStep * Mathf.Clamp(actionBuffers.ContinuousActions[1], -1f, 1f);
                        actionZ = RL_LargeActionStep * Mathf.Clamp(actionBuffers.ContinuousActions[2], -1f, 1f);
                        rx = 1f * Mathf.Clamp(actionBuffers.ContinuousActions[3], -1f, 1f);
                        ry = 1f * Mathf.Clamp(actionBuffers.ContinuousActions[4], -1f, 1f);
                        rz = 1f * Mathf.Clamp(actionBuffers.ContinuousActions[5], -1f, 1f);

                        // 更新当前的欧拉角度
                        CurrentEulerAngle[0] = (LastEulerAngle[0] + rx);
                        CurrentEulerAngle[1] = (LastEulerAngle[1] + ry);
                        CurrentEulerAngle[2] = (LastEulerAngle[2] + rz);

                        // 更新目标位置
                        goal_pos[0] += actionX;
                        goal_pos[1] += actionY;
                        goal_pos[2] += actionZ;

                        // 获取当前关节角度
                        float[] cur_rot = new float[6];
                        for (int idx = 0; idx < 6; idx++)
                        {
                            cur_rot[idx] = robotController.GetCurrentJointRotations()[idx];
                        }

                        // 计算逆运动学解
                        InverseKin.solution mysolution = new InverseKin.solution();
                        mysolution = InverseKin.Inv_calculator(goal_pos, cur_rot, CurrentEulerAngle);
                        angles = mysolution.angles; // 获取解的角度
                        AllNAN = mysolution.allnan; // 检查是否所有角度都为NAN

                        // 更新上一次的欧拉角度
                        LastEulerAngle = CurrentEulerAngle;

                        // 移动UI显示到新的角度
                        UIMove(angles);
                    }

                    else if (distanceToCubeSurface >= IK_RL_shiftDistance) 
                    {
                        RLcontrol = false; // 使用IK控制
                        //Debug.Log("IK control");

                        //获取机械臂当前转角
                        float[] cur_rot = new float[6];
                        for (int idx = 0; idx < 6; idx++)
                        {
                            cur_rot[idx] = robotController.GetCurrentJointRotations()[idx];
                        }

                        // 根据需求调整当前关节角度
                        cur_rot[0] = -cur_rot[0] - 180;
                        cur_rot[1] = cur_rot[1] + 90;
                        cur_rot[4] = -cur_rot[4] + 180;


                        // 定义目标点坐标goal_pos，并设置为目标物体与机械臂基座的相对坐标: 目标点坐标=target_pos 与j0_0之差
                        float[] goal_pos = new float[3];
                        UnityEngine.Vector3 blockPosition = cube.transform.position;

                        if (classSign == ClassSign.Default || classSign == ClassSign.Approach || classSign == ClassSign.Grip)
                        {
                            UnityEngine.Vector3 j0Position = robot.transform.position;
                            goal_pos[0] = blockPosition.x - j0Position.x;
                            goal_pos[1] = blockPosition.y + 0.03f - j0Position.y; // y轴偏移0.03防止夹爪碰撞
                            goal_pos[2] = blockPosition.z - j0Position.z;
                            // Debug.Log("夹取");
                        }

                        else if (classSign == ClassSign.Lift && classSignMove == 0)
                        {
                            UnityEngine.Vector3 j0Position = robot.transform.position;
                            goal_pos[0] = blockPosition.x - j0Position.x;
                            goal_pos[1] = 0.2f; // 提升到目标物体上方0.2米
                            goal_pos[2] = blockPosition.z - j0Position.z;
                            // Debug.Log("抬高至cube正上方0.2m");
                        }

                        else if (classSign == ClassSign.Lift && classSignMove == 1)
                        {
                            UnityEngine.Vector3 j0Position = robot.transform.position;
                            goal_pos[0] = goal_s.transform.position.x - j0Position.x;
                            goal_pos[1] = 0.15f; // 提升到0.15米高度
                            goal_pos[2] = goal_s.transform.position.z - j0Position.z;
                            // Debug.Log("转运至目标区域");
                        }

                        // 根据当前关节角度和目标点坐标，通过逆运动学求解目标关节角度
                        float[] EulerAngle = new float[3];
                        EulerAngle[0] = -180;
                        EulerAngle[1] = 0;
                        EulerAngle[2] = -180;
                        LastEulerAngle = EulerAngle;
                        InverseKin.solution mysolution = new InverseKin.solution();
                        mysolution = InverseKin.Inv_calculator(goal_pos, cur_rot, EulerAngle);
                        angles = mysolution.angles; // 获取逆运动学解的关节角度
                        AllNAN = mysolution.allnan; // 检查是否所有角度都为NAN
                        // Debug.Log("IK angles=" + angles[0]+"AllNAN = "+AllNAN);
                        

                        // 如果逆运动学没有解，则切换到RL控制
                        if (AllNAN == true)
                        {
                            //Debug.Log("Pure IK no solution!");

                            for (int jointIndex = 0; jointIndex < actionBuffers.ContinuousActions.Length; jointIndex++)
                            {
                                angles[jointIndex] = RL_LargeActionStep * Mathf.Clamp(actionBuffers.ContinuousActions[jointIndex], -1f, 1f);
                            }
                            UIMove(angles); // 使用RL控制移动UI显示
                        }
                        UIMove(angles); // 移动UI显示到新的角度
                    }
                }
                else
                {
                    for (int jointIndex = 0; jointIndex < actionBuffers.DiscreteActions.Length - 1; jointIndex++)
                    {
                        robotController.RotateJoint(jointIndex, 0, 0.0f, false);
                    }

                }

                // 每个步骤给一个小的惩罚，以鼓励代理快速完成任务
                AddReward(-1f / MaxStep);
                StepCount++;

                if (StepCount > 1499 && classSign != ClassSign.Lift)
                {
                    Debug.Log("超时");
                }

                var reward = 0.0f; // 初始化奖励

                tcpPoint = TCPpoint.GetComponent<Transform>().position; // Tool Center Point世界坐标

                //夹取目标区域：cube上方的区域范围
                float cubeUp_lowx = init.x - 0.01f;
                float cubeUp_upx = init.x + 0.01f;
                float cubeUp_lowz = init.z - 0.01f;
                float cubeUp_upz = init.z + 0.01f;
                float cubeUp_lowy = init.y + 0.045f - 0.03f;
                float cubeUp_upy = init.y + 0.045f;
                Vector3 targetpoint = new Vector3(cube.transform.position.x, cube.transform.position.y + 0.045f, cube.transform.position.z);

                //放置目标区域
                float lowX, upX, lowZ, upZ;
                lowX = (float)(goal_s.transform.position.x - 0.05);
                upX = (float)(goal_s.transform.position.x + 0.05);
                lowZ = (float)(goal_s.transform.position.z - 0.05);
                upZ = (float)(goal_s.transform.position.z + 0.05);

                var goal_position = new Vector3(goal_s.transform.position.x, 0.777f, goal_s.transform.position.z);

                // Approach/Grip
                if (classSign != ClassSign.Lift)
                {
                    // Approach
                    if (classSign != ClassSign.Grip)
                    {
                        approach_step += 1;
                        pincherController.ResetGripToOpen();

                        //// Course 1 Tool Center Point还未到达cube,执行第一个课程-靠近（惩罚放在外面）
                        if (tcpPoint.x < cubeUp_upx
                            && tcpPoint.x > cubeUp_lowx
                            && tcpPoint.z < cubeUp_upz
                            && tcpPoint.z > cubeUp_lowz
                            && tcpPoint.y < cubeUp_upy
                            && tcpPoint.y > cubeUp_lowy)
                        {
                            AddReward(1f * (1 + 1 / approach_step));
                            C1finishCount++;
                            //Debug.Log("Task 1 is finished!");

                            //跳转到夹取+保持
                            classSign = ClassSign.Grip;
                        }
                        // Tool Center Point到达目标点，给+1的reward，结束本回合
                        else
                        {
                            classSign = ClassSign.Default;//关节不做限制
                            float distanceToCube = Vector3.Distance(tcpPoint, targetpoint);
                            //机械臂靠近cube的奖励
                            reward += -distanceToCube + 0.3f; //[0,0.3]

                            float min = distanceToCubeSurface;

                            if (min > Collide_PenaltySmallDistance
                                && min < Collide_PenaltyDistance)
                            {
                                reward += -Math.Max(0, 1 - min / Collide_PenaltySmallDistance);
                            }
                            else if (min < Collide_PenaltySmallDistance)
                            {
                                reward += -Math.Max(0, 1 - min / Collide_PenaltySmallDistance) * 2;
                            }


                            SetReward(reward * 0.1f);

                        }

                        // Approach 惩罚

                        //0.夹爪碰到cube
                        if (fingertoucher_A.isTouchingFingerA || fingertoucher_B.isTouchingFingerB)
                        {
                            Debug.Log("Gripper touch cube!");
                            AddReward(-0.5f);
                            EndEpisode();
                        }

                        //2.碰到cube
                        if (cube.transform.position.y < 0.77f
                            || cube.transform.position.y > 0.8f)
                        {
                            //SetReward(-1f);
                            Debug.Log("Knock cube!");
                            //EndEpisode();
                        }

                        //3.碰障碍物

                        if (obstacleTouchDetector2.hasTouchedObstacle)
                        {
                            SetReward(-1f);
                            Debug.Log("Touching obstacle!!!-C1");
                            EndEpisode();
                        }
                    }


                    ////  Task 2 Tool Center Point到达cube指定区域，开始执行第二课程-夹取，限制6个关节
                    else if (classSign == ClassSign.Grip)
                    {
                        //还未闭合夹爪
                        if (is_gripped != true)
                        {
                            is_gripped_step = 0;
                            //闭合夹爪
                            pincherController.grip = 0.1f;
                            is_gripped = true;
                            //将target设为robot子物体
                            cube.GetComponent<Rigidbody>().isKinematic = true;
                            cube.transform.SetParent(endEffector.transform);
                            cube.transform.localPosition = new Vector3(0f, 0.11f, 0f);
                            cube.transform.localRotation = Quaternion.Euler(0f, 0f, 180f);

                        }
                        else if (is_gripped == true)
                        {
                            classgripstay = 1;
                            pincherController.grip = 0.1f;
                            is_gripped_step += 1;
                            //这里RL的训练要改成0，否则这里无法work！！！！，如果是IK+RL，则可以停留30step
                            if (is_gripped_step > 0)
                            {
                                //Debug.Log("C2 is OK!");
                                AddReward(5f);
                                classSign = ClassSign.Lift;
                            }
                        }


                        //2.碰掉cube
                        if (cube.transform.position.y < 0.5)
                        {
                            SetReward(-1f);
                            Debug.Log("Knock cube!");
                            EndEpisode();
                        }

                        //3.碰障碍物

                        if (obstacleTouchDetector2.hasTouchedObstacle)
                        {
                            SetReward(-1f);
                            Debug.Log("Touching obstacle2-C2");
                            EndEpisode();
                        }

                        SetReward(reward * 0.1f);

                    }
                }
                //C3-抬高+转运
                else if (classSign == ClassSign.Lift)
                {
                    // 如果 TCP 的高度超过 0.9，表示抬升完成，进入运输阶段。
                    if (tcpPoint.y > 0.9f)
                    {
                        //Debug.Log("C2tcpPoint.y=" + tcpPoint.y);
                        classSignMove = 1;
                    }
                    float distanceToDestination = Vector3.Distance(tcpPoint, goal_position);
                    //Debug.Log("目标距离="+distanceToDestination);
                    //夹爪一直闭合
                    pincherController.grip = 0.1f;


                    if (tcpPoint.x > lowX
                        && tcpPoint.x < upX
                        && tcpPoint.z > lowZ
                        && tcpPoint.z < upZ
                        && tcpPoint.y < 1.30f
                        && tcpPoint.y > 0.90f)
                    {
                        //打开夹爪，让物体掉落
                        if (is_realease_step == 0)
                        {
                            //打开夹爪
                            pincherController.ResetGripToOpen();
                            //Debug.Log("C3-release-OK");
                            AddReward(5f);
                            is_realease_step++;
                        }
                        //展示物体掉落过程
                        else if (is_realease_step > 0
                                && is_realease_step < 50)
                        {
                            //取消子物体，物体掉落
                            cube.transform.SetParent(null);
                            cube.GetComponent<Rigidbody>().isKinematic = false;
                            reward += ((2 - distanceToDestination) * DisScale + 0.3f);//[0.4,0.5]
                            is_realease_step++;
                        }
                        //结束
                        else if (is_realease_step >= 50)
                        {
                            AddReward(10f);
                            Debug.Log("C3-Finish-End");
                            /////////////////////////////////////////////////////////////////////////////
                            SuccessCount++;
                            Debug.Log("Success Count=" + SuccessCount);
                            ///////////////////////////////////////////////////////////////////////////////////
                            RL_running = 0;
                            Reset_running = 0;
                            

                        }

                    }
                    else //线性奖励+碰撞距离奖励
                    {
                        //靠近奖励
                        reward += ((1 - distanceToDestination) * DisScale + 0.3f);//[0.4,0.5];

                        float min = distanceToCubeSurface;


                        //避障奖励
                        if (min > Collide_PenaltySmallDistance
                            && min < Collide_PenaltyDistance)
                        {
                            reward += -Math.Max(0, 1 - min / Collide_PenaltySmallDistance);
                        }
                        else if (min < Collide_PenaltySmallDistance)
                        {
                            reward += -Math.Max(0, 1 - min / Collide_PenaltySmallDistance) * 2;
                        }
                    }
                    SetReward(reward * 0.1f);

                }

                // 碰到人体
                if (CollisionDetector.isColliding)
                {
                    if (episodeCount <= 250000)
                    {
                        SetReward(-10.0f * 1 / (approach_step + 1));
                        Debug.Log("Touching human body in" + classSign);
                    }
                    else
                    {
                        SetReward(-1.0f);
                        Debug.Log("Touching human body in" + classSign);
                        EndEpisode();
                    }
                }
                else
                {
                    SetReward(1.0f * (1 / (approach_step + 1)));
                }
                //

                if (obstacleTouchDetector2.hasTouchedObstacle)
                {
                    SetReward(-1f);
                    Debug.Log("Touching obstacle2-C3");
                    EndEpisode();
                }

                tcpPoint = TCPpoint.GetComponent<Transform>().position;//TCP锟斤拷锟斤拷锟斤拷锟斤拷                                                                              
                double xTcp = tcpPoint[0];
                double yTcp = tcpPoint[1];
                double zTcp = tcpPoint[2];

                Vector3 tcpPoint2 = TCPpoint.GetComponent<Transform>().position;//TCP2锟斤拷锟斤拷锟轿伙拷锟�1锟�7
                double xTcp2 = tcpPoint2[0];
                double yTcp2 = tcpPoint2[1];
                double zTcp2 = tcpPoint2[2];



                float[] currentAngle = new float[6];
                for (int jointIndex = 0; jointIndex < 6; jointIndex++)
                {
                    currentAngle[jointIndex] = robotController.GetCurrentJointRotations()[jointIndex];
                }
                currentAngle[0] = -currentAngle[0] - 180;
                currentAngle[1] = currentAngle[1] + 90;
                currentAngle[4] = -currentAngle[4] + 180;

                if (IsEnd && connect_sign == 1)
                {
                    client.SendServoAngle(currentAngle);
                    Debug.Log("Send Message!");
                    client.IsFinished = false;
                }


                

                double sgnCollision = 0;//没锟斤拷
                double sgnGrip = 0;//没锟斤拷

                float joint1 = (float)Math.Round(armsJoint[0].transform.localEulerAngles[1], 2);
                float joint2 = (float)Math.Round(armsJoint[1].transform.localEulerAngles[0], 2);
                float joint3 = (float)Math.Round(armsJoint[2].transform.localEulerAngles[1], 2);
                float joint4 = (float)Math.Round(armsJoint[3].transform.localEulerAngles[0], 2);
                float joint5 = (float)Math.Round(armsJoint[4].transform.localEulerAngles[1], 2);
                float joint6 = (float)Math.Round(armsJoint[5].transform.localEulerAngles[1], 2);
                float jointRL1 = -joint1 + 270;
                float jointRL2 = joint2 + 90;
                float jointRL3 = joint3;
                float jointRL4 = joint4;
                float jointRL5 = joint5;
                float jointRL6 = joint6 - 270;
                if (jointRL2 > 265)
                {
                    jointRL2 = jointRL2 - 360;
                }
                else if (jointRL2 < -85)
                {
                    jointRL2 = jointRL2 + 360;
                }
                if (jointRL3 > 175)
                {
                    jointRL3 = jointRL3 - 360;
                }
                else if (jointRL3 < -175)
                {
                    jointRL3 = jointRL3 + 360;
                }
                if (jointRL6 > 360)
                {
                    jointRL6 = jointRL6 - 360;
                }
                else if (jointRL6 < -360)
                {
                    jointRL6 = jointRL6 + 360;
                }

                TCPcoord TCPcoordRL = new TCPcoord(C3epiCount, jointRL1, jointRL2, jointRL3, jointRL4,
                    jointRL5, jointRL6, xTcp, yTcp, zTcp, sgnCollision, sgnGrip);//锟斤拷录RL转锟斤拷
                List<TCPcoord> TCPcoordsRL = new List<TCPcoord>
                {
                    TCPcoordRL
                };
                //每锟斤拷step锟斤拷锟斤拷锟斤拷锟斤拷
                File.AppendAllText(Application.dataPath + "/Json/RL_out.json", JsonMapper.ToJson(TCPcoordsRL));//锟斤拷锟斤拷锟斤拷锟斤拷锟绞碉拷锟斤拷械锟桔碉拷锟斤拷锟斤拷锟届迹
                File.AppendAllText(Application.dataPath + "/Json/RL_out.json", "\r\n");//锟斤拷锟斤拷
                if (classgripstay == 1)
                {
                    for (int i = 0; i < 5; i++)
                    {
                        File.AppendAllText(Application.dataPath + "/Json/RL_out.json", JsonMapper.ToJson(TCPcoordsRL));//锟斤拷锟斤拷锟斤拷锟斤拷锟绞碉拷锟斤拷械锟桔碉拷锟斤拷锟斤拷锟届迹
                        File.AppendAllText(Application.dataPath + "/Json/RL_out.json", "\r\n");//锟斤拷锟斤拷
                    }

                }
                //锟斤拷锟絜pisode锟斤拷锟铰匡拷始锟斤拷锟斤拷删锟斤拷锟斤拷一锟轿碉拷锟斤拷锟斤拷
                if (C3epiCount == 1)
                {
                    if (File.Exists(Application.dataPath + "/Json/RL_out.json"))
                    {
                        File.Delete(Application.dataPath + "/Json/RL_out.json");
                    }
                }
                classgripstay = 0;
            }
            
            else if (RL_running == 0 && Reset_running == 1)
            {   
                float[] angles = new float[6]; // 存储关节角度
                Debug.Log("Reset Running!");
                float[] defaultRotations0 = { -250f, -20f, -30f, 70f, 90f, -50f };
                robotController.ForceJointsToRotations(defaultRotations0);
                for (int idx = 0; idx < 6; idx++)
                {
                    angles[idx] = robotController.GetCurrentJointRotations()[idx];
                }
            }
            else if (RL_running == 0 && Reset_running == 0)
            {
                //float[] angles = new float[6]; // 存储关节角度
                //获取机械臂当前转角
                float[] cur_rot = new float[6];
                for (int idx = 0; idx < 6; idx++)
                {
                    cur_rot[idx] = robotController.GetCurrentJointRotations()[idx];
                }
                Debug.Log("Pause Running!");
                robotController.ForceJointsToRotations(cur_rot);
                // robotController.RotateJoint(6,0,0,false);
            }
        }
    }


    //机械臂逐帧运动
    public void UIMove(float[] UIAngle)
    {
        float rotationGoal_final;
        for (int jointIndex = 0; jointIndex < 6; jointIndex++)
        {
            // 获取当前关节角度并初始化变量
            float current_angle = robotController.GetCurrentJointRotations()[jointIndex];
            float speed_temp = 25;
            float current_angle_modify = current_angle;

            //UI 控制
            rotationGoal_final = UIAngle[jointIndex];

            // 根据关节索引调整目标角度和当前角度
            switch (jointIndex)
            {
                case 0:
                    rotationGoal_final = -rotationGoal_final - 180;
                    current_angle_modify = -current_angle - 180;
                    break;
                case 1:
                    rotationGoal_final = rotationGoal_final - 90;
                    current_angle_modify = current_angle + 90;
                    break;
                case 4:
                    rotationGoal_final = -rotationGoal_final + 180;
                    current_angle_modify = -current_angle + 180;
                    break;
                case 5:
                    current_angle_modify = -current_angle + 180;
                    break;
            }

            // 计算角度差异，确定旋转方向和速度
            RotationDirection rotationDirection = (RotationDirection)(0);
            float threhold = 5.0f;

            if (rotationGoal_final - current_angle >= threhold)
            {
                rotationDirection = (RotationDirection)(1);
            }
            else if (rotationGoal_final - current_angle <= -threhold)
            {
                rotationDirection = (RotationDirection)(-1);
            }
            else if (rotationGoal_final - current_angle >= 0.01 && rotationGoal_final - current_angle < threhold)
            {
                rotationDirection = (RotationDirection)(1);
                speed_temp = (Mathf.Abs(rotationGoal_final - current_angle) - 0.01f) * speed_temp / (threhold - 0.01f);
            }
            else if (rotationGoal_final - current_angle <= -0.01 && rotationGoal_final - current_angle > -threhold)
            {
                rotationDirection = (RotationDirection)(-1);
                speed_temp = (Mathf.Abs(rotationGoal_final - current_angle) - 0.01f) * speed_temp / (threhold - 0.01f);
            }
            else if (Mathf.Abs(rotationGoal_final - current_angle) < 0.01)
            {
                rotationDirection = (RotationDirection)(0);
                speed_temp = 0;
            }
            // 执行旋转操作
            robotController.RotateJoint(jointIndex, rotationDirection, speed_temp, false);
        }
    }
}

