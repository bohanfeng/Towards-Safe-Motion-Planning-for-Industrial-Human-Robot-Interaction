using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System;

public class GrabCubeAgent : Agent
{
    public GameObject endEffector;
    public GameObject cube;
    public GameObject robot;
    public GameObject table;
    public GameObject goalBox;
    public GameObject goal;
    public GameObject finger_A;
    public GameObject finger_B;
    //public GameObject obstacle;

    public Transform parent;

    RobotController robotController;
    TouchBottomDetector touchDetector;
    //TouchCubeDetector touchCubeDetector;
    //TablePositionRandomizer tablePositionRandomizer;
    TableTouchDetector tableTouchDetector;
    //ObstacleTouchDetector obstacleTouchDetector;
    PincherController pincherController;
    FingerTouch fingertoucher_A;
    FingerTouch fingertoucher_B;

    bool is_gripped = false;
    bool is_released = false;
    
    //bool is_fixed = false;

    Vector3 init;

    void Start()
    {
        robotController = robot.GetComponent<RobotController>();
        touchDetector = cube.GetComponent<TouchBottomDetector>();
        //touchCubeDetector = cube.GetComponent<TouchCubeDetector>();
        tableTouchDetector = table.GetComponent<TableTouchDetector>();
        pincherController = endEffector.GetComponent<PincherController>();
        //obstacleTouchDetector = obstacle.GetComponent<ObstacleTouchDetector>();
        //tablePositionRandomizer = cube.GetComponent<TablePositionRandomizer>();
        fingertoucher_A = finger_A.GetComponent<FingerTouch>();
        fingertoucher_B = finger_B.GetComponent<FingerTouch>();
    }


    // AGENT

    public override void OnEpisodeBegin()
    {
        //Debug.Log("Reset");
        float[] defaultRotations = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        robotController.ForceJointsToRotations(defaultRotations);
        touchDetector.hasTouchedTarget = false;
        //touchCubeDetector.hasTouchedTarget = false;
        tableTouchDetector.hasTouchedTable = false;
        //pincherController.isTouchingCube = false;
        fingertoucher_A.isTouchingCube = false;
        fingertoucher_B.isTouchingCube = false;
        //obstacleTouchDetector.hasTouchedObstacle = false;
        //tablePositionRandomizer.Move();
        cube.GetComponent<Rigidbody>().isKinematic = false;
        cube.transform.SetParent(parent);
        cube.transform.localPosition = new Vector3(0.455f, 0.778f, 0.097f);
        init = cube.transform.position;
        //obstacle.transform.localPosition = new Vector3(0.327f, 0.843f, -0.0396f);
        //obstacle.transform.localRotation = Quaternion.Euler(90, 90, 0); //= new Vector3(90, 90, 0);
        cube.transform.localRotation = Quaternion.Euler(0, 0, 0);

        is_gripped = false;
        is_released = false;
        //is_fixed = false;

        pincherController.ResetGripToOpen();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (robotController.joints[0].robotPart == null)
        {
            // No robot is present, no observation should be added
            return;
        }

        sensor.AddObservation(is_gripped);

        if (!is_gripped)
        {
            Vector3 cubePosition = cube.transform.position - robot.transform.position;
            sensor.AddObservation(cubePosition);

            //Vector3 endPosition = endEffector.transform.position - robot.transform.position;
            Vector3 endPosition = pincherController.CurrentGraspCenter() - robot.transform.position;
            sensor.AddObservation(endPosition);
            sensor.AddObservation(cubePosition - endPosition);

            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);

            Vector3 goalPosition = goalBox.transform.position - robot.transform.position;
            sensor.AddObservation(goalPosition);

            Vector3 endPosition = endEffector.transform.position - robot.transform.position;
            sensor.AddObservation(endPosition);
            sensor.AddObservation(goalPosition - endPosition);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // move
        for (int jointIndex = 0; jointIndex < actionBuffers.DiscreteActions.Length - 1; jointIndex++)
        {
            RotationDirection rotationDirection = ActionIndexToRotationDirection((int)actionBuffers.DiscreteActions[jointIndex]);
            //robotController.RotateJoint(jointIndex, rotationDirection, false);
            //Debug.Log(jointIndex + "is" + rotationDirection);
        }

        float lowX = goalBox.transform.position.x - (goal.GetComponent<MeshRenderer>().bounds.size.x / 2);
        float upX = goalBox.transform.position.x + (goal.GetComponent<MeshRenderer>().bounds.size.x / 2);
        float lowZ = goalBox.transform.position.z - (goal.GetComponent<MeshRenderer>().bounds.size.z / 2);
        float upZ = goalBox.transform.position.z + (goal.GetComponent<MeshRenderer>().bounds.size.z / 2);
        //Debug.Log(lowX);

        float input_pincher = (int)actionBuffers.DiscreteActions[7];
        //pincherController.gripState = GripStateForInput(input_pincher - 1);

        if (is_gripped)
        //    //&& !(cube.transform.position.x > lowX
        //    //&& cube.transform.position.x < upX
        //    //&& cube.transform.position.z > lowZ
        //    //&& cube.transform.position.z < upZ))
        {
            pincherController.gripState = GripStateForInput(0);
        //    //Debug.Log("Grip is fixed.");
        }
        else
        {
        //    cube.GetComponent<Rigidbody>().isKinematic = false;
        //    cube.transform.SetParent(parent);
            pincherController.gripState = GripStateForInput(input_pincher - 1);
        //    //Debug.Log("Grip is Random.");
        }

        //判断是否碰到障碍物；
        /*if (obstacleTouchDetector.hasTouchedObstacle)
        {
            SetReward(-0.72f);
            EndEpisode();
        }*/


        if (cube.GetComponent<Collider>().bounds.Contains(pincherController.CurrentGraspCenter()) 
            && pincherController.grip > 0.5f 
            && fingertoucher_A.isTouchingCube
            && fingertoucher_B.isTouchingCube
            //&& pincherController.isTouchingCube
            && cube.transform.position.y > 0.8f 
            && !is_gripped)
            //&& !is_fixed)
        {
            is_gripped = true;
            //is_fixed = true;
            Debug.Log("Perfect Cube Gripped");
            cube.GetComponent<Rigidbody>().isKinematic = true;
            cube.transform.SetParent(endEffector.transform);
            cube.transform.localPosition = new Vector3(0f, 13.5f, 0f);
            cube.transform.localRotation = Quaternion.Euler(0f, -90f, -180f);
            AddReward(0.5f);
        }

        // Knocked the cube off the table
        if (cube.transform.position.y < 0.700f)
        {
            AddReward(-0.8f);
            //Debug.Log("Cube is off");
            EndEpisode();
        }


        float appx_lowx = init.x - 0.1f;
        float appx_upx = init.x + 0.1f;
        float appx_lowz = init.z - 0.1f;
        float appx_upz = init.z + 0.1f;

        if ((cube.transform.position.x < appx_lowx
           || cube.transform.position.x > appx_upx
           || cube.transform.position.z < appx_lowz
           || cube.transform.position.z > appx_upz)
           //&& cube.transform.position.y <= 0.778f
           && cube.transform.position.x > parent.position.x
           && !is_gripped)
        {
            //Debug.Log("Wrong move for Cube.");
            AddReward(-0.8f);
            EndEpisode();
        }


        if (!is_released && is_gripped 
            //&& pincherController.grip < 0.4f 
            && cube.transform.position.x > lowX 
            && cube.transform.position.x < upX 
            && cube.transform.position.z > lowZ 
            && cube.transform.position.z < upZ)
        {
            Debug.Log("Cube released in the area!");
            is_released = true;
            is_gripped = false;
            cube.GetComponent<Rigidbody>().isKinematic = false;
            cube.transform.SetParent(parent);
            AddReward(0.8f);
        }

        if (goal.GetComponent<Goal>().goal || touchDetector.hasTouchedTarget)
        {
            Debug.Log("Finished!");
            SetReward(1f);
            EndEpisode();
        }

        if (tableTouchDetector.hasTouchedTable)
        {
            AddReward(-0.8f);
            //Debug.Log("Touch Table");
            EndEpisode();
        }

        // Penalty given each step to encourage agent to finish task quickly.
        //Debug.Log(-1f / MaxStep);
        AddReward(-1f / MaxStep);

        //if (is_gripped)
        //{
        //    //var handRotation = endEffector.transform.rotation.eulerAngles;
        //    float distanceToGoal = Vector3.Distance(endEffector.transform.position, goalBox.transform.position);
        //    var jointHeight = 0f; // This is to reward the agent for keeping high up // max is roughly 3.0f
        //    for (int jointIndex = 0; jointIndex < robotController.joints.Length; jointIndex++)
        //    {
        //        jointHeight += robotController.joints[jointIndex].robotPart.transform.position.y - goalBox.transform.position.y;
        //    }
        //    var reward = -distanceToGoal + jointHeight / 100f;

        //    ////根据转角进一步改变reward大小；
        //    //if ((handRotation.z <= 190 || handRotation.z >= 170) && (handRotation.x >= 350 || handRotation.x <= 10))
        //    //{
        //    //    reward += 2 / 100f;
        //    //}
        //    //else
        //    //{
        //    //    reward += -10 / 100f;
        //    //}
        //    ////Debug.Log(reward);

        //    AddReward(reward * 0.01f);
        //}


        //根据夹爪的转角来判断是否有较好地接触物块；
        //var handRotation = endEffector.transform.rotation.eulerAngles;
        //if (touchDetector.hasTouchedTarget)
        //{
        //    if ((handRotation.z <= 190 || handRotation.z >= 170) && (handRotation.x >= 350 || handRotation.x <= 10))
        //    {
        //        Debug.Log("Great.");
        //        SetReward(1f);
        //    }
        //    else
        //    {
        //        SetReward(-1f);
        //    }

        //EndEpisode();
        //}


        // end episode if we touched the cube
        //if (touchDetector.hasTouchedTarget)
        //{
        //    SetReward(1f);
        //    EndEpisode();
        //}




    }


    // HELPERS

    static public RotationDirection ActionIndexToRotationDirection(int actionIndex)
    {
        return (RotationDirection)(actionIndex - 1);
    }

    static GripState GripStateForInput(float input)
    {
        if (input > 0)
        {
            return GripState.Closing;
        }
        else if (input < 0)
        {
            return GripState.Opening;
        }
        else
        {
            return GripState.Fixed;
        }
    }




}

