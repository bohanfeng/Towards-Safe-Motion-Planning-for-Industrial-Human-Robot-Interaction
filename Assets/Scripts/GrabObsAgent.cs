using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System;

public class GrabObsAgent : Agent
{
    public GameObject endEffector;
    public GameObject cube;
    public GameObject robot;
    public GameObject table;
    public GameObject goalBox;
    public GameObject goal;
    public GameObject finger_A;
    public GameObject finger_B;
    public GameObject obstacle;

    public Transform parent;

    RobotController robotController;
    TouchBottomDetector touchBottomDetector;
    //TouchCubeDetector touchCubeDetector;
    //TablePositionRandomizer tablePositionRandomizer;
    TableTouchDetector tableTouchDetector;
    TouchCubeDetector touchCubeDetector;
    ObstacleTouchDetector obstacleTouchDetector;
    PincherController pincherController;
    FingerTouchA fingertoucher_A;
    FingerTouchB fingertoucher_B;

    bool is_gripped = false;
    bool is_released = false;

    //bool is_fixed = false;

    Vector3 init;
    public void Awake()
    {
        robotController = robot.GetComponent<RobotController>();
        touchBottomDetector = cube.GetComponent<TouchBottomDetector>();
        touchCubeDetector = cube.GetComponent<TouchCubeDetector>();
        tableTouchDetector = table.GetComponent<TableTouchDetector>();
        pincherController = endEffector.GetComponent<PincherController>();
        fingertoucher_A = cube.GetComponent<FingerTouchA>();
        fingertoucher_B = cube.GetComponent<FingerTouchB>();
        obstacleTouchDetector = obstacle.GetComponent<ObstacleTouchDetector>();
        //tablePositionRandomizer = cube.GetComponent<TablePositionRandomizer>();
    }

    // Update is called once per frame
    public override void OnEpisodeBegin()
    {
        Debug.Log("Reset");
        float[] defaultRotations = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        robotController.ForceJointsToRotations(defaultRotations);
        touchBottomDetector.hasTouchedTarget = false;
        touchBottomDetector.hasTouchedTable = false;
        tableTouchDetector.hasTouchedTable = false;
        //pincherController.isTouchingCube = false;
        fingertoucher_A.isTouchingFingerA = false;
        fingertoucher_B.isTouchingFingerB = false;
        touchCubeDetector.hasTouchedTarget = false;
        obstacleTouchDetector.hasTouchedObstacle = false;
        //tablePositionRandomizer.Move();
        cube.GetComponent<Rigidbody>().isKinematic = false;
        cube.transform.SetParent(parent);
        cube.transform.localPosition = new Vector3(0.455f, 0.778f, 0.097f);
        init = cube.transform.position;
        obstacle.transform.localPosition = new Vector3(0.327f, 0.822f, -0.0396f);
        obstacle.transform.localRotation = Quaternion.Euler(90, 90, 0); //= new Vector3(90, 90, 0);
        cube.transform.localRotation = Quaternion.Euler(0, 0, 0);

        is_gripped = false;
        is_released = false;
        //is_fixed = false;

        pincherController.ResetGripToOpen();
        Debug.Log("1");
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

            // relative obstacle position
            //Vector3 obstaPosition = obstacle.transform.position - robot.transform.position;
            //sensor.AddObservation(obstaPosition);
            Debug.Log("2");
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

            // relative obstacle position
            //Vector3 obstaPosition = obstacle.transform.position - robot.transform.position;
            //sensor.AddObservation(obstaPosition);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // move
        for (int jointIndex = 0; jointIndex < actionBuffers.DiscreteActions.Length - 1; jointIndex++)
        {
            //Debug.Log(actionBuffers.DiscreteActions[jointIndex]);
            RotationDirection rotationDirection = ActionIndexToRotationDirection((int)actionBuffers.DiscreteActions[jointIndex]);
            robotController.RotateJoint(jointIndex, rotationDirection,10f, false);
            //Debug.Log(jointIndex + "is" + rotationDirection);
        }

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

        // Penalty given each step to encourage agent to finish task quickly.
        //Debug.Log(-1f / MaxStep);
        AddReward(-1f / MaxStep);

        if (Academy.Instance.EnvironmentParameters.GetWithDefault("curriculumDetect", 0) == 1)
        {
            if (tableTouchDetector.hasTouchedTable)
            {
                SetReward(-1f);
                EndEpisode();
            }

            // Knocked the cube off the table
            if (cube.transform.position.y < 0.5)
            {
                SetReward(-1f);
                EndEpisode();
            }


            // end episode if we touched the cube
            if (touchCubeDetector.hasTouchedTarget)
            {
                //Debug.Log("1");
                AddReward(0.5f);
                if (cube.GetComponent<Collider>().bounds.Contains(pincherController.CurrentGraspCenter()))
                {
                    AddReward(0.5f);
                    Debug.Log("Great.");
                }
                EndEpisode();
            }


            //detect obstacle
            if (obstacleTouchDetector.hasTouchedObstacle)
            {
                SetReward(-1f);
                EndEpisode();
            }


            //reward
            float distanceToCube = Vector3.Distance(endEffector.transform.position, cube.transform.position); // roughly 0.7f

            var handRotation = endEffector.transform.rotation.eulerAngles;
            var jointHeight = 0f; // This is to reward the agent for keeping high up // max is roughly 3.0f
            for (int jointIndex = 0; jointIndex < robotController.joints.Length; jointIndex++)
            {
                jointHeight += robotController.joints[jointIndex].robotPart.transform.position.y - cube.transform.position.y;
            }
            var reward = -distanceToCube + jointHeight / 100f;


            //根据转角进一步改变reward大小；
            if ((handRotation.z <= 190 || handRotation.z >= 170) && (handRotation.x >= 350 || handRotation.x <= 10))
            {
                reward += 2 / 100f;
            }
            else
            {
                reward += -10 / 100f;
            }

            SetReward(reward * 0.1f);
        }
        else if (Academy.Instance.EnvironmentParameters.GetWithDefault("curriculumDetect", 0) == 2)
        {
            if (cube.GetComponent<Collider>().bounds.Contains(pincherController.CurrentGraspCenter())
            && pincherController.grip > 0.5f
            && fingertoucher_A.isTouchingFingerA
            && fingertoucher_B.isTouchingFingerB
            //&& pincherController.isTouchingCube
            && cube.transform.position.y > 0.8f
            && !is_gripped
            && !is_released)
            //&& !is_fixed)
            {
                is_gripped = true;
                //is_fixed = true;
                Debug.Log("Perfect Cube Gripped");
                cube.GetComponent<Rigidbody>().isKinematic = true;
                cube.transform.SetParent(endEffector.transform);
                cube.transform.localPosition = new Vector3(0f, 13.5f, 0f);
                cube.transform.localRotation = Quaternion.Euler(0f, -90f, -180f);
                AddReward(1f);
                EndEpisode();
            }

            // Knocked the cube off the table
            if (cube.transform.position.y < 0.700f)
            {
                AddReward(-0.8f);
                //Debug.Log("Cube is off");
                EndEpisode();
            }


            float appx_lowx = init.x - 0.05f;
            float appx_upx = init.x + 0.05f;
            float appx_lowz = init.z - 0.05f;
            float appx_upz = init.z + 0.05f;

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

            if (tableTouchDetector.hasTouchedTable)
            {
                AddReward(-0.8f);
                //Debug.Log("Touch Table");
                EndEpisode();
            }

            //detect obstacle
            if (obstacleTouchDetector.hasTouchedObstacle)
            {
                SetReward(-1f);
                EndEpisode();
            }
        }
        else if (Academy.Instance.EnvironmentParameters.GetWithDefault("curriculumDetect", 0) == 3)
        {
            float lowX = goalBox.transform.position.x - (goal.GetComponent<MeshRenderer>().bounds.size.x / 2);
            float upX = goalBox.transform.position.x + (goal.GetComponent<MeshRenderer>().bounds.size.x / 2);
            float lowZ = goalBox.transform.position.z - (goal.GetComponent<MeshRenderer>().bounds.size.z / 2);
            float upZ = goalBox.transform.position.z + (goal.GetComponent<MeshRenderer>().bounds.size.z / 2);
            //Debug.Log(lowX);


            if (cube.GetComponent<Collider>().bounds.Contains(pincherController.CurrentGraspCenter())
            && pincherController.grip > 0.5f
            && fingertoucher_A.isTouchingFingerA
            && fingertoucher_B.isTouchingFingerB
            //&& pincherController.isTouchingCube
            && cube.transform.position.y > 0.8f
            && !is_gripped
            && !is_released)
            //&& !is_fixed)
            {
                is_gripped = true;
                //is_fixed = true;
                Debug.Log("Perfect Cube Gripped");
                cube.GetComponent<Rigidbody>().isKinematic = true;
                cube.transform.SetParent(endEffector.transform);
                cube.transform.localPosition = new Vector3(0f, 13.5f, 0f);
                cube.transform.localRotation = Quaternion.Euler(0f, -90f, -180f);
                AddReward(1f);
            }

            // Knocked the cube off the table
            if (cube.transform.position.y < 0.700f)
            {
                AddReward(-1f);
                //Debug.Log("Cube is off");
                EndEpisode();
            }


            float appx_lowx = init.x - 0.05f;
            float appx_upx = init.x + 0.05f;
            float appx_lowz = init.z - 0.05f;
            float appx_upz = init.z + 0.05f;

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
                && cube.transform.position.z < upZ
                && cube.transform.position.y < 0.9f)
            {
                Debug.Log("Cube released in the area!");
                AddReward(1f);
                EndEpisode();
                //is_released = true;
                //is_gripped = false;
                //cube.GetComponent<Rigidbody>().isKinematic = false;
                //cube.transform.SetParent(parent);              
            }

            if (tableTouchDetector.hasTouchedTable)
            {
                AddReward(-1f);
                //Debug.Log("Touch Table");
                EndEpisode();
            }

            //detect obstacle
            if (obstacleTouchDetector.hasTouchedObstacle)
            {
                SetReward(-1f);
                EndEpisode();
            }
        }
        else
        {
            float lowX = goalBox.transform.position.x - (goal.GetComponent<MeshRenderer>().bounds.size.x / 2);
            float upX = goalBox.transform.position.x + (goal.GetComponent<MeshRenderer>().bounds.size.x / 2);
            float lowZ = goalBox.transform.position.z - (goal.GetComponent<MeshRenderer>().bounds.size.z / 2);
            float upZ = goalBox.transform.position.z + (goal.GetComponent<MeshRenderer>().bounds.size.z / 2);
            //Debug.Log(lowX);


            if (cube.GetComponent<Collider>().bounds.Contains(pincherController.CurrentGraspCenter())
            && pincherController.grip > 0.5f
            && fingertoucher_A.isTouchingFingerA
            && fingertoucher_B.isTouchingFingerB
            //&& pincherController.isTouchingCube
            && cube.transform.position.y > 0.8f
            && !is_gripped
            && !is_released)
            //&& !is_fixed)
            {
                is_gripped = true;
                //is_fixed = true;
                Debug.Log("Perfect Cube Gripped");
                cube.GetComponent<Rigidbody>().isKinematic = true;
                cube.transform.SetParent(endEffector.transform);
                cube.transform.localPosition = new Vector3(0f, 13.5f, 0f);
                cube.transform.localRotation = Quaternion.Euler(0f, -90f, -180f);
                AddReward(1f);
            }

            // Knocked the cube off the table
            if (cube.transform.position.y < 0.700f)
            {
                AddReward(-1f);
                //Debug.Log("Cube is off");
                EndEpisode();
            }


            float appx_lowx = init.x - 0.05f;
            float appx_upx = init.x + 0.05f;
            float appx_lowz = init.z - 0.05f;
            float appx_upz = init.z + 0.05f;

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
                && cube.transform.position.z < upZ
                && cube.transform.position.y < 0.9f)
            {
                Debug.Log("Cube released in the area!");
                is_released = true;
                is_gripped = false;
                cube.GetComponent<Rigidbody>().isKinematic = false;
                cube.transform.SetParent(parent);
                //AddReward(0.8f);
            }

            if (goal.GetComponent<Goal>().goal || touchBottomDetector.hasTouchedTarget)
            {
                Debug.Log("Finished!");
                AddReward(1f);
                EndEpisode();
            }

            if (is_released && touchBottomDetector.hasTouchedTable)
            {
                Debug.Log("Finished Touching Table!");
                AddReward(-1f);
                EndEpisode();
            }

            if (tableTouchDetector.hasTouchedTable)
            {
                AddReward(-0.8f);
                //Debug.Log("Touch Table");
                EndEpisode();
            }

            //detect obstacle
            if (obstacleTouchDetector.hasTouchedObstacle)
            {
                SetReward(-1f);
                EndEpisode();
            }

        }







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

