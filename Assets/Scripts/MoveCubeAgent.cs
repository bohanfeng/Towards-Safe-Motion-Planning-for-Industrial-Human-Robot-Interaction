using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System;

public class MoveCubeAgent : Agent
{
    public GameObject endEffector;
    public GameObject cube;
    public GameObject robot;
    public GameObject table;
    public GameObject goalBox;
    public GameObject goal;
    //public GameObject obstacle;

    RobotController robotController;
    TouchBottomDetector touchDetector;
    TouchCubeDetector touchCubeDetector;
    //TablePositionRandomizer tablePositionRandomizer;
    TableTouchDetector tableTouchDetector;
    //ObstacleTouchDetector obstacleTouchDetector;
    PincherController pincherController;

    bool is_gripped = false;
    bool is_released = false;


    void Start()
    {
        robotController = robot.GetComponent<RobotController>();
        touchDetector = cube.GetComponent<TouchBottomDetector>();
        touchCubeDetector = cube.GetComponent<TouchCubeDetector>();
        tableTouchDetector = table.GetComponent<TableTouchDetector>();
        pincherController = endEffector.GetComponent<PincherController>();
        //obstacleTouchDetector = obstacle.GetComponent<ObstacleTouchDetector>();
        //tablePositionRandomizer = cube.GetComponent<TablePositionRandomizer>();
    }


    // AGENT

    public override void OnEpisodeBegin()
    {
        Debug.Log("Reset");
        float[] defaultRotations = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        robotController.ForceJointsToRotations(defaultRotations);
        touchDetector.hasTouchedTarget = false;
        touchCubeDetector.hasTouchedTarget = false;
        tableTouchDetector.hasTouchedTable = false;
        //obstacleTouchDetector.hasTouchedObstacle = false;
        //tablePositionRandomizer.Move();
        cube.transform.localPosition = new Vector3(0.422f, 0.778f, 0f);
        //obstacle.transform.localPosition = new Vector3(0.327f, 0.843f, -0.0396f);
        //obstacle.transform.localRotation = Quaternion.Euler(90, 90, 0); //= new Vector3(90, 90, 0);
        cube.transform.localRotation = Quaternion.Euler(0, 0, 0);

        is_gripped = false;
        is_released = false;

        pincherController.ResetGripToOpen();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (robotController.joints[0].robotPart == null)
        {
            // No robot is present, no observation should be added
            return;
        }

        //// relative cube position
        //Vector3 cubePosition = cube.transform.position - robot.transform.position;
        //sensor.AddObservation(cubePosition);

        //// relative obstacle position
        ////Vector3 obstaPosition = obstacle.transform.position - robot.transform.position;
        ////sensor.AddObservation(obstaPosition);

        //// relative end position
        //Vector3 endPosition = endEffector.transform.position - robot.transform.position;
        //sensor.AddObservation(endPosition);
        //sensor.AddObservation(cubePosition - endPosition);

        if (!is_gripped)
        {
            Vector3 cubePosition = cube.transform.position - robot.transform.position;
            sensor.AddObservation(cubePosition);

            Vector3 endPosition = endEffector.transform.position - robot.transform.position;
            sensor.AddObservation(endPosition);
            sensor.AddObservation(cubePosition - endPosition);
        }
        else
        {
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

        float input_pincher = (int)actionBuffers.DiscreteActions[7];
        pincherController.gripState = GripStateForInput(input_pincher - 1);


        // Penalty given each step to encourage agent to finish task quickly.
        AddReward(-1f / MaxStep);

        //判断是否碰到障碍物；
        /*if (obstacleTouchDetector.hasTouchedObstacle)
        {
            SetReward(-0.72f);
            EndEpisode();
        }*/


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

        if (Academy.Instance.EnvironmentParameters.GetWithDefault("curriculumDetect", 0) == 1)
        {

            // end episode if we touched the cube
            var handRotation = endEffector.transform.rotation.eulerAngles;
            if (touchCubeDetector.hasTouchedTarget)
            {
                AddReward(0.5f);
                //EndEpisode();

                if ((handRotation.z <= 190 || handRotation.z >= 170) && (handRotation.x >= 350 || handRotation.x <= 10))
                {
                    Debug.Log("Great.");
                    AddReward(0.5f);
                }
                else
                {
                    Debug.Log("Bad.");
                    SetReward(-1f);
                }
                //EndEpisode();
            }

            if (touchCubeDetector.hasTouchedTarget)
            {
                EndEpisode();
            }

                //根据夹爪的转角来判断是否有较好地接触物块；
                //var handRotation = endEffector.transform.rotation.eulerAngles;
                //if (touchCubeDetector.hasTouchedTarget)
                //{
                //    if ((handRotation.z <= 190 || handRotation.z >= 170) && (handRotation.x >= 350 || handRotation.x <= 10))
                //    {
                //        Debug.Log("Great.");
                //        SetReward(1f);
                //    }
                //    else
                //     {
                //        SetReward(-1f);
                //    }

                //EndEpisode();
                //}


                //reward
            float distanceToCube = Vector3.Distance(endEffector.transform.position, cube.transform.position); // roughly 0.7f


            var jointHeight = 0f; // This is to reward the agent for keeping high up // max is roughly 3.0f
            for (int jointIndex = 0; jointIndex < robotController.joints.Length; jointIndex++)
            {
                jointHeight += robotController.joints[jointIndex].robotPart.transform.position.y - cube.transform.position.y;
            }
            var reward = -distanceToCube + jointHeight / 100f;

            if (touchCubeDetector.hasTouchedTarget)
            {
                //根据转角进一步改变reward大小；
                if ((handRotation.z <= 190 || handRotation.z >= 170) && (handRotation.x >= 350 || handRotation.x <= 10))
                {
                    reward += 2 / 100f;
                }
                else
                {
                    reward += -10 / 100f;
                }
            }
                
            SetReward(reward * 0.1f);
            
        }
        else
        {
            if (cube.GetComponent<Collider>().bounds.Contains(endEffector.GetComponent<PincherController>().CurrentGraspCenter()) && pincherController.grip > 0.5f && pincherController.fingerA.GetComponent<FingerTouch>().isTouchingCube && pincherController.fingerB.GetComponent<FingerTouch>().isTouchingCube && cube.transform.localPosition.y > 0.780f && !is_gripped)
            {
                is_gripped = true;
                Debug.Log("Cube Gripped");
                AddReward(0.5f);


                //if (Academy.Instance.EnvironmentParameters.GetWithDefault("curriculumGrip", 0) != 0)
                //{
                //    EndEpisode();
                //}
            }

            float lowX = goalBox.transform.localPosition.x - (goal.GetComponent<MeshRenderer>().bounds.size.x / 2);
            float upX = goalBox.transform.localPosition.x + (goal.GetComponent<MeshRenderer>().bounds.size.x / 2);
            float lowZ = goalBox.transform.localPosition.z - (goal.GetComponent<MeshRenderer>().bounds.size.z / 2);
            float upZ = goalBox.transform.localPosition.z + (goal.GetComponent<MeshRenderer>().bounds.size.z / 2);

            if (!is_released && is_gripped && pincherController.grip < 0.4f && cube.transform.localPosition.x > lowX && cube.transform.localPosition.x < upX && cube.transform.localPosition.z > lowZ && cube.transform.localPosition.z < upZ)
            {
                Debug.Log("Cube released in the area!");
                is_released = true;
                AddReward(0.5f);
            }

            if (goal.GetComponent<Goal>().goal || touchDetector.hasTouchedTarget)
            {
                Debug.Log("Finished!");
                AddReward(0.5f);
                EndEpisode();
            }
        }

            

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
