using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObstacleTouchDetector : MonoBehaviour
{
    public string touchObstacleTag;
    public string touchPincherTag;
    public string touchObjectTag;
    public bool hasTouchedObstacle = false;


    private void OnCollisionEnter(Collision collision)
    {
        if (collision.transform.gameObject.tag == touchObstacleTag || collision.transform.gameObject.tag == touchPincherTag|| collision.transform.gameObject.tag == touchObjectTag)
        {
            //Debug.Log("Collision Detected!");
            hasTouchedObstacle = true;
        }

    }

    private void OnCollisionExit(Collision collision)
    {
        if (collision.transform.gameObject.tag == touchObstacleTag)
        {
            hasTouchedObstacle = false;
        }

    }
}
