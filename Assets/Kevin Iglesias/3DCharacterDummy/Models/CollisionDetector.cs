using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionDetector : MonoBehaviour
{
    public static bool isColliding = false;

    public string touchObstacleTag;
    public string touchPincherTag;
    public bool hasTouchedObstacle = false;

    public Transform[] armJoint; // 新增的游戏对象
    private Quaternion[] rotation; // 用于存储初始位置的变量

    private void Start()
    {
        // 在 Start 方法中获取初始位置
        rotation = new Quaternion[armJoint.Length]; // 创建一个新的 Quaternion 数组
        for (int i = 0; i < armJoint.Length; i++)
        {
            rotation[i] = armJoint[i].rotation;
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.transform.gameObject.tag == touchObstacleTag || collision.transform.gameObject.tag == touchPincherTag)
        {
            Debug.Log("Collision Detected! isColliding = true");
            hasTouchedObstacle = true;
            isColliding = true;

            // 在碰撞发生时重置位置
            for (int i = 0; i < armJoint.Length; i++)
            {
                armJoint[i].rotation = rotation[i];
            }
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        if (collision.transform.gameObject.tag == touchObstacleTag || collision.transform.gameObject.tag == touchPincherTag)
        {
            hasTouchedObstacle = false;
            isColliding = false;
        }
    }
}
