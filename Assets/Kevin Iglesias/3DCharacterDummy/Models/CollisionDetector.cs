using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CollisionDetector : MonoBehaviour
{
    public static bool isColliding = false;

    public string touchObstacleTag;
    public string touchPincherTag;
    public bool hasTouchedObstacle = false;

    public Transform[] armJoint; // ��������Ϸ����
    private Quaternion[] rotation; // ���ڴ洢��ʼλ�õı���

    private void Start()
    {
        // �� Start �����л�ȡ��ʼλ��
        rotation = new Quaternion[armJoint.Length]; // ����һ���µ� Quaternion ����
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

            // ����ײ����ʱ����λ��
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
