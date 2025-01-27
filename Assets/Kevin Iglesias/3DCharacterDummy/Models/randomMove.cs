using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class randomMove : MonoBehaviour
{
    public Transform[] armJoint; // 手臂节点
    public float speed = 5f; // 初始速度
    public float delay = 15f; // 延迟时间

    private void Start()
    {
        StartCoroutine(RotateRandomly());
        StartCoroutine(ChangeSpeed());
    }

    IEnumerator RotateRandomly()
    {
        while (true)
        {
            // 对手臂节点进行旋转
            for (int i = 0; i < armJoint.Length; i++)
            {
                if (armJoint[i] != null)
                {
                    // 随机生成一个旋转角度
                    float rotationAnglex = Random.Range(-90f, 180f);
                    float rotationAngley = Random.Range(-90f, 180f);
                    float rotationAnglez = Random.Range(-90f, 180f);

                    // 创建一个旋转
                    Quaternion rotation = Quaternion.Euler(rotationAnglex, rotationAngley, rotationAnglez);

                    // 平滑地旋转到目标旋转
                    armJoint[i].rotation = Quaternion.RotateTowards(armJoint[i].rotation, rotation, speed * Time.deltaTime);
                }
                else
                {
                    Debug.LogWarning("armJoint[" + i + "] has not been assigned a value in the Unity editor.");
                }
            }

            // 等待一段时间
            yield return new WaitForSeconds(delay);
        }
    }


    IEnumerator ChangeSpeed()
    {
        while (true)
        {
            // 每隔一段时间随机改变速度
            speed = Random.Range(10f, 30f);

            // 等待一段时间
            yield return new WaitForSeconds(10*delay);
        }
    }
}
