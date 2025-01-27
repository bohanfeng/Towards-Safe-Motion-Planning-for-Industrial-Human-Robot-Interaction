using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class randomMove : MonoBehaviour
{
    public Transform[] armJoint; // �ֱ۽ڵ�
    public float speed = 5f; // ��ʼ�ٶ�
    public float delay = 15f; // �ӳ�ʱ��

    private void Start()
    {
        StartCoroutine(RotateRandomly());
        StartCoroutine(ChangeSpeed());
    }

    IEnumerator RotateRandomly()
    {
        while (true)
        {
            // ���ֱ۽ڵ������ת
            for (int i = 0; i < armJoint.Length; i++)
            {
                if (armJoint[i] != null)
                {
                    // �������һ����ת�Ƕ�
                    float rotationAnglex = Random.Range(-90f, 180f);
                    float rotationAngley = Random.Range(-90f, 180f);
                    float rotationAnglez = Random.Range(-90f, 180f);

                    // ����һ����ת
                    Quaternion rotation = Quaternion.Euler(rotationAnglex, rotationAngley, rotationAnglez);

                    // ƽ������ת��Ŀ����ת
                    armJoint[i].rotation = Quaternion.RotateTowards(armJoint[i].rotation, rotation, speed * Time.deltaTime);
                }
                else
                {
                    Debug.LogWarning("armJoint[" + i + "] has not been assigned a value in the Unity editor.");
                }
            }

            // �ȴ�һ��ʱ��
            yield return new WaitForSeconds(delay);
        }
    }


    IEnumerator ChangeSpeed()
    {
        while (true)
        {
            // ÿ��һ��ʱ������ı��ٶ�
            speed = Random.Range(10f, 30f);

            // �ȴ�һ��ʱ��
            yield return new WaitForSeconds(10*delay);
        }
    }
}
