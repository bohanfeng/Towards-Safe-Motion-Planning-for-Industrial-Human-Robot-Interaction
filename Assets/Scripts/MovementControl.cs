using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovementControl : MonoBehaviour
{
    public float speed = 0f; // 设置速度为5cm/s，注意Unity中单位是米，所以实际上应该是0.05
    public float maxDistance = 0.4f; // 设置物体可以移动的最大距离为0.4米
    public float waitTime = 0f; // 设置物体在到达目标位置后等待的时间

    private float initialPosition;
    private int direction = 1; // 使用一个方向变量，1代表向前移动，-1代表向后移动
    private float timer = 0.0f; // 计时器

    void Start()
    {
        // 记录初始位置
        initialPosition = transform.position.z; // 使用z
    }

    void Update()
    {

        // 计算新的位置
        float newPosition = transform.position.z + speed * Time.deltaTime * direction; // 使用z轴位置代替x轴位置
        //float noisePosition = newPosition + noiseZ;
        // 如果物体超出了最大距离，改变移动方向
        if (Mathf.Abs(newPosition - initialPosition) > maxDistance)
        {
            // 如果计时器已经达到等待时间，改变移动方向
            if (timer > waitTime)
            {
                direction *= -1;
                timer = 0.0f; // 重置计时器
            }
            else
            {
                timer += Time.deltaTime; // 计时器累加
            }
        }
        else
        {
            // 仅修改物体的z位置
            transform.position = new Vector3(transform.position.x, transform.position.y, newPosition); // 使用z轴位置代替x轴位置
        }
    }
}
