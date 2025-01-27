using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System;
using System.Threading;

public class Client_CV : MonoBehaviour
{

    private Socket tcpClient;
    // private string serverIP = "192.168.2.100";//服务器ip地址 suohazhandui
    private string serverIP = "10.5.5.109";//服务器ip地址 CAB
    // private string serverIP = "192.168.1.101";//服务器ip地址 寝室wifi
    // private string serverIP = "10.167.209.98";//服务器ip地址 SJTU wifi
    private int serverPort = 9999;//端口号
    private Thread receiveThread;
    int row_num = 4;
    int colm_num = 7;
    public float[,] array = new float[4,7];

    // Start is called before the first frame update
    public void StartDetection()
    {
        //1、创建socket
        tcpClient = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

        //2、建立一个连接请求
        IPAddress iPAddress = IPAddress.Parse(serverIP);
        EndPoint endPoint = new IPEndPoint(iPAddress, serverPort);
        tcpClient.Connect(endPoint);
        Debug.Log("Pose Estimation Initialized.");
        receiveThread = new Thread(ReceivePose);
        receiveThread.Start();
        Debug.Log("CV Connected!");
    }

    public void CloseDetection()
    {
        tcpClient.Close();
        Debug.Log("Disconnected with camera.");
    }

    private void ReceivePose()
    {
        while (true)
        {
            try
            {
                byte[] receiveBuffer = new byte[1024];
                int receiveLength = tcpClient.Receive(receiveBuffer); // 接收数据
                string recieveString = Encoding.UTF8.GetString(receiveBuffer, 0, receiveLength);
                Debug.Log("Recieved Pose String:"+recieveString);
                int ByteSize = row_num * colm_num * sizeof(float);
                Debug.Log("Byte Size = "+ByteSize);
                byte[] data = new byte[ByteSize];
                int length = tcpClient.Receive(data);
                array = GetArray(data, row_num, colm_num);
                Debug.Log("The size of array is "+array.GetLength(0)+"*"+array.GetLength(1));
                
            }
            catch (Exception e)
            {
                Debug.LogError("Pose reception error: " + e.Message);
            }
        }
    }

    private float[,] GetArray(byte[] bytes, int row, int colm)
    {
        // 创建二维数组
        float[,] array = new float[row, colm];

        // 将字节数组中的数据拷贝到二维数组中
        Buffer.BlockCopy(bytes, 0, array, 0, bytes.Length);

        // 返回二维数组
        return array;
    }

    
}
