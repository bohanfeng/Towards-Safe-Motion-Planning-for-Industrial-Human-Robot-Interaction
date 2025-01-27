using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System;
using System.Threading;

public class Client : MonoBehaviour
{
    private Socket tcpClient;
    // private string serverIP = "192.168.2.103";//服务器ip地址 suohazhandui
    // private string serverIP = "192.168.3.11";// 服务器ip地址 126
    private string serverIP  = "10.5.5.101";  //ip 地址 cab
    // private string serverIP = "192.168.1.110";//服务器ip地址 寝室wifi
    // private string serverIP = "10.167.209.98";//服务器ip地址 SJTU wifi
    private int serverPort = 5000;//端口号
    private Thread receiveThread;
    string[] storage1 = new string[10];
    public bool IsFinished = true;

    public void Start1()
    {
        //1、创建socket
        tcpClient = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

        //2、建立一个连接请求
        IPAddress iPAddress = IPAddress.Parse(serverIP);
        EndPoint endPoint = new IPEndPoint(iPAddress, serverPort);
        tcpClient.Connect(endPoint);
        Debug.Log("请求服务器连接");
        receiveThread = new Thread(ReceiveData);
        receiveThread.Start();
        Debug.Log("接收线程开启");
    }

    public void Close1()
    {
        tcpClient.Close();
        Debug.Log("Disconnected with robot.");
    }

    private void ReceiveData()
    {
        while (true)
        {
            Debug.Log("尝试接收信号");
            try
            {
                byte[] receiveBuffer = new byte[1024];
                int receiveLength = tcpClient.Receive(receiveBuffer); // 接收数据
                string recieveString = Encoding.UTF8.GetString(receiveBuffer, 0, receiveLength);
                Debug.Log("接收到的信号：" + recieveString);
                if (recieveString=="1")
                {
                    IsFinished = true;
                }
            }
            catch (Exception e)
            {
                Debug.LogError("接收数据错误: " + e.Message);
            }
        }
    }


    public string Angle_input()
    {
        byte[] data = new byte[1024];
        int length = tcpClient.Receive(data);
        string message = Encoding.UTF8.GetString(data, 0, length);
        return message;
    }

    // 发送Json文件
    public void SendPathBytes(float[,] array)
    {
        // int mode = 1;
        int rows = array.GetLength(0);
        int columns = array.GetLength(1);
        // string modeString = mode.ToString();
        string rowString = rows.ToString();
        string colString = columns.ToString();

        // byte [] dataMode = Encoding.UTF8.GetBytes(modeString);
        // tcpClient.Send(dataMode);
        // System.Threading.Thread.Sleep(600);

        byte [] dataRow = Encoding.UTF8.GetBytes(rowString);
        tcpClient.Send(dataRow);
        System.Threading.Thread.Sleep(600);

        byte [] dataColm = Encoding.UTF8.GetBytes(colString);
        tcpClient.Send(dataColm);
        System.Threading.Thread.Sleep(600);

        // 计算数组中所有元素的总字节数
        int length = array.GetLength(0) * array.GetLength(1) * sizeof(float);

        // 创建一个字节数组
        byte[] bytes = new byte[length];

        // 将数组中的元素拷贝到字节数组中
        Buffer.BlockCopy(array, 0, bytes, 0, length);

        tcpClient.Send(bytes);
    }

    // 发送角度数组（UI界面）
    public void SendAngleByte(float[] UIInfo)
    {
        // int mode = 2;
        // string modeString = mode.ToString();
        // byte [] dataMode = Encoding.UTF8.GetBytes(modeString);
        // tcpClient.Send(dataMode);
        // System.Threading.Thread.Sleep(600);
        string pos_str = string.Join(" ", UIInfo);
        byte[] data2 = Encoding.UTF8.GetBytes(pos_str);//将字符串转成字节数组
        tcpClient.Send(data2);
    }

    public void SendServoAngle(float[] UIInfo)
    {

        // int mode = 3;
        // string modeString = mode.ToString();
        // byte [] dataMode = Encoding.UTF8.GetBytes(modeString);
        // tcpClient.Send(dataMode);
        // System.Threading.Thread.Sleep(600);
        string pos_str = string.Join(" ", UIInfo);
        byte[] data2 = Encoding.UTF8.GetBytes(pos_str);//将字符串转成字节数组
        tcpClient.Send(data2);
    }

    public void SendMode(int mode)
    {
        Debug.Log("send mode num=" + mode);
        string index = "Mode";
        byte [] indexMode = Encoding.UTF8.GetBytes(index);
        tcpClient.Send(indexMode);
        System.Threading.Thread.Sleep(500);
        string modeString = mode.ToString();
        byte [] dataMode = Encoding.UTF8.GetBytes(modeString);
        tcpClient.Send(dataMode);
        System.Threading.Thread.Sleep(500);
    }

    public void EpisodeBeginSignal()
    {
        System.Threading.Thread.Sleep(50);
        string index = "BeginEpisode";
        byte [] indexMode = Encoding.UTF8.GetBytes(index);
        tcpClient.Send(indexMode);
        System.Threading.Thread.Sleep(50);
    }

}
