using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

public class UDPPoseReceiver : MonoBehaviour
{
    private Thread _receiveThread;
    private UdpClient _client;
    public const int listenPort = 5054;
    public Vector3[] _poseDataArray = new Vector3[33]; // 创建包含33个元素的数组以存储从0到32的数据
    private object _dataLock = new object(); // 用于同步访问_poseDataArray

    // Start is called before the first frame update
    void Start()
    {
        StartReceivingData();
    }

    private void StartReceivingData()
    {
        _receiveThread = new Thread(new ThreadStart(ReceiveData));
        _receiveThread.IsBackground = true;
        _receiveThread.Start();
    }

    private void ReceiveData()
    {
        _client = new UdpClient(listenPort);
        while (true)
        {
            try
            {
                // Receive bytes
                IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
                byte[] data = _client.Receive(ref anyIP);

                // Assuming each string is 4 bytes long
                if (data.Length >= 16) // 4 strings * 4 bytes per string
                {
                    // Extract the strings from the byte array
                    string str1 = Encoding.ASCII.GetString(data, 0, 4).Trim('\0');
                    string str2 = Encoding.ASCII.GetString(data, 4, 4).Trim('\0');
                    string str3 = Encoding.ASCII.GetString(data, 8, 4).Trim('\0');
                    string str4 = Encoding.ASCII.GetString(data, 12, 4).Trim('\0');

                    // Convert strings to float
                    float num1, num2, num3, num4;
                    bool parseSuccess1 = float.TryParse(str1, out num1);
                    bool parseSuccess2 = float.TryParse(str2, out num2);
                    //num2 = num2 / 200;
                    bool parseSuccess3 = float.TryParse(str3, out num3);
                    //num3 = num3 / 1000;
                    bool parseSuccess4 = float.TryParse(str4, out num4);

                    // Check if all conversions were successful
                    if (parseSuccess1 && parseSuccess2 && parseSuccess3 && parseSuccess4)
                    {
                        // Cast num1 to int for use as an index
                        int index = (int)num1;

                        // Ensure the index is within the bounds of the array
                        if (index >= 0 && index < _poseDataArray.Length)
                        {
                            // Create a Vector3 from num2, num3, num4
                            Vector3 pose = new Vector3(num2, num3, num4);

                            // Store it in the array
                            lock (_dataLock)
                            {
                                _poseDataArray[index] = pose;
                            }

                            // Output the data to the Unity console (optional)
                            UnityMainThreadDispatcher.Instance().Enqueue(() =>
                                Debug.Log($"Stored pose at index {index}: {pose}"));
                        }
                    }
                    else
                    {
                        // Handle the error in parsing the float
                        UnityMainThreadDispatcher.Instance().Enqueue(() =>
                            Debug.LogError("Error parsing float values from received strings"));
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogError(e.ToString());
            }
        }
    }

    void OnApplicationQuit()
    {
        // Close the UDP client and stop the thread
        if (_receiveThread != null && _receiveThread.IsAlive)
        {
            _receiveThread.Abort();
        }
        _client?.Close();
    }

    // Example method to access the data safely from another script
    public Vector3? GetPoseData(int index)
    {
        if (index >= 0 && index < _poseDataArray.Length)
        {
            lock (_dataLock)
            {
                return _poseDataArray[index];
            }
        }
        return null;
    }
}