
using System.Collections.Generic;
using UnityEngine;

public class UnityMainThreadDispatcher : MonoBehaviour
{
    private static readonly Queue<System.Action> _executionQueue = new Queue<System.Action>();
    private static UnityMainThreadDispatcher _instance = null;

    public static UnityMainThreadDispatcher Instance()
    {
        if (_instance == null)
        {
            _instance = FindObjectOfType<UnityMainThreadDispatcher>();

            if (_instance == null)
            {
                var gameObject = new GameObject("UnityMainThreadDispatcher");
                _instance = gameObject.AddComponent<UnityMainThreadDispatcher>();
                DontDestroyOnLoad(gameObject);
            }
        }

        return _instance;
    }

    private void Awake()
    {
        if (_instance == null)
        {
            _instance = this;
            DontDestroyOnLoad(this.gameObject);
        }
        else if (_instance != this)
        {
            Destroy(gameObject);
        }
    }

    public void Enqueue(System.Action action)
    {
        lock (_executionQueue)
        {
            _executionQueue.Enqueue(action);
        }
    }

    private void Update()
    {
        lock (_executionQueue)
        {
            while (_executionQueue.Count > 0)
            {
                _executionQueue.Dequeue().Invoke();
            }
        }
    }
}