using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FingerTouchA : MonoBehaviour
{

    public bool isTouchingFingerA = false;
    public string target;
    private void OnTriggerStay(Collider collision)  
    {
        if (collision.gameObject.tag == target)
        {
            //Debug.Log("Finger " + finger + " is touching the cube");
            isTouchingFingerA = true;
        }

    }

    private void OnTriggerExit(Collider collision)
    {
        if (collision.gameObject.tag == target)
        {
            //Debug.Log("Finger " + finger + " is not touching the cube");
            isTouchingFingerA = false;
        }

    }

    public bool isTouching()
    {
        return isTouchingFingerA;
    }

    public void resetIstouching()
    {
        isTouchingFingerA = false;
    }
}
