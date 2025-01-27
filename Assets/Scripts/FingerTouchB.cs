using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FingerTouchB : MonoBehaviour
{

    public bool isTouchingFingerB = false;
    public string target;
    private void OnTriggerStay(Collider collision)
    {
        if (collision.gameObject.tag == target)
        {
            //Debug.Log("Finger " + finger + " is touching the cube");
            isTouchingFingerB = true;
        }

    }

    private void OnTriggerExit(Collider collision)
    {
        if (collision.gameObject.tag == target)
        {
            //Debug.Log("Finger " + finger + " is not touching the cube");
            isTouchingFingerB = false;
        }

    }

    public bool isTouching()
    {
        return isTouchingFingerB;
    }

    public void resetIstouching()
    {
        isTouchingFingerB = false;
    }
}
