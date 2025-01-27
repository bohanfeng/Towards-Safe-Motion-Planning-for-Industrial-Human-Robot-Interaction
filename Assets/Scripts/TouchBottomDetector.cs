using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TouchBottomDetector : MonoBehaviour
{
    public string touchTargetTag;
    public string touchTableTag;
    public bool hasTouchedTarget = false;
    public bool hasTouchedTable = false;


    private void OnCollisionEnter(Collision collision)
    {
        if (collision.transform.gameObject.tag == touchTargetTag)
        {
            Debug.Log("Touch Bottom!");
            hasTouchedTarget = true;
        }

        if (collision.transform.gameObject.tag == touchTableTag)
        {
            //Debug.Log("Touch Table!");
            hasTouchedTable = true;
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        if (collision.transform.gameObject.tag == touchTargetTag)
        {
            hasTouchedTarget = false;
        }

        if (collision.transform.gameObject.tag == touchTableTag)
        {
            hasTouchedTable = false;
        }

    }


}
