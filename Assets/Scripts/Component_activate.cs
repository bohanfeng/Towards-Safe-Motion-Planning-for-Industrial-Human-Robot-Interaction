using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Component_activate : MonoBehaviour
{
    public GameObject j0_0;
    public GameObject j1;
    public GameObject j2;
    public GameObject j3;
    public GameObject j4;
    public GameObject j5;
    public GameObject j6_Hand;
    public GameObject j6;
    public GameObject HandE;
    public GameObject Claw1;
    public GameObject Claw2;

    public  void Activate_joint()
    {
        j0_0.GetComponent<ArticulationBody>().enabled = true;
        j1.GetComponent<ArticulationBody>().enabled = true;
        j2.GetComponent<ArticulationBody>().enabled = true;
        j3.GetComponent<ArticulationBody>().enabled = true;
        j4.GetComponent<ArticulationBody>().enabled = true;
        j5.GetComponent<ArticulationBody>().enabled = true;
        j6.GetComponent<ArticulationBody>().enabled = true;
        HandE.GetComponent<ArticulationBody>().enabled = true;
        Claw1.GetComponent<ArticulationBody>().enabled = true;
        Claw2.GetComponent<ArticulationBody>().enabled = true;
    }

    public  void Deactivate_joint()
    {
        j0_0.GetComponent<ArticulationBody>().enabled = true;
        j1.GetComponent<ArticulationBody>().enabled = false;
        j2.GetComponent<ArticulationBody>().enabled = false;
        j3.GetComponent<ArticulationBody>().enabled = false;
        j4.GetComponent<ArticulationBody>().enabled = false;
        j5.GetComponent<ArticulationBody>().enabled = false;
        j6.GetComponent<ArticulationBody>().enabled = false;
        HandE.GetComponent<ArticulationBody>().enabled = false;
        Claw1.GetComponent<ArticulationBody>().enabled = false;
        Claw2.GetComponent<ArticulationBody>().enabled = false;
    }
}
