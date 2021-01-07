using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;



public class KeyInput_half_joint : MonoBehaviour
{

    public float speed = 30.0f;


    private ArticulationBody articulation;

    // Start is called before the first frame update
    void Start()
    {
        articulation = GetComponent<ArticulationBody>();
    }

    // Update is called once per frame
    void Update()
    {
        float rotationChange;
        float rotationGoal = CurrentPrimaryAxisRotation();
        if (Input.GetKey(KeyCode.P))
        {
            rotationChange = speed * Time.fixedDeltaTime;
            rotationGoal += rotationChange;
            RotateTo(rotationGoal);
            print("P");
        }
        else if (Input.GetKey(KeyCode.O))
        {
            rotationChange = speed * Time.fixedDeltaTime;
            rotationGoal -= rotationChange;
            RotateTo(rotationGoal);
            print("O");
        }

    }

    float CurrentPrimaryAxisRotation()
    {
        float currentRotationRads = articulation.jointPosition[0];
        float currentRotation = Mathf.Rad2Deg * currentRotationRads;
        return currentRotation;
    }

    void RotateTo(float primaryAxisRotation)
    {
        var drive = articulation.xDrive;
        drive.target = primaryAxisRotation;
        articulation.xDrive = drive;
    }
}
