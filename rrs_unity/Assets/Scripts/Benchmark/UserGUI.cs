using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

public class UserGUI : MonoBehaviour
{
    // Start is called before the first frame update
    public Transform initial_left_marker;
    public Transform initial_right_marker;

    public GameObject right_marker;
    public GameObject left_marker;

    public RobotInferenceController controller;
    public string stringToEdit = "Enter target to pour in grams";
    void Start()
    {
        right_marker.transform.localPosition = initial_right_marker.localPosition + new Vector3(0, 0, 0.080f);
        right_marker.transform.localRotation = initial_right_marker.localRotation;

        left_marker.transform.localPosition = initial_left_marker.localPosition + new Vector3(0, 0, -0.054f);
        left_marker.transform.localRotation = initial_left_marker.localRotation;
    }

    private void OnGUI()
    {
        stringToEdit = GUI.TextArea(new Rect(700, 10, 200, 20), stringToEdit);
        
        if (GUI.Button(new Rect(700, 60, 100, 50),"Init"))
        {
            print("Starting!!!!");
            controller.setTargetToPour(stringToEdit);
            controller.ConfigureAgent(1);
        }

        if (GUI.Button(new Rect(700, 110, 100, 50), "reset"))
        {
            print("Restarting!!!");
            right_marker.transform.localPosition = initial_right_marker.localPosition + new Vector3(0, 0, 0.080f);
            right_marker.transform.localRotation = initial_right_marker.localRotation;
            left_marker.transform.localPosition = initial_left_marker.localPosition + new Vector3(0, 0, -0.054f);
            left_marker.transform.localRotation = initial_left_marker.localRotation;
            controller.ConfigureAgent(-1);
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
