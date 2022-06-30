using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using NVIDIA.Flex;
using System;

public class UserGUI : MonoBehaviour
{
    // Start is called before the first frame update
    public Transform initial_left_marker;
    public Transform initial_right_marker;
    public GameObject right_marker;
    public GameObject left_marker;

    public GameObject cup;
    public GameObject mug_1;
    public GameObject mug_2;
    public GameObject hexagonal_glass;
    public GameObject spiral_mug;

    public FlexContainer water;
    public FlexContainer ink;
    public FlexContainer oil;
    public FlexContainer glycerine;
    public FlexContainer honey;
    public FlexContainer milk;
    public FlexContainer handgel;
    public FlexContainer ketchup;
    public FlexContainer shampoo;

    private GameObject pouringContainer;
    private FlexContainer selectedLiquid;

    public RobotInferenceController controller;
    public string stringToEdit = "Enter target to pour in grams";

    public bool is_markers_visible = false;

    void Start()
    {
        right_marker.transform.position = initial_right_marker.position;
        right_marker.transform.rotation = initial_right_marker.rotation;
        left_marker.transform.position = initial_left_marker.position;
        left_marker.transform.rotation = initial_left_marker.rotation;
    }

    private void OnGUI()
    {
        GUI.skin.label.fontSize = 15;
        stringToEdit = GUI.TextArea(new Rect(20, 10 + 200 , 100, 20), stringToEdit);
        
        if (GUI.Button(new Rect(20, 60 + 200, 100, 50),"Init"))
        {
            print("Starting!!!!");
            controller.setTargetToPour(stringToEdit);
            controller.ConfigureAgent(1);
        }

        if (GUI.Button(new Rect(20, 110 + 200, 100, 50), "reset"))
        {
            controller.ConfigureAgent(-1);
            print("Restarting!!!");

            if (Statics.current_environment == Statics.Environments.Real)
            {
                if (Statics.network_manager_left_arm != null)
                {
                    Statics.network_manager_left_arm.killAll();
                    Statics.network_manager_left_arm = null;
                }

                if (Statics.network_manager_right_arm != null)
                {
                    Statics.network_manager_right_arm.killAll();
                    Statics.network_manager_right_arm = null;
                }

                if (Statics.network_manager_movo_status != null)
                {
                    Statics.network_manager_movo_status.killAll();
                    Statics.network_manager_right_arm = null;
                }

                GC.Collect();
                GC.WaitForPendingFinalizers();

                SceneManager.LoadScene(0);
            }
            else
            {
                right_marker.transform.position = initial_right_marker.position;
                right_marker.transform.rotation = initial_right_marker.rotation;
                left_marker.transform.position = initial_left_marker.position;
                left_marker.transform.rotation = initial_left_marker.rotation;
            }

        }

        // Select Mode: Tele or PourNet
        if (GUI.Button(new Rect(20 + 150, 60 + 200, 100, 50), "Teleoperation"))
        {
            //Statics.current_config.mode = "movo_tele";
            //print(Statics.current_config.mode);
            controller.ConfigureAgent(-1);
            print("Tele");

        }

        if (GUI.Button(new Rect(20 + 150, 110 + 200, 100, 50), "PourNet"))
        {
            //Statics.current_config.mode = "movo";
            //print(Statics.current_config.mode);
            controller.ConfigureAgent(1);
            print("PourNet");

        }

        // Pouring Mugs 
        if (GUI.Button(new Rect(20, 200 + 200, 100, 50), "Conical Glass"))
        {
            pouringContainer = cup;
        }
        if (GUI.Button(new Rect(20, 250 + 200, 100, 50), "Engraved Mug"))
        {
            pouringContainer = mug_1;
        }
        if (GUI.Button(new Rect(20, 300 + 200, 100, 50), "Deep Mug"))
        {
            pouringContainer = mug_2;
        }
        if (GUI.Button(new Rect(20, 350 + 200, 100, 50), "Hexagonal Glass"))
        {
            pouringContainer = hexagonal_glass;
        }
        if (GUI.Button(new Rect(20, 400 + 200, 100, 50), "Spiral Mug"))
        {
            pouringContainer = spiral_mug;
        }

        // Liquid Types
        if (GUI.Button(new Rect(20 + 150, 200 + 200, 100, 25), "Water"))
        {
            selectedLiquid = water;
        }

        if (GUI.Button(new Rect(20 + 150, 225 + 200, 100, 25), "Milk"))
        {
            selectedLiquid = milk;
        }

        if (GUI.Button(new Rect(20 + 150, 250 + 200, 100, 25), "Ink"))
        {
            selectedLiquid = ink;
        }

        if (GUI.Button(new Rect(20 + 150, 275 + 200, 100, 25), "Shampoo"))
        {
            selectedLiquid = shampoo;
        }

        if (GUI.Button(new Rect(20 + 150, 300 + 200, 100, 25), "Oil"))
        {
            selectedLiquid = oil;
        }

        if (GUI.Button(new Rect(20 + 150, 325 + 200, 100, 25), "Handgel"))
        {
            selectedLiquid = handgel;
        }

        if (GUI.Button(new Rect(20 + 150, 350 + 200, 100, 25), "Glycerine"))
        {
            selectedLiquid = glycerine;
        }

        if (GUI.Button(new Rect(20 + 150, 375 + 200, 100, 25), "Ketchup"))
        {
            selectedLiquid = ketchup;
        }

        if (GUI.Button(new Rect(20 + 150, 400 + 200, 100, 25), "Honey"))
        {
            selectedLiquid = honey;
        }

        if (pouringContainer != null)
        {
            controller.setPouringContainer(pouringContainer);
        }
        if (selectedLiquid != null)
        {
            controller.setSelectedLiquid(selectedLiquid);
        }
    }

    // Update is called once per frame
    void Update()
    {
        initial_left_marker.gameObject.GetComponent<Renderer>().enabled = is_markers_visible;
        initial_right_marker.gameObject.GetComponent<Renderer>().enabled = is_markers_visible;
        right_marker.GetComponent<Renderer>().enabled = is_markers_visible;
        left_marker.GetComponent<Renderer>().enabled = is_markers_visible;
    }
}
