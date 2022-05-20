using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class ButtonHandler : MonoBehaviour
{
    // [SerializeField] MugMove;
    //public GameObject Mug2;
    public void changeContainer(int i){
       
        print("Type button pressed");

        if(i== 1){

            MugMove.containerName = "type1";
            print("Type1 been pressed!");
            MugMove.typeButtonPressed = true;

        }
        else if(i== 2){

            MugMove.containerName = "type2";
            print("Type2 been pressed!");
            MugMove.typeButtonPressed = true;



        }
        else if (i == 3)
        {

            MugMove.containerName = "type3";
            print("Type3 been pressed!");
            MugMove.typeButtonPressed = true;



        }
        else if (i == 4)
        {

            MugMove.containerName = "type4";
            print("Type4 been pressed!");
            MugMove.typeButtonPressed = true;



        }
        else if (i == 5)
        {

            MugMove.containerName = "type5";
            print("Type5 been pressed!");
            MugMove.typeButtonPressed = true;



        }
        else if (i == 6)
        {

            MugMove.containerName = "type6";
            print("Type6 been pressed!");
            MugMove.typeButtonPressed = true;



        }

    }

}
