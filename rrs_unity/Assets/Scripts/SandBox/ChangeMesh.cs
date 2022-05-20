using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class ChangeMesh : MonoBehaviour
{
    //[SerializeField] MugMove;
    
    public void changeMesh(int i){
       
        print("Type button pressed");


            if(i== 1){

            MugMove.actorName = "cup";
            print("Type1 been pressed!");
            
        }
        else if(i== 2){

            MugMove.actorName = "bowl";
            print("Type2 been pressed!");
        }


        
    }

}