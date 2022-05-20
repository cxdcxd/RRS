using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ChangeTargetMesh : MonoBehaviour
{
    // Start is called before the first frame update
    public void changeTargetMesh(int i){
       
        print("Type button pressed");


            if(i== 1){

            MugMove.targetName = "cup";
            print("Type1 been pressed!");
            
        }
        else if(i==2){

            MugMove.targetName = "bowl";
            print("Type2 been pressed!");
        }


        
    }

}
