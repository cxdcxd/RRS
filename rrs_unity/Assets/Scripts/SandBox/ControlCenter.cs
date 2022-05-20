using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

public class ControlCenter : MonoBehaviour
{   
    //public Button resetButton;
    // Start is called before the first frame update
    
   void Start(){

       Debug.Log("button pressed!");
   }

    // void OnEnable()
    // {
    //     //Register Button Event
    //     resetButton.onClick.AddListener(() => buttonCallBack());
    // }

    public void resetScene()
    {
        // UnityEngine.Debug.Log("Clicked: " + resetButton.name);

        // //Get current scene name
        // string scene = SceneManager.GetActiveScene().name;
        // //Load it
        // SceneManager.LoadScene(scene, LoadSceneMode.Single);

        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
    }

    // void OnDisable()
    // {
    //     //Un-Register Button Event
    //     resetButton.onClick.RemoveAllListeners();
    // }
   

    
}
