using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
public class WeightDisplay : MonoBehaviour
{
    Text txt;				//定义静态变量名以用于其他脚本内的引用**

    public string massDisplay;
    
    public GameObject Scale;
	void Start () 
	{
		txt = GameObject.Find("Canvas/Text").GetComponent<Text>();
        txt.text = "Weight:0";	 
         //Debug.Log("Weight:"+massDisplay);

	}

    void Update()
    {

        massDisplay = (Scale.GetComponent<WeightScale>().combinedForce_0 + Scale.GetComponent<WeightScale>().combinedForce_1).ToString("f3") ;
        txt.text = "Weight:" + massDisplay + " g";

        // Debug.Log("Weight:"+massDisplay);
    }

}
