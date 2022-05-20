using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
public class CountDisplay : MonoBehaviour
{
    Text txt;				//定义静态变量名以用于其他脚本内的引用**

    // static public int incup_particles;
    
    public GameObject Empty;
	void Start () 
	{
		txt = GameObject.Find("Canvas/Count").GetComponent<Text>();
        txt.text = "Count:0";	 
        
	}

    void Update()
    {
        int count = Empty.GetComponent<MugMove>().incup_particles;
        if(count>20)
        {
        txt.text = "Count:"+count;
        }
        //Debug.Log("Count:"+count);
    }

}
