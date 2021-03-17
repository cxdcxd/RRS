using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BenchmarkManager : MonoBehaviour
{
    public GameObject robot_lmt;
    public GameObject robot_rossharp;
    public GameObject robot_ros2;

    public  enum BenchmarkMode
    { 
     LMT,
     ROSSharp,
     ROS2
    }

    public BenchmarkMode mode = BenchmarkMode.ROSSharp;

    // Start is called before the first frame update
    void Start()
    {
        if ( mode == BenchmarkMode.LMT )
        {
            robot_lmt.SetActive(true);
            robot_rossharp.SetActive(false);
            robot_ros2.SetActive(false);
        }

        if (mode == BenchmarkMode.ROS2)
        {
            robot_lmt.SetActive(false);
            robot_rossharp.SetActive(false);
            robot_ros2.SetActive(true);
        }

        if (mode == BenchmarkMode.ROSSharp)
        {
            robot_lmt.SetActive(false);
            robot_rossharp.SetActive(true);
            robot_ros2.SetActive(false);
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
