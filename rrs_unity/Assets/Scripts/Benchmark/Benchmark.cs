using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using RRS.Tools.Network;
using RRS.Tools.Protobuf;

public class Benchmark : MonoBehaviour
{
    //Benchmark
    //State of the art Methods
    //ROS#​
    //ROS2​
    //Net2​

    //Test Cases ​
    //1)‌ Network properties(Packet lost, Delay)​
    //2) Unity Performance Analysis(FPS, RAM, CPU,...)​
    //3) Latency(Serialization/ Deserialization)​
    //4)‌‌ Overal Latency with network
    //5) Bandwidth​(Packet Size)

    ///Scenario​
    //1)‌ One instance(Raw Data [Image,‌Pointcloud, nested objects,...])​
    //2)‌ Pick and Place​
    //3) Multi-Instance(1,2,10,100)

    //Net2
    public ColorCameraOpenGL color_camera;
    public ColorCameraOpenGL depth_camera;
    public IMU imu;
    public Lidar lidar;
    bool inited = false;

    Net2.Net2HandlerPublisher publisher_camera_rgb_net2;
    Net2.Net2HandlerPublisher publisher_camera_depth_net2;
    Net2.Net2HandlerPublisher publisher_imu_net2;
    Net2.Net2HandlerPublisher publisher_lidar_net2;

    //Unity Benchamark Report

    //ROS2


    //ROS#


    void init()
    {
        //Initialization
        publisher_lidar_net2 = Net2.Publisher("lidar");
        publisher_camera_rgb_net2 = Net2.Publisher("camera_color");
        publisher_camera_depth_net2 = Net2.Publisher("camera_depth");
        publisher_imu_net2 = Net2.Publisher("imu");

        color_camera.delegateCameraDataChanged += Color_camera_delegateCameraDataChanged;
        depth_camera.delegateCameraDataChanged += Depth_camera_delegateCameraDataChanged;
        imu.delegateIMUDataChanged += Imu_delegateIMUDataChanged;
        lidar.delegateLidarDataChanged += Lidar_delegateLidarDataChanged;
    }


    private void Color_camera_delegateCameraDataChanged(byte[] buffer)
    {
        publisher_camera_rgb_net2.Send(buffer);
        UnityEngine.Debug.Log("net image publisherd");
    }

    private void Depth_camera_delegateCameraDataChanged(byte[] buffer)
    {
        publisher_camera_depth_net2.Send(buffer);
    }

    private void Imu_delegateIMUDataChanged(byte[] buffer)
    {
        publisher_imu_net2.Send(buffer);
    }

    private void Lidar_delegateLidarDataChanged(byte[] buffer)
    {
        publisher_lidar_net2.Send(buffer);
    }

    // Update is called once per fram
    void Update()
    {
        if (Manager.inited && !inited)
        {
            init();
            inited = true;
        }
    }
}
