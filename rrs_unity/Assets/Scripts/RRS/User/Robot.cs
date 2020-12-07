using ProtoBuf;
using RRS.Tools.Network;
using RRS.Tools.Protobuf;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class Robot : MonoBehaviour
{
    Net2.Net2HandlerPublisher publisher_lidar_1;
    Net2.Net2HandlerPublisher publisher_camera_color;
   
    //SUBS
    Net2.Net2HandlerSubscriber subscriber_cmd_vel;

    public Lidar lidar;
    public ColorCamera color_camera;

    bool inited = false;

    public Vector3 speed = new Vector3();

    // Start is called before the first frame update
    void Start()
    {
        
    }

    void init()
    {
        publisher_lidar_1 = Net2.Publisher("lidar_1");
        publisher_camera_color = Net2.Publisher("camera_color");
     
        subscriber_cmd_vel = Net2.Subscriber("rrs_ros-cmd_vel");
        subscriber_cmd_vel.delegateNewData += Subscriber_cmd_vel_delegateNewData;

        //Sensors
        color_camera.delegateCameraDataChanged += Color_camera_delegateCameraDataChanged;
        lidar.delegateLidarDataChanged += Lidar_delegateLidarDataChanged;
    }

    private void Color_camera_delegateCameraDataChanged(byte[] buffer)
    {
        publisher_camera_color.Send(buffer);
    }

    private void Lidar_delegateLidarDataChanged(byte[] buffer)
    {
        publisher_lidar_1.Send(buffer);
    }

    private void Subscriber_cmd_vel_delegateNewData(ulong sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
    {
        MemoryStream ms = new MemoryStream(buffer);
        RVector3 cmd = Serializer.Deserialize<RVector3>(ms);

        speed.x = cmd.x * 15;
        speed.y = cmd.y * 15;
        speed.z = cmd.theta * 10;

        print(speed.x + " " + speed.y + " " + speed.z);
    }

    // Update is called once per frame
    void Update()
    {
        transform.Translate(speed.y / 1000, 0, speed.x / 1000, Space.Self);
        transform.Rotate(0, -1 * speed.z / 10, 0, Space.Self);

        if (Manager.inited && !inited)
        {
            init();
            inited = true;
        }
    }
}
