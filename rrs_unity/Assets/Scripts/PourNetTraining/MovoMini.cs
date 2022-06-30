using ProtoBuf;
using RRS.Tools.Network;
using RRS.Tools.Protobuf;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class MovoMini : MonoBehaviour
{
    #region Net2

    //SUBS
    Net2.Net2HandlerSubscriber subscriber_rrs_joint_left_gripper;
    Net2.Net2HandlerSubscriber subscriber_rrs_joint_right_gripper;

    Net2.Net2HandlerPublisher publisher_status_right;
    Net2.Net2HandlerPublisher publisher_status_left;

    Net2.Net2HandlerPublisher publisher_status_mode;

    #endregion

    public GameObject right_gripper;
    public GameObject left_gripper;
    RVector7 last_gripper_command_left = null;
    RVector7 last_gripper_command_right = null;
    private bool inited = false;
    float next_status_time = 0;

    // Start is called before the first frame update
    void Start()
    {
        Statics.movo_mini_ref = this;
    }

    void init()
    {

        if (Statics.current_environment == Statics.Environments.Sim)
        {
            subscriber_rrs_joint_left_gripper = Net2.Subscriber("rrs_ros-gripper_left");
            subscriber_rrs_joint_left_gripper.delegateNewData += Subscriber_rrs_joint_left_gripper_delegateNewData;

            subscriber_rrs_joint_right_gripper = Net2.Subscriber("rrs_ros-gripper_right");
            subscriber_rrs_joint_right_gripper.delegateNewData += Subscriber_rrs_joint_right_gripper_delegateNewData;

            publisher_status_left = Net2.Publisher("gripper_status_left");
            publisher_status_right = Net2.Publisher("gripper_status_right");

            publisher_status_mode = Net2.Publisher("status_mode");

        }
        
    }

    private void Subscriber_rrs_joint_right_gripper_delegateNewData(long sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
    {
        //print("Gripper Right");

        if (priority == 10) return;


        MemoryStream ms = new MemoryStream(buffer);
        RVector7 t_last_gripper_command_right = Serializer.Deserialize<RVector7>(ms);

        Vector3 p = Helper.Ros2Unity(new Vector3(t_last_gripper_command_right.x, t_last_gripper_command_right.y, t_last_gripper_command_right.z));
        Quaternion q = Helper.Ros2Unity(new Quaternion(t_last_gripper_command_right.qx, t_last_gripper_command_right.qy, t_last_gripper_command_right.qz, t_last_gripper_command_right.qw));

        last_gripper_command_right = new RVector7();
        last_gripper_command_right.x = -p.y;
        last_gripper_command_right.y = p.x;
        last_gripper_command_right.z = p.z;

        last_gripper_command_right.qx = q.z;
        last_gripper_command_right.qy = q.y;
        last_gripper_command_right.qz = -q.x;
        last_gripper_command_right.qw = q.w;
    }

    private void Subscriber_rrs_joint_left_gripper_delegateNewData(long sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
    {
        //print("Gripper Left");

        if (priority == 10) return;

        MemoryStream ms = new MemoryStream(buffer);
        RVector7 t_last_gripper_command_left = Serializer.Deserialize<RVector7>(ms);

        Vector3 p = Helper.Ros2Unity(new Vector3(t_last_gripper_command_left.x, t_last_gripper_command_left.y, t_last_gripper_command_left.z));
        Quaternion q = Helper.Ros2Unity(new Quaternion(t_last_gripper_command_left.qx, t_last_gripper_command_left.qy, t_last_gripper_command_left.qz, t_last_gripper_command_left.qw));

        last_gripper_command_left = new RVector7();
        last_gripper_command_left.x = -p.y;
        last_gripper_command_left.y = p.x;
        last_gripper_command_left.z = p.z;

        last_gripper_command_left.qx = -q.y;
        last_gripper_command_left.qy = q.x;
        last_gripper_command_left.qz = q.z;
        last_gripper_command_left.qw = q.w;
    }

    private float timer_status = 0;
    private float timer_motor_update = 0;

    [Range(1, 100)]
    public int fps_status = 50;

    void updateMotors()
    {
        float t_s = 1;

        if (last_gripper_command_right != null)
        {
            right_gripper.transform.position = new Vector3(last_gripper_command_right.x * t_s, last_gripper_command_right.y * t_s, last_gripper_command_right.z * t_s);
            right_gripper.transform.rotation = new Quaternion(last_gripper_command_right.qx, last_gripper_command_right.qy, last_gripper_command_right.qz , last_gripper_command_right.qw );
            right_gripper.transform.Rotate(0, 0, 90);


        }

        if (last_gripper_command_left != null)
        {
            left_gripper.transform.position = new Vector3(last_gripper_command_left.x * t_s, last_gripper_command_left.y * t_s, last_gripper_command_left.z * t_s);
            left_gripper.transform.rotation = new Quaternion(last_gripper_command_left.qx, last_gripper_command_left.qy, last_gripper_command_left.qz , last_gripper_command_left.qw);
        }
    }

    void sendStatus()
    {
        RVector7 right_marker = new RVector7();

        var c_pose = Helper.Unity2Ros(right_gripper.transform.localPosition);

        right_marker.x = c_pose.x;
        right_marker.y = c_pose.y;
        right_marker.z = c_pose.z;

        var c_rot = Helper.Unity2Ros(right_gripper.transform.localRotation);

        right_marker.qx = c_rot.x;
        right_marker.qy = c_rot.y;
        right_marker.qz = c_rot.z;
        right_marker.qw = c_rot.w;

        MemoryStream ms = new MemoryStream();
        Serializer.Serialize<RVector7>(ms, right_marker);
        byte[] data_right = ms.ToArray();

        //Left
        RVector7 left_marker = new RVector7();

        c_pose = Helper.Unity2Ros(left_gripper.transform.localPosition);

        left_marker.x = c_pose.x;
        left_marker.y = c_pose.y;
        left_marker.z = c_pose.z;

        c_rot = Helper.Unity2Ros(left_gripper.transform.localRotation);

        left_marker.qx = c_rot.x;
        left_marker.qy = c_rot.y;
        left_marker.qz = c_rot.z;
        left_marker.qw = c_rot.w;

        ms = new MemoryStream();
        Serializer.Serialize<RVector7>(ms, left_marker);
        byte[] data_left = ms.ToArray();

        publisher_status_right.Send(data_right);
        publisher_status_left.Send(data_left);
        }


    void sendMode()
    {
        //MemoryStream ms = new MemoryStream();
        //ms = new MemoryStream();
        //Serializer.Serialize<string>(ms, Statics.current_config.mode);
        //byte[] data = ms.ToArray();
        //publisher_status_mode.Send(data);
    }

    // Update is called once per frame
    void Update()
    {
        timer_status += Time.deltaTime;
        timer_motor_update += Time.deltaTime;

        if (inited && Time.time >= next_status_time)
        {
            next_status_time = Time.time + (1 / fps_status);
            sendStatus();
            updateMotors();
            sendMode();
        }

        if (Manager.inited && !inited)
        {
            init();
            inited = true;
        }
    }
}
