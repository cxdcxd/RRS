using System;
using System.Linq;
using System.Collections.Generic;
using System.IO;
using ProtoBuf;
using RRS.Tools.Network;
using RRS.Tools.Protobuf;
using UnityEngine;

public class Franka : MonoBehaviour
{
    #region Net2

    public GameObject nmpc_marker;

    //PUBS
    Net2.Net2HandlerPublisher publisher_joint_state;
    Net2.Net2HandlerPublisher publisher_nmpc_in;
    Net2.Net2HandlerSubscriber subscriber_rrs_command;
    Net2.Net2HandlerSubscriber subscriber_rrs_joint_command;

    #endregion

    void Start()
    {
        Statics.franka_ref = this;
    }

    public enum Links
    {
        panda_link0, 
        panda_link1,  
        panda_link2, 
        panda_link3, 
        panda_link4, 
        panda_link5, 
        panda_link6, 
        panda_hand
    }

    [Range(1, 100)]
    public int fps_status = 50;

    float next_status_time = 0;

    [Range(1, 100)]
    public float fps_control_update = 50;

    float next_control_time = 0;

    public GameObject target;
    
    public GameObject tag_test;
   
    public GameObject[] arm_joints;
    public GameObject[] finger_joints;
   

    public HapticRender arm_force = new HapticRender();


    private bool inited = false;
    private SVector3 current_position = new SVector3(0, 0, 0);
    private SVector4 current_orientation = new SVector4(0, 0, 0, 0);
    private GameObject _target;
    private float timer_status = 0;
    private float timer_motor_update = 0;
    private Vector3 speed_hand = new Vector3(0, 0, 0);
    private float speed_gripper = 0;
    public GameObject ik_hand_target;
    public ArticulationBody root_body;
    public Transform root_transform;
    public float[] d_joints;
    float[] c_joints;
    int joint_numbers = 8;
    public float stifness = 200;
    public float damping = 60;
    public float linear_friction = 0.05f;
    public float angular_friction = 0.05f;
    public float friction = 0.05f;
    public float target_velocity = 0.7f;

    void init()
    {
        d_joints = new float[joint_numbers];
        c_joints = new float[joint_numbers];

        if (Statics.current_environment == Statics.Environments.Sim)
        {
            publisher_joint_state = Net2.Publisher("franka_joint_state");
            publisher_nmpc_in = Net2.Publisher("nmpc_franka_in");
            subscriber_rrs_command = Net2.Subscriber("rrs_ros-rrs_command");
            subscriber_rrs_joint_command = Net2.Subscriber("rrs_ros-joint_franka");
            subscriber_rrs_joint_command.delegateNewData += Subscriber_rrs_joint_command_delegateNewData;
        }
       
    }


    #region MotorControl
   
    public bool enable_script_control = false;

    void updateMotors()
    {
        simpleviz();
    }

    #endregion

    public GameObject[] franka_arm;

   

    public float arm_1 = 0;
    public float arm_2 = 0;
    public float arm_3 = 0;
    public float arm_4 = 90;
    public float arm_5 = 0;
    public float arm_6 = -90;
    public float arm_7 = 0;
    public float gripper = 0;

    void simpleviz()
    {
        if (Statics.current_environment == Statics.Environments.Sim)
        {
            if (enable_script_control)
            {
                if (inited){
                    arm_1 = d_joints[0]* Mathf.Rad2Deg;
                    arm_2 = d_joints[1]* Mathf.Rad2Deg;
                    arm_3 = d_joints[2]* Mathf.Rad2Deg;
                    arm_4 = -d_joints[3]* Mathf.Rad2Deg;
                    arm_5 = d_joints[4]* Mathf.Rad2Deg;
                    arm_6 = -d_joints[5]* Mathf.Rad2Deg;
                    arm_7 = d_joints[6]* Mathf.Rad2Deg;
                    //print(arm_1);
                }
                //arm_1 += d_joints[0] *  Mathf.Rad2Deg;
               // arm_2 += d_joints[1] *  Mathf.Rad2Deg;
               // arm_3 += d_joints[2] *  Mathf.Rad2Deg;
               // arm_4 += d_joints[3] *  Mathf.Rad2Deg;
               // arm_5 += d_joints[4] *  Mathf.Rad2Deg;
               // arm_6 += d_joints[5] *  Mathf.Rad2Deg;
              //  arm_7 += d_joints[6] *  Mathf.Rad2Deg;
                //print(arm_1);
            }
        }
        else
        {
            arm_1 = d_joints[0];
            arm_2 = d_joints[1];
            arm_3 = d_joints[2];
            arm_4 = d_joints[3];
            arm_5 = d_joints[4];
            arm_6 = d_joints[5];
            arm_7 = d_joints[6];
        }

        arm_joints[1].transform.localRotation = Quaternion.Euler(0,arm_1, 0);
        arm_joints[2].transform.localRotation = Quaternion.Euler(arm_2, 0, 90 );
        arm_joints[3].transform.localRotation = Quaternion.Euler(arm_3, 0, -90 );
        arm_joints[4].transform.localRotation = Quaternion.Euler(arm_4, 0, -90);
        arm_joints[5].transform.localRotation = Quaternion.Euler(arm_5, 0, 90 );
        arm_joints[6].transform.localRotation = Quaternion.Euler(arm_6, 0, -90 );
        arm_joints[7].transform.localRotation = Quaternion.Euler(arm_7 - 45, 0, -90);
    }
   
    private void Subscriber_rrs_joint_command_delegateNewData(long sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
    {
        MemoryStream ms = new MemoryStream(buffer);
        RRSJointCommand cmd = Serializer.Deserialize<RRSJointCommand>(ms);
        for (int i = 0; i < 7; i++)
        {
            d_joints[i] = cmd.goal[i];
        }

        //print("Received");
    }
   
    void sendNMPCMarkers()
    {
        //Right
        RVector7 marker = new RVector7();

        var c_pose = Helper.Unity2Ros(nmpc_marker.transform.localPosition);

        marker.x = c_pose.x;
        marker.y = c_pose.y;
        marker.z = c_pose.z;

        var c_rot = Helper.Unity2Ros(nmpc_marker.transform.localRotation);

        marker.qx = c_rot.x;
        marker.qy = c_rot.y;
        marker.qz = c_rot.z;
        marker.qw = c_rot.w;

        MemoryStream ms = new MemoryStream();

        //print(marker.x);
        Serializer.Serialize<RVector7>(ms, marker);
        byte[] data = ms.ToArray();

        if (Statics.current_environment == Statics.Environments.Sim)
        {
            publisher_nmpc_in.Send(data);
        }
        else
        {
            HapticCommand command = new HapticCommand();
            command.position = new RVector3();
            command.rotation_q = new SVector4();

            command.position.x = marker.x;
            command.position.y = marker.y;
            command.position.z = marker.z;

            command.rotation_q.x = marker.qx;
            command.rotation_q.y = marker.qy;
            command.rotation_q.z = marker.qz;
            command.rotation_q.w = marker.qw;

            Statics.network_manager_arm.sendMessage(command);
        }

    }

    private void sendState()
    {
        if (Statics.current_environment == Statics.Environments.Sim)
        sendJointState();
        sendNMPCMarkers();
    }

    RRSTransform getRRSTransform(Transform t,Links name)
    {
        RRSTransform rtransform = new RRSTransform();

        Quaternion ros_q = Helper.Unity2Ros(t.localRotation);
        Vector3 ros_p = Helper.Unity2Ros(t.localPosition);

        rtransform.position = new RRS.Tools.Protobuf.SVector3(ros_p.x, ros_p.y, ros_p.z);
        rtransform.orientation = new RRS.Tools.Protobuf.SVector4(ros_q.x, ros_q.y, ros_q.z, ros_q.w);
       
        return rtransform;
    }

    void sendGroundtruth()
    {
        RRSTransform t = new RRSTransform();

        t.position = new RRS.Tools.Protobuf.SVector3();
        t.orientation = new RRS.Tools.Protobuf.SVector4();
        Vector3 convertp = new Vector3();
      
        convertp = Helper.Unity2Ros(transform.position);

        t.position.x = convertp.x;
        t.position.y = convertp.y;
        t.position.z = convertp.z;

        var convertr = Helper.Unity2Ros(transform.rotation);
        t.orientation.x = convertr.x;
        t.orientation.y = convertr.y;
        t.orientation.z = convertr.z;
        t.orientation.w = convertr.w;

        MemoryStream ms = new MemoryStream();
        Serializer.Serialize<RRSTransform>(ms, t);
        byte[] data = ms.ToArray();

        //publisher_groundtruth.Send(data);
    }

    (float, float, float) CurrentPrimaryAxisRotationLinear()
    {
        float currentRotation = 0, currentEffort = 0, currentVel = 0;
        return (currentRotation, currentVel, currentEffort);
    }

   

    void sendJointState()
    {
        RRSJointState joint_state_msg = new RRSJointState();

        joint_state_msg.name = new string[joint_numbers];
        joint_state_msg.position = new float[joint_numbers];
        joint_state_msg.velocity = new float[joint_numbers];
        joint_state_msg.effort = new float[joint_numbers];

        joint_state_msg.name[0] = Links.panda_link1.ToString();
        joint_state_msg.name[1] = Links.panda_link2.ToString();
        joint_state_msg.name[2] = Links.panda_link3.ToString();
        joint_state_msg.name[3] = Links.panda_link4.ToString();
        joint_state_msg.name[4] = Links.panda_link5.ToString();
        joint_state_msg.name[5] = Links.panda_link6.ToString();
        joint_state_msg.name[6] = Links.panda_hand.ToString();
        joint_state_msg.name[7] = Links.panda_link0.ToString();

        //print(right_arm_1 * Mathf.Deg2Rad);
        //print(right_arm_2 * Mathf.Deg2Rad * -1);

        (joint_state_msg.position[0], joint_state_msg.velocity[0], joint_state_msg.effort[0]) = (arm_1 * Mathf.Deg2Rad , 0, 0);
        (joint_state_msg.position[1], joint_state_msg.velocity[1], joint_state_msg.effort[1]) = (arm_2 * Mathf.Deg2Rad , 0, 0);
        (joint_state_msg.position[2], joint_state_msg.velocity[2], joint_state_msg.effort[2]) = (arm_3 * Mathf.Deg2Rad , 0, 0);
        (joint_state_msg.position[3], joint_state_msg.velocity[3], joint_state_msg.effort[3]) = (-arm_4 * Mathf.Deg2Rad , 0, 0);
        (joint_state_msg.position[4], joint_state_msg.velocity[4], joint_state_msg.effort[4]) = (arm_5 * Mathf.Deg2Rad , 0, 0);
        (joint_state_msg.position[5], joint_state_msg.velocity[5], joint_state_msg.effort[5]) = (-arm_6 * Mathf.Deg2Rad , 0, 0);
        (joint_state_msg.position[6], joint_state_msg.velocity[6], joint_state_msg.effort[6]) = (arm_7 * Mathf.Deg2Rad , 0, 0);
        (joint_state_msg.position[7], joint_state_msg.velocity[7], joint_state_msg.effort[7]) = (0, 0, 0);

        //if (is_moving == false)
        //{
        //    for (int i = 0; i < joint_state_msg.position.Length; i++)
        //    {
        //        c_joints[i] = joint_state_msg.position[i];
        //    }
        //}
        //else
        //{
        //    for (int i = 0; i < joint_state_msg.position.Length; i++)
        //    {
        //        joint_state_msg.position[i] = c_joints[i];
        //    }
        //}

        MemoryStream ms = new MemoryStream();
        Serializer.Serialize<RRSJointState>(ms, joint_state_msg);
        byte[] data = ms.ToArray();
        inited=true;
        publisher_joint_state.Send(data);

        //print("Franka State Published");
    }

   
    void Update()
    {
        timer_status += Time.deltaTime;
        timer_motor_update += Time.deltaTime;

        current_position.x = transform.position.x;
        current_position.y = transform.position.y;
        current_position.z = transform.position.z;

        current_orientation.x = transform.rotation.x;
        current_orientation.y = transform.rotation.y;
        current_orientation.z = transform.rotation.z;
        current_orientation.w = transform.rotation.w;

        if ( inited && Time.time >= next_status_time )
        {
            next_status_time = Time.time + (1 / fps_status);
            sendState();
            updateMotors();
        }

       
        if (Manager.inited && !inited)
        {
            init();
            inited = true;
        }
    }

    private float originalWidth = 1000;
    private float originalHeight = 800;

}
