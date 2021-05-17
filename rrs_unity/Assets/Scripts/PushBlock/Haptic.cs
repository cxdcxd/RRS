using ProtoBuf;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public enum SystemMode
{
    TeleoperationSim,
    TeleoperationReal,
    Training,
    TestingReal,
    TestingSim,
}

public enum DoF
{
    two,
    three
}


public class Haptic : MonoBehaviour
{
    public static SystemMode system_mode = SystemMode.TeleoperationReal;
    public static DoF system_dof = DoF.two;

    public Vector3 current_speed = new Vector3(0, 0, 0);
    public ColorCamera color_camera;

    public float fps = 1;
    private float timer = 0.0f;

    public GameObject Block;
    public GameObject Agent;

    public Vector3 real_corner_a_left_up;
    public Vector3 real_corner_b_right_down;
    public float scale_factor = 20;

    public Vector3 sim_corner_a_left_up;
    public Vector3 sim_corner_b_right_down;

    public Vector3 realtosim(RVector3 real_pose, int index)
    {
        Vector3 new_pose = new Vector3(0,0,0);

        float sim_w = sim_corner_b_right_down.x - sim_corner_a_left_up.x;
        float sim_h = sim_corner_a_left_up.z - sim_corner_b_right_down.z;

        float real_w = real_corner_b_right_down.x - real_corner_a_left_up.x;
        float real_h = real_corner_a_left_up.z - real_corner_b_right_down.z;

        float new_x = ((real_pose.x - real_corner_a_left_up.x) / real_w ) * sim_w + sim_corner_a_left_up.x;
        float new_z = ((real_pose.z - real_corner_b_right_down.z) / real_h) * sim_h + sim_corner_b_right_down.z;

        new_pose.x = new_x;
        if (index == 1)
            new_pose.y = 1.5f;
        if (index == 2)
            new_pose.y = 1.5f;
        new_pose.z = new_z;

        return new_pose;
    }

    void Start()
    {
        AsyncIO.ForceDotNet.Force();

        print(Statics2.remote_ip);

        Statics2.network_main = new Network<IRawData, SVector3>(Statics2.local_ip, Statics2.remote_ip, "9150", "9151", "", NetworkType.PUBSUB, "unity");
        Statics2.network_main.eventDataUpdated += Network_main_eventDataUpdated;

        Statics2.network_real = new Network<HapticCommand, ObservationRL>(Statics2.local_ip, Statics2.robot_ip, "7778", "7777", "", NetworkType.PUBSUB, "real");
        Statics2.network_real.eventDataUpdated += Network_real_eventDataUpdated;

        //color_camera = GetComponent<ColorCamera>();
        //color_camera.delegateCameraDataChanged += Color_camera_delegateCameraDataChanged;

        float sim_w = sim_corner_b_right_down.x - sim_corner_a_left_up.x;
        float sim_h = sim_corner_a_left_up.z - sim_corner_b_right_down.z;

        float real_w = real_corner_b_right_down.x - real_corner_a_left_up.x;
        float real_h = real_corner_a_left_up.z - real_corner_b_right_down.z;

        print(sim_w + " " + real_w);
        print(sim_h + " " + real_h);
    }

    private void Network_real_eventDataUpdated()
    {
        //print("get");
        //TBD
        Statics2.current_env = Statics2.network_real.get;

       
    }

    private void Color_camera_delegateCameraDataChanged(byte[] buffer)
    {
        //print("Camera Changed");
        //RawData data = new RawData();
        //data.Data = buffer;
        //data.Length = buffer.Length;
        //Statics2.network_main.sendMessage(data);
    }

    //Linux Speed Information (X,Z)
    private void Network_main_eventDataUpdated()
    {
        //print("Netwrok Get");
        if (Statics2.network_main.get != null)
        {
            
                if (system_dof == DoF.three)
                {
                    current_speed.x = Statics2.network_main.get.x;
                    current_speed.y = Statics2.network_main.get.y;
                    current_speed.z = Statics2.network_main.get.z;
                }
                else if ( system_dof == DoF.two)
                {
                    float z = Statics2.network_main.get.x;
                    float x = Statics2.network_main.get.z;

                    if (z < 0) z = -1;
                    else if (z > 0) z = 1;

                    if (x > 0) x = -1;
                    else if (x < 0) x = 1;

                    current_speed.z = z;
                    current_speed.x = x;

                   //print("Tele real, Up/Down: " + current_speed.z + " Right/Left: " + current_speed.x);
            }

        }
      
    }


    void OnApplicationQuit()
    {
        if (Statics2.network_main != null)
        Statics2.network_main.killAll();

        if (Statics2.network_real != null)
            Statics2.network_real.killAll();

        NetMQ.NetMQConfig.Cleanup(false);
        print("Terminated Done");
    }

    void updateEnv()
    {
        if ( system_mode == SystemMode.TeleoperationReal || system_mode == SystemMode.TestingReal )
        {
            if ( Statics2.current_env != null)
            {
                Block.transform.localPosition = realtosim(Statics2.current_env.block_position,1);
                Block.transform.localRotation = new Quaternion(Statics2.current_env.block_rotation_q.x, Statics2.current_env.block_rotation_q.y, Statics2.current_env.block_rotation_q.z, Statics2.current_env.block_rotation_q.w);
                Vector3 t1 = Block.transform.localRotation.eulerAngles;
                t1.x = 0;
                t1.z = 0;
                Block.transform.localRotation = Quaternion.Euler(t1);


                Agent.transform.localPosition = realtosim(Statics2.current_env.hand_position, 2);
                Agent.transform.localRotation = new Quaternion(Statics2.current_env.hand_rotation_q.x, Statics2.current_env.hand_rotation_q.y, Statics2.current_env.hand_rotation_q.z, Statics2.current_env.hand_rotation_q.w);
                
                //Vector3 t2 = Block.transform.localRotation.eulerAngles;
                //t2.x = 0;
                //t2.z = 0;
                //Agent.transform.localRotation = Quaternion.Euler(t2);
            }
        }
    }

    void FixedUpdate()
    {
        timer += Time.deltaTime;
        if (timer > (1 / fps))
        {
			//SVector3 msg = new 	SVector3(0,0,0);
            //Statics2.network_main.sendMessage(msg);
            timer = 0;

          
        }

        updateEnv();

    }
}
