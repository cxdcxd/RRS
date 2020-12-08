using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ProtoBuf;
using System.IO;
using System.Timers;
using System;
using RRS.Tools.Protobuf;
using RRS.Tools.Network;

public class Movo : MonoBehaviour
{
    public enum Links
    {
        left_finger_1_link,
        left_finger_2_link,
        left_finger_3_link,
        right_finger_1_link,
        right_finger_2_link,
        right_finger_3_link,
        left_arm_main_link,
        left_arm_1_link,
        left_arm_2_link,
        left_arm_3_link,
        left_arm_4_link,
        left_arm_5_link,
        left_arm_6_link,
        left_arm_7_link,
        linear_link,
        pan_link,
        right_arm_main_link,
        right_arm_1_link,
        right_arm_2_link,
        right_arm_3_link,
        right_arm_4_link,
        right_arm_5_link,
        right_arm_6_link,
        right_arm_7_link,
        tilt_link,
        camera_link,
        lidar_1_link,
        lidar_2_link,
        imu_link,
        base_link,
        map_link,
        odom        
    }

    #region Net2

    //PUBS
    Net2.Net2HandlerPublisher publisher_lidar_1;
    Net2.Net2HandlerPublisher publisher_lidar_2; //Later
    Net2.Net2HandlerPublisher publisher_camera_color;
    Net2.Net2HandlerPublisher publisher_camera_info;
    Net2.Net2HandlerPublisher publisher_camera_depth;
    Net2.Net2HandlerPublisher publisher_camera_segment;
    Net2.Net2HandlerPublisher publisher_joint_state;
    Net2.Net2HandlerPublisher publisher_imu;
    Net2.Net2HandlerPublisher publisher_odometry;
    Net2.Net2HandlerPublisher publisher_tf;
    Net2.Net2HandlerPublisher publisher_groundtruth;

    //SUBS
    Net2.Net2HandlerSubscriber subscriber_cmd_vel;
    Net2.Net2HandlerSubscriber subscriber_planner_viz;
    Net2.Net2HandlerSubscriber subscriber_navigation_state;
    Net2.Net2HandlerSubscriber subscriber_rrs_command;
    Net2.Net2HandlerSubscriber subscriber_rrs_joint_command;

    #endregion

    public float fps_status = 10;
    public float fps_motor_update = 10;
    public float fps_nav_path_update = 3;

    public GameObject target;
    public Camera sensor_kinect;
    public GameObject sensor_lidar_1;
    public GameObject sensor_lidar_2;
    public GameObject sensor_imu;
    public GameObject tag_test;
    public GameObject[] head_joints;
    public GameObject[] left_arm_joints;
    public GameObject[] right_arm_joints;
    public GameObject[] left_finger_joints;
    public GameObject[] right_finger_joints;
    public GameObject linear_joint;

    public Lidar lidar;
    public ColorCamera color_camera;
    public DepthCamera depth_camera;
    public SegmentCamera segment_camera;
    public IMU imu;

    private RVector3[] current_path = null;
    private bool path_updated = false;
    private bool inited = false;
    private SVector3 current_position = new SVector3(0, 0, 0);
    private SVector4 current_orientation = new SVector4(0, 0, 0, 0);
    private GameObject _target;
    private GameObject _temp_target;
    private float timer_status = 0;
    private float timer_motor_update = 0;
    private float timer_nav_path = 0;
    private Vector3 speed = new Vector3(0, 0, 0);
    private Vector2 speed_head = new Vector2(0, 0);
    private Vector3 speed_right_hand = new Vector3(0, 0, 0);
    private Vector3 speed_left_hand = new Vector3(0, 0, 0);
    private float speed_right_gripper = 0;
    private float speed_left_gripper = 0;

    public GameObject ik_head_target;
    public GameObject ik_right_hand_target;
    public GameObject ik_left_hand_target;

    public float right_arm_1 = 0;
    public float right_arm_2 = 0;
    public float right_arm_3 = 0;
    public float right_arm_4 = 0;
    public float right_arm_5 = 0;
    public float right_arm_6 = 0;
    public float right_arm_7 = 0;

    public float[] d_right_arm;

    //public float right_arm_8 = 0;

    void init()
    {
        d_right_arm = new float[7];

       
        publisher_joint_state = Net2.Publisher("joint_state");
        //publisher_lidar_1 = Net2.Publisher("lidar_1");
        //publisher_lidar_2 = Net2.Publisher("lidar_2");
        //publisher_camera_color = Net2.Publisher("camera_color");
        //publisher_camera_info = Net2.Publisher("camera_info");
        //publisher_camera_depth = Net2.Publisher("camera_depth");
        //publisher_camera_segment = Net2.Publisher("camera_segment");
        //publisher_groundtruth = Net2.Publisher("groundtruth");
        //publisher_imu = Net2.Publisher("imu");
        //publisher_odometry = Net2.Publisher("odometry");
        publisher_tf = Net2.Publisher("tf");

        //subscriber_cmd_vel = Net2.Subscriber("rrs_ros-cmd_vel");
        //subscriber_cmd_vel.delegateNewData += Subscriber_cmd_vel_delegateNewData;

        //subscriber_planner_viz = Net2.Subscriber("rrs_ros-planner_viz");
        //subscriber_planner_viz.delegateNewData += Subscriber_planner_viz_delegateNewData;

        //subscriber_navigation_state = Net2.Subscriber("rrs_ros-navigation_state");
        //subscriber_navigation_state.delegateNewData += Subscriber_navigation_state_delegateNewData;

        //subscriber_rrs_command = Net2.Subscriber("rrs_ros-rrs_command");
        //subscriber_rrs_command.delegateNewData += Subscriber_rrs_command_delegateNewData;

        subscriber_rrs_joint_command = Net2.Subscriber("rrs_ros-joint_command");
        subscriber_rrs_joint_command.delegateNewData += Subscriber_rrs_joint_command_delegateNewData;

        //Sensors
        //color_camera.delegateCameraDataChanged += Color_camera_delegateCameraDataChanged;
        //depth_camera.delegateCameraDataChanged += Depth_camera_delegateCameraDataChanged;
        //segment_camera.delegateCameraDataChanged += Segment_camera_delegateCameraDataChanged;
        //imu.delegateIMUDataChanged += Imu_delegateIMUDataChanged;
        //lidar.delegateLidarDataChanged += Lidar_delegateLidarDataChanged;
        
    }

    #region MotorControl
    void locomotion()
    {
        transform.Translate(speed.y / 1000, 0, speed.x / 1000, Space.Self);
        transform.Rotate(0, -1 * speed.z / 10, 0, Space.Self);
    }

    void moveHead()
    {
        ik_head_target.transform.Translate(speed_head.x / 1000, speed_head.y / 1000, 0, Space.Self);
    }

    void moveLeftHand()
    {
        //ik_left_hand_target.transform.Translate(speed_left_hand.x / 1000, speed_left_hand.y / 1000, speed_left_hand.z / 1000, Space.Self);
    }

    void moveRightHand()
    {
        if (d_right_arm.Length == 0) return;
        //ik_right_hand_target.transform.Translate(speed_right_hand.x / 1000, speed_right_hand.y / 1000, speed_right_hand.z / 1000, Space.Self);

        //right_arm_joints[0].transform.localRotation = Quaternion.Euler(right_arm_1 - 90, 90, -90);
        //right_arm_joints[1].transform.localRotation = Quaternion.Euler(right_arm_2, 0, -90);
        //right_arm_joints[2].transform.localRotation = Quaternion.Euler(right_arm_3, 0, 90);
        //right_arm_joints[3].transform.localRotation = Quaternion.Euler(right_arm_4, 0, 90);
        //right_arm_joints[4].transform.localRotation = Quaternion.Euler(right_arm_5, -180, 90);
        //right_arm_joints[5].transform.localRotation = Quaternion.Euler(right_arm_6, 0, 90);
        //right_arm_joints[6].transform.localRotation = Quaternion.Euler(right_arm_7, -180, 90);
        //right_arm_joints[0].transform.localRotation = Quaternion.Euler(right_arm_8, 0, 0);

        for ( int i = 0; i < 7; i++)
        {
            //print("Rotate to " + i.ToString() + " Value : " + d_right_arm[i]);
            //RotateTo(d_right_arm[i], i);
        }

    }

    void rightGripper()
    {
      //later
    }

    void leftGripper()
    {
      //later
    }

    void updateMotors()
    {
        //Navigation
        //locomotion();
        //moveHead();
        //moveLeftHand();
        moveRightHand();
        rightGripper();
        //leftGripper();

        //Joints (Torque + Dynamics) (PhysX 4.0)
        //Later
    }
    #endregion

    #region SensorFeedbak
    private void Lidar_delegateLidarDataChanged(byte[] buffer)
    {
        publisher_lidar_1.Send(buffer);
    }

    private void Imu_delegateIMUDataChanged(byte[] buffer)
    {
        publisher_imu.Send(buffer);
    }

    private void Segment_camera_delegateCameraDataChanged(byte[] buffer)
    {
        //print("Segment");
        publisher_camera_segment.Send(buffer);
    }

    private void Depth_camera_delegateCameraDataChanged(byte[] buffer)
    {
        //print("Depth");
        publisher_camera_depth.Send(buffer);
    }

    private void Color_camera_delegateCameraDataChanged(byte[] buffer)
    {
        publisher_camera_color.Send(buffer);

        //Sending Camera Info

        RRSCameraInfo info_msg = new RRSCameraInfo();

        //width
        //800

        //height
        //600

        //[narrow_stereo]

        //Camera matrix
        //520.907761 0.000000 398.668872
        //0.000000 520.193056 298.836832
        //0.000000 0.000000 1.000000

        //Distortion
        //0.006218 -0.004135 0.000217 -0.000706 0.000000

        //Rectification
        //1.000000 0.000000 0.000000
        //0.000000 1.000000 0.000000
        //0.000000 0.000000 1.000000

        //Projection
        //522.240356 0.000000 397.936985 0.000000
        //0.000000 521.915222 299.013784 0.000000
        //0.000000 0.000000 1.000000 0.000000

        info_msg.width = 800;
        info_msg.height = 600;

        info_msg.P = new float[12];

        info_msg.P[0] = 522.240356f;
        info_msg.P[1] = 0.000000f;
        info_msg.P[2] = 397.936985f;
        info_msg.P[3] = 0.000000f;
        info_msg.P[4] = 0.000000f;
        info_msg.P[5] = 521.915222f;
        info_msg.P[6] = 299.013784f;
        info_msg.P[7] = 0.000000f;
        info_msg.P[8] = 0.000000f;
        info_msg.P[9] = 0.000000f;
        info_msg.P[10] = 1.000000f;
        info_msg.P[11] = 0.000000f;

        info_msg.R = new float[9];
        info_msg.R[0] = 1.000000f;
        info_msg.R[1] = 0.000000f;
        info_msg.R[2] = 0.000000f;
        info_msg.R[3] = 0.000000f;
        info_msg.R[4] = 1.000000f;
        info_msg.R[5] = 0.000000f;
        info_msg.R[6] = 0.000000f;
        info_msg.R[7] = 0.000000f;
        info_msg.R[8] = 1.000000f;

        info_msg.K = new float[9];
        info_msg.K[0] = 520.907761f;
        info_msg.K[1] = 0.000000f;
        info_msg.K[2] = 398.668872f;
        info_msg.K[3] = 0.000000f;
        info_msg.K[4] = 520.193056f;
        info_msg.K[5] = 298.836832f;
        info_msg.K[6] = 0.000000f;
        info_msg.K[7] = 0.000000f;
        info_msg.K[8] = 1.000000f;

        info_msg.D = new float[5];
        info_msg.D[0] = 0.006218f;
        info_msg.D[1] = -0.004135f;
        info_msg.D[2] = 0.000217f;
        info_msg.D[3] = -0.000706f;
        info_msg.D[4] = 0.000000f;

        info_msg.distortion_model = "plumb_bob";
      
        MemoryStream ms = new MemoryStream();
        ms = new MemoryStream();
        Serializer.Serialize<RRSCameraInfo>(ms, info_msg);

        byte[] data = ms.ToArray();
        publisher_camera_info.Send(data);


    }
    #endregion

    void RotateTo(float primaryAxisRotation, int index)
    {
        index++;
        var articulation = right_arm_joints[index].GetComponent<ArticulationBody>();
        var drive = articulation.xDrive;
        drive.target = primaryAxisRotation;
        articulation.xDrive = drive;
    }

    private void Subscriber_rrs_joint_command_delegateNewData(ulong sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
    {
        //Movo Joint update 
        //print("Get Movo Joint Command");

        MemoryStream ms = new MemoryStream(buffer);

        RRSJointCommand cmd = Serializer.Deserialize<RRSJointCommand>(ms);

        print("Get Goal");
        print(cmd.goal.Length);

        for ( int i = 0; i < cmd.goal.Length; i++)
        {
            //RotateTo(cmd.goal[i] , i);
            d_right_arm[i] = cmd.goal[i] * Mathf.Rad2Deg * -1;
            //print("Joint Command Goal " + cmd.goal[i]);
            //break;
         }




    }

    private void Subscriber_rrs_command_delegateNewData(ulong sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
    {
        //RRS Command
        //print("Get RRS Command");
    }

    private void Subscriber_navigation_state_delegateNewData(ulong sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
    {
        //Navigation Status
        //print("Get Navigation Status");
    }

    private void Subscriber_planner_viz_delegateNewData(ulong sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
    {
        MemoryStream ms = new MemoryStream(buffer);

        try
        {
            RRSRobot cmd = Serializer.Deserialize<RRSRobot>(ms);
            current_path = (RVector3[])cmd.path.Clone();
            path_updated = true;
            //print("Path serialized done");
        }
        catch (Exception e)
        {
            //print("error : " + e.Message);
            //string msg = e.Message;
        }
    }

    private void sendState()
    {
        //sendGroundtruth();
        sendJointState();
        sendJointTF();
        //sendOdometry();
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

    public void manualCmdVel(Vector3 speed)
    {
        this.speed.x = speed.x;
        this.speed.y = speed.y;
        this.speed.z = speed.z; 
    }

    public void manualHead(Vector2 speed)
    {
        this.speed_head.x = speed.x;
        this.speed_head.y = speed.y;
    }

    RRSTransform getRRSTransform(Transform t,Links name)
    {
        RRSTransform rtransform = new RRSTransform();

        Quaternion ros_q = Helper.Unity2Ros(t.localRotation);
        Vector3 ros_p = Helper.Unity2Ros(t.localPosition);

        rtransform.position = new SVector3(ros_p.x, ros_p.y, ros_p.z);
        rtransform.orientation = new SVector4(ros_q.x, ros_q.y, ros_q.z, ros_q.w);
       
        return rtransform;
    }

    void sendJointTF()
    {
        RRSTF tf_msg = new RRSTF();
        tf_msg.names = new string[14];
        tf_msg.parents = new string[14];
        tf_msg.transforms = new RRSTransform[14];

        //tf_msg.names[0] = Links.pan_link.ToString();
        //tf_msg.parents[0] = Links.linear_link.ToString();
        //tf_msg.transforms[0] = getRRSTransform(head_joints[0].transform, Links.pan_link);

        //tf_msg.names[1] = Links.tilt_link.ToString();
        //tf_msg.parents[1] = Links.pan_link.ToString();
        //tf_msg.transforms[1] = getRRSTransform(head_joints[1].transform, Links.tilt_link);

        //tf_msg.names[2] = Links.camera_link.ToString();
        //tf_msg.parents[2] = Links.tilt_link.ToString();
        //tf_msg.transforms[2] = getRRSTransform(sensor_kinect.transform, Links.camera_link);

        //tf_msg.names[3] = Links.lidar_1_link.ToString();
        //tf_msg.parents[3] = Links.base_link.ToString();
        //tf_msg.transforms[3] = getRRSTransform(sensor_lidar_1.transform, Links.lidar_1_link);

        //tf_msg.names[4] = Links.lidar_2_link.ToString();
        //tf_msg.parents[4] = Links.base_link.ToString();
        //tf_msg.transforms[4] = getRRSTransform(sensor_lidar_2.transform, Links.lidar_2_link);
        int i = 0;

        tf_msg.names[i] = Links.odom.ToString();
        tf_msg.parents[i] = Links.map_link.ToString();
        tf_msg.transforms[i] = getRRSTransform(transform, Links.odom);

        i++;

        tf_msg.names[i] = Links.base_link.ToString();
        tf_msg.parents[i] = Links.odom.ToString();
        tf_msg.transforms[i] = getRRSTransform(transform, Links.base_link);

        i++;

        tf_msg.names[i] = Links.linear_link.ToString();
        tf_msg.parents[i] = Links.base_link.ToString();
        tf_msg.transforms[i] = getRRSTransform(linear_joint.transform, Links.linear_link);

        i++;

        tf_msg.names[i] = Links.right_arm_main_link.ToString();
        tf_msg.parents[i] = Links.linear_link.ToString();
        tf_msg.transforms[i] = getRRSTransform(right_arm_joints[0].transform, Links.right_arm_main_link);

        i++;

        tf_msg.names[i] = Links.right_arm_1_link.ToString();
        tf_msg.parents[i] = Links.right_arm_main_link.ToString();
        tf_msg.transforms[i] = getRRSTransform(right_arm_joints[1].transform, Links.right_arm_1_link);

        i++;

        tf_msg.names[i] = Links.right_arm_2_link.ToString();
        tf_msg.parents[i] = Links.right_arm_1_link.ToString();
        tf_msg.transforms[i] = getRRSTransform(right_arm_joints[2].transform, Links.right_arm_2_link);

        i++;

        tf_msg.names[i] = Links.right_arm_3_link.ToString();
        tf_msg.parents[i] = Links.right_arm_2_link.ToString();
        tf_msg.transforms[i] = getRRSTransform(right_arm_joints[3].transform, Links.right_arm_3_link);

        i++;

        tf_msg.names[i] = Links.right_arm_4_link.ToString();
        tf_msg.parents[i] = Links.right_arm_3_link.ToString();
        tf_msg.transforms[i] = getRRSTransform(right_arm_joints[4].transform, Links.right_arm_4_link);

        i++;

        tf_msg.names[i] = Links.right_arm_5_link.ToString();
        tf_msg.parents[i] = Links.right_arm_4_link.ToString();
        tf_msg.transforms[i] = getRRSTransform(right_arm_joints[5].transform, Links.right_arm_5_link);

        i++;

        tf_msg.names[i] = Links.right_arm_6_link.ToString();
        tf_msg.parents[i] = Links.right_arm_5_link.ToString();
        tf_msg.transforms[i] = getRRSTransform(right_arm_joints[6].transform, Links.right_arm_6_link);

        i++;

        tf_msg.names[i] = Links.right_arm_7_link.ToString();
        tf_msg.parents[i] = Links.right_arm_6_link.ToString();
        tf_msg.transforms[i] = getRRSTransform(right_arm_joints[7].transform, Links.right_arm_7_link);

        i++;

        tf_msg.names[i] = Links.right_finger_1_link.ToString();
        tf_msg.parents[i] = Links.right_arm_7_link.ToString();
        tf_msg.transforms[i] = getRRSTransform(right_finger_joints[0].transform, Links.right_finger_1_link);

        i++;

        tf_msg.names[i] = Links.right_finger_2_link.ToString();
        tf_msg.parents[i] = Links.right_arm_7_link.ToString();
        tf_msg.transforms[i] = getRRSTransform(right_finger_joints[1].transform, Links.right_finger_2_link);

        i++;

        tf_msg.names[i] = Links.right_finger_3_link.ToString();
        tf_msg.parents[i] = Links.right_arm_7_link.ToString();
        tf_msg.transforms[i] = getRRSTransform(right_finger_joints[2].transform, Links.right_finger_3_link);



        //tf_msg.names[17] = Links.left_arm_1_link.ToString();
        //tf_msg.parents[17] = Links.linear_link.ToString();
        //tf_msg.transforms[17] = getRRSTransform(left_arm_joints[0].transform, Links.left_arm_1_link);

        //tf_msg.names[18] = Links.left_arm_2_link.ToString();
        //tf_msg.parents[18] = Links.left_arm_1_link.ToString();
        //tf_msg.transforms[18] = getRRSTransform(left_arm_joints[1].transform, Links.left_arm_2_link);

        //tf_msg.names[19] = Links.left_arm_3_link.ToString();
        //tf_msg.parents[19] = Links.left_arm_2_link.ToString();
        //tf_msg.transforms[19] = getRRSTransform(left_arm_joints[2].transform, Links.left_arm_3_link);

        //tf_msg.names[20] = Links.left_arm_4_link.ToString();
        //tf_msg.parents[20] = Links.left_arm_3_link.ToString();
        //tf_msg.transforms[20] = getRRSTransform(left_arm_joints[3].transform, Links.left_arm_4_link);

        //tf_msg.names[21] = Links.left_arm_5_link.ToString();
        //tf_msg.parents[21] = Links.left_arm_4_link.ToString();
        //tf_msg.transforms[21] = getRRSTransform(left_arm_joints[4].transform, Links.left_arm_5_link);

        //tf_msg.names[22] = Links.left_arm_6_link.ToString();
        //tf_msg.parents[22] = Links.left_arm_5_link.ToString();
        //tf_msg.transforms[22] = getRRSTransform(left_arm_joints[5].transform, Links.left_arm_6_link);

        //tf_msg.names[23] = Links.left_arm_7_link.ToString();
        //tf_msg.parents[23] = Links.left_arm_6_link.ToString();
        //tf_msg.transforms[23] = getRRSTransform(left_arm_joints[6].transform, Links.left_arm_7_link);

        //tf_msg.names[24] = Links.left_finger_1_link.ToString();
        //tf_msg.parents[24] = Links.left_arm_7_link.ToString();
        //tf_msg.transforms[24] = getRRSTransform(left_finger_joints[0].transform, Links.left_finger_1_link);

        //tf_msg.names[25] = Links.left_finger_2_link.ToString();
        //tf_msg.parents[25] = Links.left_arm_7_link.ToString();
        //tf_msg.transforms[25] = getRRSTransform(left_finger_joints[1].transform, Links.left_finger_2_link);

        //tf_msg.names[26] = Links.left_finger_3_link.ToString();
        //tf_msg.parents[26] = Links.left_arm_7_link.ToString();
        //tf_msg.transforms[26] = getRRSTransform(left_finger_joints[2].transform, Links.left_finger_3_link);

        //tf_msg.names[27] = Links.imu_link.ToString();
        //tf_msg.parents[27] = Links.base_link.ToString();
        //tf_msg.transforms[27] = getRRSTransform(sensor_imu.transform, Links.imu_link);

        MemoryStream ms = new MemoryStream();
        Serializer.Serialize<RRSTF>(ms, tf_msg);
        byte[] data = ms.ToArray();

        //print("publish tf");
        publisher_tf.Send(data);
    }

    void sendOdometry()
    {
        RRSOdom t = new RRSOdom();

        t.position = new SVector3();
        t.orientation = new SVector4();

        var convertp = Helper.Unity2Ros(transform.position);
        t.position.x = convertp.x;
        t.position.y = convertp.y;
        t.position.z = convertp.z;

        var convertr = Helper.Unity2Ros(transform.rotation);
        t.orientation.x = convertr.x;
        t.orientation.y = convertr.y;
        t.orientation.z = convertr.z;
        t.orientation.w = convertr.w;

        t.linear_speed = new SVector3();
        var convertlv = Helper.Unity2Ros(imu.linVel);
        t.linear_speed.x = convertlv.x;
        t.linear_speed.y = convertlv.y;
        t.linear_speed.z = convertlv.z;

        t.angular_speed = new SVector3();
        var convertav = Helper.Unity2Ros(imu.angVel);
        t.angular_speed.x = convertav.x;
        t.angular_speed.y = convertav.y; 
        t.angular_speed.z = convertav.z; 

        MemoryStream ms = new MemoryStream();
        Serializer.Serialize<RRSOdom>(ms, t);
        byte[] data = ms.ToArray();

        publisher_odometry.Send(data);
    }

    void sendGroundtruth()
    {
        RRSTransform t = new RRSTransform();

        t.position = new SVector3();
        t.orientation = new SVector4();
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

        publisher_groundtruth.Send(data);
    }

    float CurrentPrimaryAxisRotation(int index)
    {
        index++;
        //print("Get Current Rotation " + index);
        var articulation = right_arm_joints[index].GetComponent<ArticulationBody>();
        float currentRotationRads = articulation.jointPosition[0];
        float currentRotation = currentRotationRads;
        return currentRotation;
    }

    void sendJointState()
    {
        RRSJointState joint_state_msg = new RRSJointState();

        joint_state_msg.name = new string[10];
        joint_state_msg.position = new float[10];
        joint_state_msg.velocity = new float[10];
        joint_state_msg.effort = new float[10];

        //joint_state_msg.name[0] = Links.pan_link.ToString();
        //joint_state_msg.name[1] = Links.tilt_link.ToString();

        joint_state_msg.name[0] = Links.right_arm_1_link.ToString();
        joint_state_msg.name[1] = Links.right_arm_2_link.ToString();
        joint_state_msg.name[2] = Links.right_arm_3_link.ToString();
        joint_state_msg.name[3] = Links.right_arm_4_link.ToString();
        joint_state_msg.name[4] = Links.right_arm_5_link.ToString();
        joint_state_msg.name[5] = Links.right_arm_6_link.ToString();
        joint_state_msg.name[6] = Links.right_arm_7_link.ToString();
        joint_state_msg.name[7] = Links.right_finger_1_link.ToString();
        joint_state_msg.name[8] = Links.right_finger_2_link.ToString();
        joint_state_msg.name[9] = Links.right_finger_3_link.ToString();

        //joint_state_msg.name[12] = Links.left_arm_1_link.ToString();
        //joint_state_msg.name[13] = Links.left_arm_2_link.ToString();
        //joint_state_msg.name[14] = Links.left_arm_3_link.ToString();
        //joint_state_msg.name[15] = Links.left_arm_4_link.ToString();
        //joint_state_msg.name[16] = Links.left_arm_5_link.ToString();
        //joint_state_msg.name[17] = Links.left_arm_6_link.ToString();
        //joint_state_msg.name[18] = Links.left_arm_7_link.ToString();
        //joint_state_msg.name[19] = Links.left_finger_1_link.ToString();
        //joint_state_msg.name[20] = Links.left_finger_2_link.ToString();
        //joint_state_msg.name[21] = Links.left_finger_3_link.ToString();

        //joint_state_msg.name[22] = Links.linear_link.ToString();

        //joint_state_msg.position[0] = head_joints[0].transform.localRotation.eulerAngles.y;
        //joint_state_msg.position[1] = head_joints[1].transform.localRotation.eulerAngles.x;

        //joint_state_msg.position[0] = right_arm_joints[0].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[1] = right_arm_joints[1].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[2] = right_arm_joints[2].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[3] = right_arm_joints[3].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[4] = right_arm_joints[4].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[5] = right_arm_joints[5].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[6] = right_arm_joints[6].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[7] = right_finger_joints[0].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[8] = right_finger_joints[1].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[9] = right_finger_joints[2].transform.localRotation.eulerAngles.x;

        joint_state_msg.position[0] = CurrentPrimaryAxisRotation(0);
        joint_state_msg.position[1] = CurrentPrimaryAxisRotation(1);
        joint_state_msg.position[2] = CurrentPrimaryAxisRotation(2);
        joint_state_msg.position[3] = CurrentPrimaryAxisRotation(3);
        joint_state_msg.position[4] = CurrentPrimaryAxisRotation(4);
        joint_state_msg.position[5] = CurrentPrimaryAxisRotation(5);
        joint_state_msg.position[6] = CurrentPrimaryAxisRotation(6);
        joint_state_msg.position[7] = 0;
        joint_state_msg.position[8] = 0;
        joint_state_msg.position[9] = 0;

        //joint_state_msg.position[12] = left_arm_joints[0].transform.localRotation.eulerAngles.y;
        //joint_state_msg.position[13] = left_arm_joints[1].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[14] = left_arm_joints[2].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[15] = left_arm_joints[3].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[16] = left_arm_joints[4].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[17] = left_arm_joints[5].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[18] = left_arm_joints[6].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[19] = left_finger_joints[0].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[20] = left_finger_joints[1].transform.localRotation.eulerAngles.x;
        //joint_state_msg.position[21] = left_finger_joints[2].transform.localRotation.eulerAngles.x;

        //joint_state_msg.position[22] = linear_joint.transform.localPosition.y;

        for ( int i = 0; i < 10; i++)
        {
            joint_state_msg.velocity[i] = 0;
            joint_state_msg.effort[i] = 0;
        }

        MemoryStream ms = new MemoryStream();
        Serializer.Serialize<RRSJointState>(ms, joint_state_msg);
        byte[] data = ms.ToArray();

        publisher_joint_state.Send(data);

        //print("Joint publish done");
    }

   

    Vector3[] generatePoints(Vector3[] keyPoints, int segments = 100)
    {
        Vector3[] Points = new Vector3[(keyPoints.Length - 1) * segments + keyPoints.Length];
        for (int i = 1; i < keyPoints.Length; i++)
        {
            Points[(i - 1) * segments + i - 1] = new Vector3(keyPoints[i - 1].x, 0.2f, keyPoints[i - 1].z);
            for (int j = 1; j <= segments; j++)
            {
                float x = keyPoints[i - 1].x;
                float y = 0.2f;
                float z = keyPoints[i - 1].z; //keyPoints [i - 1].z;
                float dx = (keyPoints[i].x - keyPoints[i - 1].x) / segments;
                float dz = (keyPoints[i].z - keyPoints[i - 1].z) / segments;
                Points[(i - 1) * segments + j + i - 1] = new Vector3(x + dx * j, 0.2f, z + dz * j);
            }
        }
        Points[(keyPoints.Length - 1) * segments + keyPoints.Length - 1] = new Vector3(keyPoints[keyPoints.Length - 1].x, 0.2f, keyPoints[keyPoints.Length - 1].z);
        return Points;
    }

    public void setTargetLocation(RVector3 location)
    {
        if (location != null && location.x != 0 && location.y != 0)
        {
            if (_target == null)
            {
                _target = (GameObject)Instantiate(target, new Vector3(location.x, 0.1f, location.y), Quaternion.identity);
                Renderer x = _target.GetComponent<Renderer>();
                x.material.color = Color.red;

                TextMesh txt = _target.transform.Find("number").gameObject.GetComponent<TextMesh>();
                txt.text = "F #" + 0;
            }
            else
            {
                _target.transform.position = new Vector3(location.x, 0.1f, location.y);
            }

            _target.gameObject.SetActive(true);
        }
        else
        {
            if (_target != null)
            {
                Destroy(_target);
            }
        }
    }

    public void setTempTargetLocation(RVector3 location)
    {
        if (location != null && location.x != 0 && location.y != 0)
        {
            if (_temp_target == null)
            {
                _temp_target = (GameObject)Instantiate(target, new Vector3(location.x, 0.1f, location.y), Quaternion.identity);
                Renderer x = _temp_target.GetComponent<Renderer>();
                x.material.color = Color.red;

                TextMesh txt = _temp_target.transform.Find("number").gameObject.GetComponent<TextMesh>();
                txt.text = "T #0";
            }
            else
            {
                _temp_target.transform.position = new Vector3(location.x, 0.1f, location.y);
            }

            _temp_target.gameObject.SetActive(true);
        }
        else
        {
            if (_temp_target != null)
            {
                Destroy(_target);
            }
        }
    }

    public void setPath(RVector3[] path)
    {
        path_updated = false;

        LineRenderer lr = GetComponent<LineRenderer>();
        lr.enabled = true;

        if (path != null)
        {
            lr.material.color = Color.red;
            lr.startWidth = 0.05f;
            lr.endWidth = 0.05f;

            //print("Path render " + path.Length);

            Vector3[] _path = new Vector3[path.Length];
            for (int i = 0; i < path.Length; i++)
                _path[i] = new Vector3(-1 * path[i].y , 0.01f, path[i].x );

            Vector3[] _path2 = generatePoints(_path);

            //print(_path2.Length);

            lr.positionCount = _path2.Length;
            lr.SetPositions(_path2);
        }
        else
        {
            lr.positionCount = 0;
        }
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

        if (inited)
        {
            if (timer_motor_update > (1 / fps_motor_update))
            {
                updateMotors();
                timer_motor_update = 0;
            }

            if ( timer_status > (1 / fps_status))
            {
                sendState();
                timer_status = 0;
            }


        }
           


            if (path_updated && current_path != null)
                setPath(current_path);
        

        if (Manager.inited && !inited)
        {
            init();
            inited = true;
        }
    }
}
