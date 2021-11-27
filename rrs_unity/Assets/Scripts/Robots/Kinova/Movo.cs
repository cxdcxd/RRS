﻿using System;
using System.IO;
using ProtoBuf;
using RRS.Tools.Network;
using RRS.Tools.Protobuf;
using UnityEngine;

public class Movo : MonoBehaviour
{
    #region Net2

    public ArticulationMove right_kinova;
    public ArticulationMove left_kinova;

    public GameObject nmpc_right_marker;
    public GameObject nmpc_left_marker;

    //PUBS
    Net2.Net2HandlerPublisher publisher_lidar_front;
    Net2.Net2HandlerPublisher publisher_lidar_rear;
    Net2.Net2HandlerPublisher publisher_camera_color;
    Net2.Net2HandlerPublisher publisher_camera_info;
    Net2.Net2HandlerPublisher publisher_camera_depth;
    Net2.Net2HandlerPublisher publisher_camera_segment;
    Net2.Net2HandlerPublisher publisher_camera_normal;
    Net2.Net2HandlerPublisher publisher_joint_state;
    Net2.Net2HandlerPublisher publisher_imu;
    Net2.Net2HandlerPublisher publisher_odometry;
    Net2.Net2HandlerPublisher publisher_tf;
    Net2.Net2HandlerPublisher publisher_groundtruth;
    Net2.Net2HandlerPublisher publisher_nmpc_in_right;
    Net2.Net2HandlerPublisher publisher_nmpc_in_left;

    //SUBS
    Net2.Net2HandlerSubscriber subscriber_cmd_vel;
    Net2.Net2HandlerSubscriber subscriber_planner_viz;
    Net2.Net2HandlerSubscriber subscriber_navigation_state;
    Net2.Net2HandlerSubscriber subscriber_rrs_command;
    Net2.Net2HandlerSubscriber subscriber_rrs_joint_command;

    #endregion

    public enum Links
    {
        left_arm_half_joint, 
        left_elbow_joint, 
        left_gripper_finger1_joint, 
        left_shoulder_lift_joint,
        left_shoulder_pan_joint, 
        left_wrist_3_joint,
        left_wrist_spherical_1_joint,
        left_wrist_spherical_2_joint,
        linear_joint, 
        pan_joint, 
        right_arm_half_joint, 
        right_elbow_joint, 
        right_gripper_finger1_joint,
        right_shoulder_lift_joint, 
        right_shoulder_pan_joint, 
        right_wrist_3_joint, 
        right_wrist_spherical_1_joint,
        right_wrist_spherical_2_joint, 
        tilt_joint,
        base_link,
        odom
    }

    [Range(1, 100)]
    public int fps_status = 50;

    float next_status_time = 0;

    [Range(1, 100)]
    public float fps_control_update = 50;

    float next_control_time = 0;

    [Range(1, 100)]
    public float fps_nav_path_update = 3;

    public GameObject target;
    public Camera sensor_kinect;
    public GameObject tag_test;
    public GameObject[] head_joints;
    public GameObject[] left_arm_joints;
    public GameObject[] right_arm_joints;
    public GameObject[] left_finger_joints;
    public GameObject[] right_finger_joints;
    public GameObject linear_joint;

    public Lidar lidar_front;
    public Lidar lidar_rear;
    public ColorCamera color_camera;
    ColorCamera depth_camera;
    ColorCamera segment_camera;
    ColorCamera normal_camera;
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

    public ArticulationBody root_right_body;
    public ArticulationBody root_left_body;
    public ArticulationBody root_head_body;

    public Transform root_right_transform;
    public Transform root_left_transform;
    public Transform root_head_transform;

    public bool is_moving = false;

    public float[] d_joints;
    float[] c_joints;
    int joint_numbers = 19;

    public float stifness = 200;
    public float damping = 60;
    public float linear_friction = 0.05f;
    public float angular_friction = 0.05f;
    public float friction = 0.05f;
    public float target_velocity = 0.7f;

    void initJointsConfig()
    {
        foreach (var item in right_kinova.joints)
        {
            item.angularDamping = angular_friction;
            item.linearDamping = linear_friction;
            item.jointFriction = friction;

            ArticulationDrive drive = item.xDrive;
            drive.damping = damping;
            drive.stiffness = stifness;
            drive.targetVelocity = target_velocity;
            item.xDrive = drive;
        }

        foreach (var item in left_kinova.joints)
        {
            item.angularDamping = angular_friction;
            item.linearDamping = linear_friction;
            item.jointFriction = friction;

            ArticulationDrive drive = item.xDrive;
            drive.damping = damping;
            drive.stiffness = stifness;
            drive.targetVelocity = target_velocity;
            item.xDrive = drive;
        }

    }

    void init()
    {
        

        d_joints = new float[joint_numbers];
        c_joints = new float[joint_numbers];

        publisher_joint_state = Net2.Publisher("joint_state");
        publisher_lidar_front = Net2.Publisher("lidar_front");
        publisher_lidar_rear = Net2.Publisher("lidar_rear");
        publisher_camera_color = Net2.Publisher("camera_color");
        publisher_camera_info = Net2.Publisher("camera_info");
        //publisher_camera_depth = Net2.Publisher("camera_depth");
        //publisher_camera_segment = Net2.Publisher("camera_segment");
        //publisher_camera_normal = Net2.Publisher("camera_normal");
        publisher_groundtruth = Net2.Publisher("groundtruth");
        publisher_imu = Net2.Publisher("imu");
        publisher_odometry = Net2.Publisher("odometry");
        publisher_tf = Net2.Publisher("tf");
        publisher_nmpc_in_right = Net2.Publisher("nmpc_right_in");
        publisher_nmpc_in_left = Net2.Publisher("nmpc_left_in");

        subscriber_cmd_vel = Net2.Subscriber("rrs_ros-cmd_vel");
        subscriber_cmd_vel.delegateNewData += Subscriber_cmd_vel_delegateNewData;

        subscriber_planner_viz = Net2.Subscriber("rrs_ros-planner_viz");
        subscriber_planner_viz.delegateNewData += Subscriber_planner_viz_delegateNewData;

        subscriber_rrs_command = Net2.Subscriber("rrs_ros-rrs_command");
        subscriber_rrs_command.delegateNewData += Subscriber_rrs_command_delegateNewData;

        subscriber_rrs_joint_command = Net2.Subscriber("rrs_ros-joint_command");
        subscriber_rrs_joint_command.delegateNewData += Subscriber_rrs_joint_command_delegateNewData;

        //Sensors
        color_camera.delegateCameraDataChanged += Color_camera_delegateCameraDataChanged;
        //depth_camera.delegateCameraDataChanged += Depth_camera_delegateCameraDataChanged;
        //segment_camera.delegateCameraDataChanged += Segment_camera_delegateCameraDataChanged;
        //normal_camera.delegateCameraDataChanged += Normal_camera_delegateCameraDataChanged;

        imu.delegateIMUDataChanged += Imu_delegateIMUDataChanged;
        lidar_front.delegateLidarDataChanged += Lidar_front_delegateLidarDataChanged;
        lidar_rear.delegateLidarDataChanged += Lidar_rear_delegateLidarDataChanged;
    }

    private void Normal_camera_delegateCameraDataChanged(byte[] buffer)
    {
        //publisher_camera_normal.Send(buffer);
    }

    #region MotorControl

    void locomotion()
    {
        if (speed.x != 0 || speed.y != 0 || speed.z != 0)
        {
            is_moving = true;
        }
        else
        {
            is_moving = false;
        }


        transform.Translate(speed.y / 1000, 0, speed.x / 1000, Space.Self);
        transform.Rotate(0, -1 * speed.z / 10, 0, Space.Self);

        if (is_moving)
        {
            root_right_body.TeleportRoot(root_right_transform.position, root_right_transform.rotation);
            root_left_body.TeleportRoot(root_left_transform.position, root_left_transform.rotation);
            root_head_body.TeleportRoot(root_head_transform.position, root_head_transform.rotation);
        }

    }

    void moveHead()
    {
        if (d_joints.Length == 0) return;

        RotateToHead(d_joints[17], 0);
        RotateToHead(d_joints[18], 1);
    }

    void moveLeftHand()
    {
        if (d_joints.Length == 0) return;

        for (int i = 0; i < 7; i++)
        {
            RotateToLeft(d_joints[i + 8], i);
        }
    }

    void moveRightHand()
    {
        if (d_joints.Length == 0) return;

        for ( int i = 0; i < 7; i++)
        {
            RotateToRight(d_joints[i], i);
        }

    }

    void rightGripper()
    {
        if (d_joints.Length == 0) return;

        float value = d_joints[7];

        //RotateToRightFinger(value);
    }

    void leftGripper()
    {
        if (d_joints.Length == 0) return;

        float value = d_joints[15];

        //RotateToLefttFinger(value);
    }

    public bool enable_script_control = false;

    void updateMotors()
    {
        simpleviz();

        //initJointsConfig();

        //if (enable_script_control == false) return;



        //locomotion();

        //moveHead();

        //moveRightHand();
        //moveLeftHand();

        //rightGripper();
        //leftGripper();


    }

    #endregion

    #region SensorFeedbak
    private void Lidar_front_delegateLidarDataChanged(byte[] buffer)
    {
        //print("Send Lidar Front");
        publisher_lidar_front.Send(buffer);
    }

    private void Lidar_rear_delegateLidarDataChanged(byte[] buffer)
    {
        publisher_lidar_rear.Send(buffer);
    }

    private void Imu_delegateIMUDataChanged(byte[] buffer)
    {
        publisher_imu.Send(buffer);
    }

    private void Segment_camera_delegateCameraDataChanged(byte[] buffer)
    {
        //print("Segment");
        //publisher_camera_segment.Send(buffer);
    }

    private void Depth_camera_delegateCameraDataChanged(byte[] buffer)
    {
        //print("Depth");
        //publisher_camera_depth.Send(buffer);
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

    void RotateToRight(float primaryAxisRotation, int index)
    {
        var articulation = right_arm_joints[++index].GetComponent<ArticulationBody>();
        var drive = articulation.xDrive;
        drive.target = primaryAxisRotation;
        articulation.xDrive = drive;
    }

    void RotateToRightFinger(float primaryAxisRotation)
    {
        var articulation1 = right_finger_joints[0].GetComponent<ArticulationBody>();
        var articulation2 = right_finger_joints[1].GetComponent<ArticulationBody>();
        var articulation3 = right_finger_joints[2].GetComponent<ArticulationBody>();

        var drive = articulation1.xDrive;
        drive.target = primaryAxisRotation;
        articulation1.xDrive = drive;

        drive = articulation2.xDrive;
        drive.target = primaryAxisRotation;
        articulation2.xDrive = drive;

        drive = articulation3.xDrive;
        drive.target = primaryAxisRotation;
        articulation3.xDrive = drive;
    }

    void RotateToLefttFinger(float primaryAxisRotation)
    {
        var articulation1 = left_finger_joints[0].GetComponent<ArticulationBody>();
        var articulation2 = left_finger_joints[1].GetComponent<ArticulationBody>();
        var articulation3 = left_finger_joints[2].GetComponent<ArticulationBody>();

        var drive = articulation1.xDrive;
        drive.target = primaryAxisRotation;
        articulation1.xDrive = drive;

        drive = articulation2.xDrive;
        drive.target = primaryAxisRotation;
        articulation2.xDrive = drive;

        drive = articulation3.xDrive;
        drive.target = primaryAxisRotation;
        articulation3.xDrive = drive;
    }

    void RotateToLeft(float primaryAxisRotation, int index)
    {
        var articulation = left_arm_joints[++index].GetComponent<ArticulationBody>();
        var drive = articulation.xDrive;
        drive.target = primaryAxisRotation;
        articulation.xDrive = drive;
    }

    void RotateToHead(float primaryAxisRotation, int index)
    {
        var articulation = head_joints[index].GetComponent<ArticulationBody>();
        var drive = articulation.xDrive;
        drive.target = primaryAxisRotation;
        articulation.xDrive = drive;
    }

    public GameObject[] movo_head;
    public GameObject[] movo_right_arm;
    public GameObject[] movo_left_arm;

    public float head_pan = 0;
    public float head_tilt = 0;

    public float right_arm_1 = 0;
    public float right_arm_2 = 0;
    public float right_arm_3 = 0;
    public float right_arm_4 = 0;
    public float right_arm_5 = 0;
    public float right_arm_6 = 0;
    public float right_arm_7 = 0;
    public float right_gripper = 0;

    public float left_arm_1 = 0;
    public float left_arm_2 = 0;
    public float left_arm_3 = 0;
    public float left_arm_4 = 0;
    public float left_arm_5 = 0;
    public float left_arm_6 = 0;
    public float left_arm_7 = 0;
    public float left_gripper = 0;

    void simpleviz()
    {

        right_arm_1 = d_joints[0] * -1;
        right_arm_2 = d_joints[1] ;
        right_arm_3 = d_joints[2] * -1;
        right_arm_4 = d_joints[3] * -1;
        right_arm_5 = d_joints[4] * -1;
        right_arm_6 = d_joints[5] * -1;
        right_arm_7 = d_joints[6] * -1;

        left_arm_1 = d_joints[8] * -1;
        left_arm_2 = d_joints[9] ;
        left_arm_3 = d_joints[10] * -1;
        left_arm_4 = d_joints[11] * -1;
        left_arm_5 = d_joints[12] * -1;
        left_arm_6 = d_joints[13] * -1;
        left_arm_7 = d_joints[14] * -1;

        head_pan = d_joints[17] * -1;
        head_tilt = d_joints[18] * -1;

        head_joints[0].transform.localRotation = Quaternion.Euler(0, head_pan, -180);
        head_joints[1].transform.localRotation = Quaternion.Euler(90 + head_tilt,0 , 90);

        right_arm_joints[1].transform.localRotation = Quaternion.Euler(-180,right_arm_1,0);
        right_arm_joints[2].transform.localRotation = Quaternion.Euler(right_arm_2, 0, -90 );
        right_arm_joints[3].transform.localRotation = Quaternion.Euler(right_arm_3, 0, 90 );
        right_arm_joints[4].transform.localRotation = Quaternion.Euler(right_arm_4, 0, 90 );
        right_arm_joints[5].transform.localRotation = Quaternion.Euler(right_arm_5, 180, 90 );
        right_arm_joints[6].transform.localRotation = Quaternion.Euler(right_arm_6, 0, 90 );
        right_arm_joints[7].transform.localRotation = Quaternion.Euler(right_arm_7, 180, 90);

        left_arm_joints[1].transform.localRotation = Quaternion.Euler(-180, left_arm_1, 0);
        left_arm_joints[2].transform.localRotation = Quaternion.Euler(left_arm_2, 0, -90);
        left_arm_joints[3].transform.localRotation = Quaternion.Euler(left_arm_3, 0, 90);
        left_arm_joints[4].transform.localRotation = Quaternion.Euler(left_arm_4, 0, 90);
        left_arm_joints[5].transform.localRotation = Quaternion.Euler(left_arm_5, 180, 90);
        left_arm_joints[6].transform.localRotation = Quaternion.Euler(left_arm_6, 0, 90);
        left_arm_joints[7].transform.localRotation = Quaternion.Euler(left_arm_7, 180, 90);
    }


    void RotateToLinear(float primaryAxisRotation)
    {

    }

    private void Subscriber_rrs_joint_command_delegateNewData(long sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
    {
        MemoryStream ms = new MemoryStream(buffer);
        RRSJointCommand cmd = Serializer.Deserialize<RRSJointCommand>(ms);

        //print(cmd.goal.Length);

        for ( int i = 0; i < cmd.goal.Length; i++)
        {
            d_joints[i] = cmd.goal[i] * Mathf.Rad2Deg * -1;
        }

        //print("-------------------------");
    }

    private void Subscriber_rrs_command_delegateNewData(long sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
    {
        //RRS Command
        //print("Get RRS Command");
    }

    private void Subscriber_navigation_state_delegateNewData(long sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
    {
        //Navigation Status
        //print("Get Navigation Status");
    }

    private void Subscriber_planner_viz_delegateNewData(long sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
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

    void sendNMPCMarkers()
    {
        //Right
        RVector7 right_marker = new RVector7();

        var c_pose = Helper.Unity2Ros(nmpc_right_marker.transform.localPosition);

        right_marker.x = c_pose.x;
        right_marker.y = c_pose.y;
        right_marker.z = c_pose.z;

        var c_rot = Helper.Unity2Ros(nmpc_right_marker.transform.localRotation);

        right_marker.qx = c_rot.x;
        right_marker.qy = c_rot.y;
        right_marker.qz = c_rot.z;
        right_marker.qw = c_rot.w;

        MemoryStream ms = new MemoryStream();
        Serializer.Serialize<RVector7>(ms, right_marker);
        byte[] data_right = ms.ToArray();

        //Left
        RVector7 left_marker = new RVector7();

        c_pose = Helper.Unity2Ros(nmpc_left_marker.transform.localPosition);

        left_marker.x = c_pose.x;
        left_marker.y = c_pose.y;
        left_marker.z = c_pose.z;

        c_rot = Helper.Unity2Ros(nmpc_left_marker.transform.localRotation);

        left_marker.qx = c_rot.x;
        left_marker.qy = c_rot.y;
        left_marker.qz = c_rot.z;
        left_marker.qw = c_rot.w;

        ms = new MemoryStream();
        Serializer.Serialize<RVector7>(ms, left_marker);
        byte[] data_left = ms.ToArray();

        publisher_nmpc_in_right.Send(data_right);
        publisher_nmpc_in_left.Send(data_left);

        //print("published");
    }

    private void sendState()
    {
        //sendGroundtruth();
        sendJointState();
        sendJointTF();
        sendNMPCMarkers();
    }

    private void Subscriber_cmd_vel_delegateNewData(long sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
    {
        MemoryStream ms = new MemoryStream(buffer);
        RVector3 cmd = Serializer.Deserialize<RVector3>(ms);

        speed.x = cmd.x * 15;
        speed.y = cmd.y * 15; 
        speed.z = cmd.z * 10;

        //print(speed.x + " " + speed.y + " " + speed.z);
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

        rtransform.position = new RRS.Tools.Protobuf.SVector3(ros_p.x, ros_p.y, ros_p.z);
        rtransform.orientation = new RRS.Tools.Protobuf.SVector4(ros_q.x, ros_q.y, ros_q.z, ros_q.w);
       
        return rtransform;
    }

    void sendJointTF()
    {
        RRSTF tf_msg = new RRSTF();
        tf_msg.names = new string[1];
        tf_msg.parents = new string[1];
        tf_msg.transforms = new RRSTransform[1];

        tf_msg.names[0] = Links.base_link.ToString();
        tf_msg.parents[0] = Links.odom.ToString();
        tf_msg.transforms[0] = getRRSTransform(transform, Links.base_link);

        MemoryStream ms = new MemoryStream();
        Serializer.Serialize<RRSTF>(ms, tf_msg);
        byte[] data = ms.ToArray();

        ////print("publish tf");
        publisher_tf.Send(data);
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

        publisher_groundtruth.Send(data);
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

        joint_state_msg.name[0] = Links.right_shoulder_pan_joint.ToString();
        joint_state_msg.name[1] = Links.right_shoulder_lift_joint.ToString();
        joint_state_msg.name[2] = Links.right_arm_half_joint.ToString();
        joint_state_msg.name[3] = Links.right_elbow_joint.ToString();
        joint_state_msg.name[4] = Links.right_wrist_spherical_1_joint.ToString();
        joint_state_msg.name[5] = Links.right_wrist_spherical_2_joint.ToString();
        joint_state_msg.name[6] = Links.right_wrist_3_joint.ToString();
        joint_state_msg.name[7] = Links.right_gripper_finger1_joint.ToString();

        joint_state_msg.name[8] = Links.left_shoulder_pan_joint.ToString();
        joint_state_msg.name[9] = Links.left_shoulder_lift_joint.ToString();
        joint_state_msg.name[10] = Links.left_elbow_joint.ToString();
        joint_state_msg.name[11] = Links.left_arm_half_joint.ToString();
        joint_state_msg.name[12] = Links.left_wrist_spherical_1_joint.ToString();
        joint_state_msg.name[13] = Links.left_wrist_spherical_2_joint.ToString();
        joint_state_msg.name[14] = Links.left_wrist_3_joint.ToString();
        joint_state_msg.name[15] = Links.left_gripper_finger1_joint.ToString();

        joint_state_msg.name[16] = Links.linear_joint.ToString();
        joint_state_msg.name[17] = Links.pan_joint.ToString();
        joint_state_msg.name[18] = Links.tilt_joint.ToString();
      
        for ( int i = 0; i < 7; i++)
        {
            (joint_state_msg.position[i], joint_state_msg.velocity[i], joint_state_msg.effort[i]) = (d_joints[i] * Mathf.Deg2Rad * -1, 0, 0 );
        }

        (joint_state_msg.position[7], joint_state_msg.velocity[7], joint_state_msg.effort[7]) = (d_joints[7] * Mathf.Deg2Rad * -1, 0, 0);

        for (int i = 8; i < 15; i++)
        {
            (joint_state_msg.position[i], joint_state_msg.velocity[i], joint_state_msg.effort[i]) = (d_joints[i] * Mathf.Deg2Rad * -1, 0, 0);
        }

        (joint_state_msg.position[15], joint_state_msg.velocity[15], joint_state_msg.effort[15]) = (d_joints[15] * Mathf.Deg2Rad * -1, 0, 0);

        (joint_state_msg.position[16], joint_state_msg.velocity[16], joint_state_msg.effort[16]) = (d_joints[16] * Mathf.Deg2Rad * -1, 0, 0);
        (joint_state_msg.position[17], joint_state_msg.velocity[17], joint_state_msg.effort[17]) = (d_joints[17] * Mathf.Deg2Rad * -1, 0, 0);
        (joint_state_msg.position[18], joint_state_msg.velocity[18], joint_state_msg.effort[18]) = (d_joints[18] * Mathf.Deg2Rad * -1, 0, 0);

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

        //print(joint_state_msg.position[1]);

        MemoryStream ms = new MemoryStream();
        Serializer.Serialize<RRSJointState>(ms, joint_state_msg);
        byte[] data = ms.ToArray();

        publisher_joint_state.Send(data);
    }

   
    void FixedUpdate()
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
}
