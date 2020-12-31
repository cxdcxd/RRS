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

    #region Net2

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

    //SUBS
    Net2.Net2HandlerSubscriber subscriber_cmd_vel;
    Net2.Net2HandlerSubscriber subscriber_planner_viz;
    Net2.Net2HandlerSubscriber subscriber_navigation_state;
    Net2.Net2HandlerSubscriber subscriber_rrs_command;
    Net2.Net2HandlerSubscriber subscriber_rrs_joint_command;

    #endregion

    public float fps_status = 50;
    public float fps_motor_update = 50;
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
    public ColorCameraOpenGL color_camera;
    public ColorCameraOpenGL depth_camera;
    public ColorCameraOpenGL segment_camera;
    public ColorCameraOpenGL normal_camera;
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

    float[] d_joints;

    void init()
    {
        d_joints = new float[19];

        publisher_joint_state = Net2.Publisher("joint_state");
        publisher_lidar_front = Net2.Publisher("lidar_front");
        publisher_lidar_rear = Net2.Publisher("lidar_rear");
        publisher_camera_color = Net2.Publisher("camera_color");
        publisher_camera_info = Net2.Publisher("camera_info");
        publisher_camera_depth = Net2.Publisher("camera_depth");
        publisher_camera_segment = Net2.Publisher("camera_segment");
        publisher_camera_normal = Net2.Publisher("camera_normal");
        publisher_groundtruth = Net2.Publisher("groundtruth");
        publisher_imu = Net2.Publisher("imu");
        publisher_odometry = Net2.Publisher("odometry");
        publisher_tf = Net2.Publisher("tf");

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
        depth_camera.delegateCameraDataChanged += Depth_camera_delegateCameraDataChanged;
        segment_camera.delegateCameraDataChanged += Segment_camera_delegateCameraDataChanged;
        normal_camera.delegateCameraDataChanged += Normal_camera_delegateCameraDataChanged;

        imu.delegateIMUDataChanged += Imu_delegateIMUDataChanged;
        lidar_front.delegateLidarDataChanged += Lidar_front_delegateLidarDataChanged;
        lidar_rear.delegateLidarDataChanged += Lidar_rear_delegateLidarDataChanged;
    }

    private void Normal_camera_delegateCameraDataChanged(byte[] buffer)
    {
        publisher_camera_normal.Send(buffer);
    }

    #region MotorControl
    void locomotion()
    {
        transform.Translate(speed.y / 1000, 0, speed.x / 1000, Space.Self);
        transform.Rotate(0, -1 * speed.z / 10, 0, Space.Self);
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
      //@TODO
    }

    void leftGripper()
    {
      //@TODO
    }

    void updateMotors()
    {
        locomotion();

        moveHead();
       
        moveRightHand();
        rightGripper();

        moveLeftHand();
        leftGripper();
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
        publisher_camera_segment.Send(buffer);
    }

    private void Depth_camera_delegateCameraDataChanged(byte[] buffer)
    {
        print("Depth");
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

    void RotateToRight(float primaryAxisRotation, int index)
    {
        var articulation = right_arm_joints[++index].GetComponent<ArticulationBody>();
        var drive = articulation.xDrive;
        drive.target = primaryAxisRotation;
        articulation.xDrive = drive;
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

    private void Subscriber_rrs_joint_command_delegateNewData(ulong sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
    {
        MemoryStream ms = new MemoryStream(buffer);
        RRSJointCommand cmd = Serializer.Deserialize<RRSJointCommand>(ms);

        //print("Get Goal");
        //print(cmd.goal.Length);

        for ( int i = 0; i < cmd.goal.Length; i++)
        {
            d_joints[i] = cmd.goal[i] * Mathf.Rad2Deg * -1;
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
        sendGroundtruth();
        sendJointState();
        sendOdometry();
        sendJointTF();
    }

    private void Subscriber_cmd_vel_delegateNewData(ulong sequence, byte[] buffer, uint priority, Net2.Net2HandlerBase sender)
    {
        MemoryStream ms = new MemoryStream(buffer);
        RVector3 cmd = Serializer.Deserialize<RVector3>(ms);

        speed.x = cmd.x * 15;
        speed.y = cmd.y * 15; 
        speed.z = cmd.theta * 10;

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

        rtransform.position = new SVector3(ros_p.x, ros_p.y, ros_p.z);
        rtransform.orientation = new SVector4(ros_q.x, ros_q.y, ros_q.z, ros_q.w);
       
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
        //var convertlv = Helper.Unity2Ros(imu.linVel);
        //t.linear_speed.x = convertlv.x;
        //t.linear_speed.y = convertlv.y;
        //t.linear_speed.z = convertlv.z;

        t.angular_speed = new SVector3();
        //var convertav = Helper.Unity2Ros(imu.angVel);
        //t.angular_speed.x = convertav.x;
        //t.angular_speed.y = convertav.y; 
        //t.angular_speed.z = convertav.z; 

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

    (float, float, float) CurrentPrimaryAxisRotationRight(int index)
    {
        float currentRotation = 0, currentEffort = 0, currentVel = 0;
        if (index >= 0 && index <= 7)
        {
            var articulation = right_arm_joints[++index].GetComponent<ArticulationBody>();
            if (articulation.jointPosition.dofCount == 1)
            {
                float currentRotationRads = articulation.jointPosition[0] * -1;
                currentRotation = currentRotationRads;
                currentVel = articulation.jointVelocity[0];
                currentEffort = articulation.jointForce[0];

                List<float> f = new List<float>();
                int x = articulation.GetJointForces(f);
                print(x);
                print(f.Count);
                print(f[0]);


            }
        }
        return (currentRotation, currentVel, currentEffort);
    }

    (float, float, float) CurrentPrimaryAxisRotationLeft(int index)
    {
        float currentRotation = 0, currentEffort = 0, currentVel = 0;
        if (index >= 0 && index <= 7)
        {
            var articulation = left_arm_joints[++index].GetComponent<ArticulationBody>();
            if (articulation.jointPosition.dofCount == 1)
            {
                float currentRotationRads = articulation.jointPosition[0] * -1;
                currentRotation = currentRotationRads;
                currentVel = articulation.jointVelocity[0];
                currentEffort = articulation.jointForce[0];
            }
        }
        return (currentRotation, currentVel, currentEffort);
    }

    (float, float, float) CurrentPrimaryAxisRotationHead(int index)
    {
        float currentRotation = 0, currentEffort = 0, currentVel = 0;
        var articulation = head_joints[index].GetComponent<ArticulationBody>();
        if (articulation.jointPosition.dofCount == 1)
        {
            float currentRotationRads = articulation.jointPosition[0] * -1;
            currentRotation = currentRotationRads;
            currentVel = articulation.jointVelocity[0];
            currentEffort = articulation.jointForce[0];
        }
        return (currentRotation, currentVel, currentEffort);
    }

    (float, float, float) CurrentPrimaryAxisRotationLinear()
    {
        float currentRotation = 0, currentEffort = 0, currentVel = 0;
        return (currentRotation, currentVel, currentEffort);
    }

    void sendJointState()
    {
        RRSJointState joint_state_msg = new RRSJointState();

        joint_state_msg.name = new string[19];
        joint_state_msg.position = new float[19];
        joint_state_msg.velocity = new float[19];
        joint_state_msg.effort = new float[19];

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
            (joint_state_msg.position[i], joint_state_msg.velocity[i], joint_state_msg.effort[i]) = CurrentPrimaryAxisRotationRight(i);
        }

        for (int i = 8; i < 15; i++)
        {
            (joint_state_msg.position[i], joint_state_msg.velocity[i], joint_state_msg.effort[i]) = CurrentPrimaryAxisRotationLeft(i - 8);
        }

        (joint_state_msg.position[16], joint_state_msg.velocity[16], joint_state_msg.effort[16]) = CurrentPrimaryAxisRotationLinear();

        (joint_state_msg.position[17], joint_state_msg.velocity[17], joint_state_msg.effort[17]) = CurrentPrimaryAxisRotationHead(0);
        (joint_state_msg.position[18], joint_state_msg.velocity[18], joint_state_msg.effort[18]) = CurrentPrimaryAxisRotationHead(1);

    
        MemoryStream ms = new MemoryStream();
        Serializer.Serialize<RRSJointState>(ms, joint_state_msg);
        byte[] data = ms.ToArray();

        publisher_joint_state.Send(data);
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
            updateMotors();
            sendState();
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
