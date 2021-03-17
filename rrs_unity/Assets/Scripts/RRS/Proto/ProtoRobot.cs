using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ProtoBuf;

namespace RRS.Tools.Protobuf
{
    [ProtoContract]
    public class RRSCollision
    {
        [ProtoMember(1)]
        public int collision_time;

        [ProtoMember(2)]
        public SVector3[] points;

        [ProtoMember(3)]
        public int robot_id;

        [ProtoMember(4)]
        public int collision_type;

        public RRSCollision()
        {

        }
    }
    
    [ProtoContract]
    public class RRSLaser
    {
        [ProtoMember(1)]
        public float[] ranges; //m

        [ProtoMember(2)]
        public int robot_id; //the robot id

        [ProtoMember(3)]
        public float angel_min; //rad

        [ProtoMember(4)]
        public float angel_max; //rad

        [ProtoMember(5)]
        public float angel_increment; //rad

        [ProtoMember(6)]
        public float range_min; //m

        [ProtoMember(7)]
        public float range_max; //m

        [ProtoMember(8)]
        public float time_increment; //sec

        [ProtoMember(9)]
        public float scan_time; //sec

        public RRSLaser()
        {

        }
    }

    [ProtoContract]
    public class RRSTF
    {
        [ProtoMember(1)]
        public string[] names;
        [ProtoMember(2)]
        public string[] parents;
        [ProtoMember(3)]
        public RRSTransform[] transforms;
    }

    [ProtoContract]
    public class RRSCameraInfo
    {
        [ProtoMember(1)]
        public int height = 0;

        [ProtoMember(2)]
        public int width = 0;

        [ProtoMember(3)]
        public string distortion_model;

        [ProtoMember(4)]
        public float[] D;

        [ProtoMember(5)]
        public float[] K;

        [ProtoMember(6)]
        public float[] R;

        [ProtoMember(7)]
        public float[] P;

        [ProtoMember(8)]
        public int binning_x = 0;

        [ProtoMember(9)]
        public int binning_y = 0;
    }

    [ProtoContract]
    public class RRSParam
    {
        [ProtoMember(1)]
        public string[] list;
    }

    [ProtoContract]
    public class RRSTransform
    {
        [ProtoMember(1)]
        public SVector3 position;

        [ProtoMember(2)]
        public SVector4 orientation;
    }

    [ProtoContract]
    public class RRSOdom
    {
        [ProtoMember(1)]
        public SVector3 position;

        [ProtoMember(2)]
        public SVector4 orientation;

        [ProtoMember(3)]
        public SVector3 linear_speed;

        [ProtoMember(4)]
        public SVector3 angular_speed;
    }

    [ProtoContract]
    public class RRSNavGoal
    {
        [ProtoMember(1)]
        public RRSTransform transform;
    }

    [ProtoContract]
    public class RRSTagList
    {
        [ProtoMember(1)]
        public RRSTransform[] transforms;
    }

    [ProtoContract]
    public class RRSPointList
    {
        [ProtoMember(1)]
        public RRSTransform[] points;
    }

    [ProtoContract]
    public class RRSLasers
    {
        [ProtoMember(1)]
        public RRSLaser[] lasers_data;

        public RRSLasers()
        {

        }
    }

    [ProtoContract]
    public class RRSJointState
    {
        [ProtoMember(1)]
        public string[] name;
        [ProtoMember(2)]
        public float[] position;
        [ProtoMember(3)]
        public float[] velocity;
        [ProtoMember(4)]
        public float[] effort;
    }

    [ProtoContract]
    public class RRSJointCommand
    {
        [ProtoMember(1)]
        public string[] name;
        [ProtoMember(2)]
        public float[] goal;
        [ProtoMember(3)]
        public float[] velocity;
        [ProtoMember(4)]
        public float[] torque;
    }

    [ProtoContract]
    public class RRSIMU
    {
        [ProtoMember(1)]
        public SVector4 orientation;
        [ProtoMember(2)]
        public float[] orientation_covariance;
        [ProtoMember(3)]
        public SVector3 angular_velocity;
        [ProtoMember(4)]
        public float[] angular_velocity_covariance;
        [ProtoMember(5)]
        public SVector3 linear_acceleration;
        [ProtoMember(6)]
        public float[] linear_acceleration_covariance;
    }

    [ProtoContract]
    public class RobotCommand
    {
        [ProtoMember(1)]
        public int version = 1;
        [ProtoMember(2)]
        public ulong time_span = 0;
        [ProtoMember(3)]
        public RVector3 velocity = new RVector3(0, 0, 0);
        [ProtoMember(4)]
        public string command = "";
        [ProtoMember(5)]
        public float joint_right_shoulder_goal = 0;
        [ProtoMember(6)]
        public float joint_left_shoulder_goal = 0;
        [ProtoMember(7)]
        public float joint_right_elbow_goal = 0;
        [ProtoMember(8)]
        public float joint_left_elbow_goal = 0;
        [ProtoMember(9)]
        public float joint_head_pan_goal = 0;
        [ProtoMember(10)]
        public float joint_head_tilt_goal = 0;
        [ProtoMember(11)]
        public int id = 0;
        [ProtoMember(12)]
        public RRSBaseProtocol face_command;
        [ProtoMember(13)]
        public LEDProtocol led_command;

        public RobotCommand()
        {

        }
    }
}
