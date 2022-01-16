using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using IIR_Butterworth_C_Sharp;
using ProtoBuf;
using UnityEngine;

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

    [Serializable]
    [ProtoContract]
    public class Laser
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

        public Laser()
        {

        }
    }


    [Serializable]
    [ProtoContract]
    public class MovoStatus
    {
        [ProtoMember(1)]
        public int id;
        [ProtoMember(2)]
        public RVector3 location;
        [ProtoMember(3)]
        public RVector3 target_location;
        [ProtoMember(4)]
        public RVector3 temp_target_location;
        [ProtoMember(5)]
        public int state;
        [ProtoMember(6)]
        public float battery;
        [ProtoMember(7)]
        public RVector3[] path;
        [ProtoMember(8)]
        public RVector3[] temp_path;
        [ProtoMember(9)]
        public int temprature = 0;
        [ProtoMember(10)]
        public Laser laser;
        [ProtoMember(11)]
        public int version;
        [ProtoMember(12)]
        public ulong time_span;
        [ProtoMember(13)]
        public int speed1;
        [ProtoMember(14)]
        public int speed2;
        [ProtoMember(15)]
        public int position;
        [ProtoMember(16)]
        public int load;
        [ProtoMember(17)]
        public int sensor;
        [ProtoMember(18)]
        public int alarm;
        [ProtoMember(19)]
        public byte[] map_data;
        [ProtoMember(20)]
        public int map_size;
        [ProtoMember(21)]
        public HapticRender right_arm_haptic_render;
        [ProtoMember(22)]
        public HapticRender left_arm_haptic_render;
        [ProtoMember(23)]
        public MovoArm right_arm;
        [ProtoMember(24)]
        public MovoArm left_arm;
        [ProtoMember(25)]
        public MovoHead head;
        [ProtoMember(26)]
        public RVector6 right_arm_end_effector;
        [ProtoMember(27)]
        public RVector6 left_arm_end_effector;
        [ProtoMember(28)]
        public RVector7 q_right_arm_end_effector;
        [ProtoMember(29)]
        public RVector7 q_left_arm_end_effector;
        [ProtoMember(30)]
        public float z_axes;
        [ProtoMember(31)]
        public SVector4 temp_target_rotation;

        public MovoStatus()
        {
        }
    }

    [Serializable]
    [ProtoContract]
    public class MovoJoint
    {
        [ProtoMember(1)]
        public float position;
        [ProtoMember(2)]
        public float velocity;
        [ProtoMember(3)]
        public float effort;

        public MovoJoint()
        {

        }
    }

    [Serializable]
    [ProtoContract]
    public class MovoArm
    {
        [ProtoMember(1)]
        public MovoJoint[] joints;
        [ProtoMember(2)]
        public MovoJoint gripper;

        public MovoArm()
        {

        }
    }

    [Serializable]
    [ProtoContract]
    public class MovoHead
    {
        [ProtoMember(1)]
        public MovoJoint[] joints;

        public MovoHead()
        {

        }
    }


   
   

    [Serializable]
    [ProtoContract]
    public class HapticRender
    {
        [ProtoMember(1)]
        public string cmd = "";
        [ProtoMember(2)]
        public RVector3 force = null;
        [ProtoMember(3)]
        public RVector3 torque = null;
        [ProtoMember(4)]
        public float gripper_force = 0;
        [ProtoMember(5)]
        public int version = 1;
        [ProtoMember(6)]
        public ulong time_span = 0;

        private List<double> list = new List<double>();
        double[] filtered_list;
        private int window = 20;
        private float distance = 0.01f;

        bool use_filter = true;
        IIR_Butterworth_Interface IBI;
        double f1 = 49;          //Cut-off
        static double  sf = 100; //Sampling frequency
        int order_filt = 2;      //Order
        double Nyquist_F = sf / 2;

        double[][] coeff_final = new double[2][];

        public double current_magnitude = 0;
        public float raw_magnitude = 0;

        public HapticRender()
        {

        }

        public HapticRender(float distance)
        {
            this.distance = distance;
            IBI = new IIR_Butterworth_C_Sharp.IIR_Butterworth_Implementation();

            coeff_final = IBI.Lp2lp(f1 / Nyquist_F, order_filt);
            bool check = IBI.Check_stability_iir(coeff_final);
        }

        public void addMessurment(HapticRender next)
        {
            Vector3 vt = new Vector3(next.torque.x, next.torque.y, next.torque.theta);
            float final = vt.magnitude / distance;

            final = (final / Physics.gravity.magnitude);
            raw_magnitude = final;

            list.Add(final);

            if (list.Count > window)
            {
                list.RemoveAt(0);
            }

            if (use_filter)
            {
                filtered_list = IBI.Filter_Data(coeff_final, list.ToArray());
                current_magnitude = filtered_list[filtered_list.Length - 1];
            }
        }

        public float forceMagnitude()
        {
            if (!use_filter)
            {
                if (list.Count > 0)
                    return (float)list.Average();
            }
           
            return (float)current_magnitude; 
        }

       
    }

}
