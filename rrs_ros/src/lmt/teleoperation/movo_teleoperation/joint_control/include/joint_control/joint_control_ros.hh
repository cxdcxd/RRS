#ifndef _Joint_Control_ROS_HH_
#define _Joint_Control_ROS_HH_

#include "joint_control/joint_control.hh"

#include <ros/ros.h>
#include <ros/subscriber.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <image_transport/image_transport.h>

#include <sstream>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <numeric>
#include <yaml-cpp/yaml.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Int32.h>

#include <message_filters/sync_policies/approximate_time.h>

#include <movo_msgs/PanTiltCmd.h>
#include <movo_msgs/LinearActuatorCmd.h>
#include <movo_msgs/JacoCartesianVelocityCmd.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

#include "joint_control/HapticCommand.h"
#include "joint_control/HapticRender.h"

#include <ros/package.h>
#include <kinova_msgs/PoseVelocity.h>
#include <mutex>          // std::mutex

#include <geometry_msgs/WrenchStamped.h>

#include <kinova_msgs/KinovaPose.h>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

#include "tf/LinearMath/Quaternion.h"

#include <tf_conversions/tf_kdl.h>

#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h"
#include "robotiq_2f_gripper_control/Robotiq2FGripper_robot_input.h"

using namespace std;

namespace roboland 
{

enum OperationMode
{
  ML,
  TeleMovo,
  TeleGripper
};

struct settings
{
  std::string operation_mode = "ML";
};


class JointControlRos 
{

public:
  JointControlRos(ros::NodeHandle &nh,ros::NodeHandle &pnh, int argc, char *argv[]);
  ~JointControlRos();

  boost::thread thread_main;

  void thrMain();

  JointControl *joint_control;
  
  bool is_file_exist(const char *fileName);
  bool loadYaml();

  std::string config_path = "";
  YAML::Node m_config;
  settings m_settings;
  OperationMode operation_mode = OperationMode::ML;

  //Voice
  ros::Publisher pub_voice_cmd;

  //Arm Feedbacks
  ros::Subscriber sub_right_gripper_feedback;
  ros::Subscriber sub_left_gripper_feedback;
  ros::Subscriber sub_right_arm_feedback;
  ros::Subscriber sub_left_arm_feedback;
  ros::Subscriber sub_ft_right;
  ros::Subscriber sub_ft_left;

  ros::Subscriber sub_jaco_right_force_feedback;
  ros::Subscriber sub_jaco_left_force_feedback;
  ros::Subscriber sub_jaco_right_hand_pose;

  //Arm Commands 
  ros::Publisher pub_right_hand_cmd;
  ros::Publisher pub_jaco_right_hand_cmd;
  ros::Publisher pub_jaco_right_hand_pose_cmd;
  ros::Publisher pub_jaco_left_hand_cmd;
  ros::Publisher pub_jaco_gripper;
  ros::Publisher pub_right_gripper_cmd;
  ros::Publisher pub_left_gripper_cmd;
  ros::Publisher pub_left_hand_cmd;
  ros::Publisher pub_ft;

  ros::Publisher pub_right_end_effector;
  ros::Publisher pub_right_end_effector_stamp;

  ros::Publisher pub_left_end_effector;
  ros::Publisher pub_left_end_effector_stamp;

  std_msgs::Header right_hand_header;
  std_msgs::Header left_hand_header;
  std_msgs::Header right_hand_gripper_header;
  std_msgs::Header left_hand_gripper_header;

  std::vector<float> ft_fx_list;
  std::vector<float> ft_fy_list;
  std::vector<float> ft_fz_list;
  std::vector<float> ft_tx_list;
  std::vector<float> ft_ty_list;
  std::vector<float> ft_tz_list;
  int window_size = 0;

  //Arm Home
  ros::Publisher pub_home_arms;

  //Arm Force Publisher
  ros::Publisher pub_right_force_render;
  ros::Publisher pub_left_force_render;

  ros::Subscriber sub_right_arm_haptic_control;
  ros::Subscriber sub_left_arm_haptic_control;

  ros::Subscriber sub_r_end_effector;
  ros::Subscriber sub_l_end_effector;

  int home_wait = 0; 
  int test_time = 0;
  bool haptic_update = false;
  bool init_offset_left = false;
  bool init_offset_right = false;

  //Arm and Grippers Callbacks
  void callbackFTRight(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  void callbackFTLeft(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  void callbackRightGripper(const sensor_msgs::JointState::ConstPtr &msg);
  void callbackLeftGripper(const sensor_msgs::JointState::ConstPtr &msg);
  void callbackRightArm(const sensor_msgs::JointState::ConstPtr &msg);
  void callbackLeftArm(const sensor_msgs::JointState::ConstPtr &msg);
  void callbackLeftArmPose(const kinova_msgs::KinovaPose::ConstPtr &msg);
  void callbackRightArmPose(const kinova_msgs::KinovaPose::ConstPtr &msg);
  
  void callbackRightControlHaptic(const joint_control::HapticCommand::ConstPtr &msg);
  void callbackLeftControlHaptic(const joint_control::HapticCommand::ConstPtr &msg);

  void callbackJacoRightForceFeedback();
  void callbackJacoLeftForceFeedback();

  bool callbackArmHome(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res);
  bool callbackModeNone(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res);
  bool callbackModeAutonomous(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res);
  bool callbackModeSigma7(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res);
  bool callbackModeMixed(std_srvs::Empty::Request  &req,std_srvs::Empty::Response &res);

  void callbackNmpcREefPose(const geometry_msgs::Pose::ConstPtr &msg);
  void callbackNmpcLEefPose(const geometry_msgs::Pose::ConstPtr &msg);

  geometry_msgs::Pose nmpc_right_offset;
  geometry_msgs::Pose nmpc_left_offset;

  geometry_msgs::Pose sigma_right_offset;
  geometry_msgs::Pose sigma_left_offset;

  geometry_msgs::Pose nmpc_right_current;
  geometry_msgs::Pose nmpc_left_current;
  
  joint_control::HapticCommand desire_right_haptic;
  bool desire_right_haptic_updated = false;
  joint_control::HapticCommand desire_left_haptic;
  bool desire_left_haptic_updated = false;

  bool set_stop;

  bool get_new;
  bool mutex;
  bool app_exit;
  int counter;
  int send_counter;
  std::string old_cmd = "";

  bool nmpc_active = true;

  void sendHomeArms();

  void sendLeftArm(double x,double y,double z,double rx,double ry,double rz);
  void sendRightArm(double x,double y,double z,double rx,double ry,double rz);

  void sendRightJacoArm(double x,double y,double z,double rx,double ry,double rz);
  void sendLeftJacoArm(double x,double y,double z,double rx,double ry,double rz);

  void sendLeftJacoArmNMPCQ(double x,double y,double z,double rx,double ry,double rz, double rw, string mode);
  void sendRightJacoArmNMPCQ(double x,double y,double z,double rx,double ry,double rz, double rw, string mode);

  void sendLeftGripper(int x);
  void sendRightGripperPos(int x);
  void sendRightGripperVel(int x);
  void sendLeftGripperPos(int x);
  void sendLeftGripperVel(int x);

  int left_current_gripper_pos = 0;
  int right_current_gripper_pos = 0;

  void sendHapticRenderRightArm(joint_control::HapticRender msg);
  void sendHapticRenderLeftArm(joint_control::HapticRender msg);

  void playVoice(std::string text);

  ros::ServiceServer srv_arm_home;

  int control_mode = 1;

  //0 => none
  //1 => direct
  //2 => shared 
  //3 => autonomous 

  sensor_msgs::JointState current_right_arm_state;
  sensor_msgs::JointState current_left_arm_state;
  sensor_msgs::JointState current_right_gripper_state;
  sensor_msgs::JointState current_left_gripper_state;
  kinova_msgs::KinovaPose current_right_arm_pose;

  std::mutex mtx_right_arm;
  std::mutex mtx_left_arm;
  std::mutex mtx_right_gripper;
  std::mutex mtx_right_arm_pose;
  std::mutex mtx_left_gripper;
  std::mutex mtx_right_desire_haptic;
  std::mutex mtx_left_desire_haptic; 
  std::mutex mtx_right_arm_nmpc;
  std::mutex mtx_left_arm_nmpc;

};

} 

#endif 
