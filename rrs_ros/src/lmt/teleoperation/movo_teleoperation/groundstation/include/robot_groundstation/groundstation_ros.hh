#ifndef _Groundstation_ROS_HH_
#define _Groundstation_ROS_HH_

#include "robot_groundstation/groundstation.hh"

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
#include <vector>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/WrenchStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <movo_msgs/PanTiltCmd.h>
#include <movo_msgs/LinearActuatorCmd.h>
#include <movo_msgs/JacoCartesianVelocityCmd.h>
#include <movo_msgs/Battery.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include "joint_control/HapticCommand.h"
#include "joint_control/HapticRender.h"
#include "joint_control/GamepadCommand.h"
#include "joint_control/ArmTrajectory.h"
#include "joint_control/ArmTrajectoryList.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <mutex>        // std::mutex

#include <robot_groundstation/DataReduction.h>
#include <robot_groundstation/text.h>
#include <kinova_msgs/KinovaPose.h>

#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"

#include <chrono>
#include <ctime>

#include <eigen3/Eigen/Dense>
#include <numeric>
#include "Iir.h"


using namespace std;

namespace roboland 
{

struct settings
{
  std::string station_WIFI_ip = "10.66.171.182";
  std::string station_5G1_ip = "10.162.148.135";
  std::string station_5G2_ip = "10.162.148.31";
  std::string station_mode = "WIFI"; //or 5G
  bool station_camera_stream = true;
  bool station_lidar_stream = true;
  bool station_status_stream = true;
  int station_camera_frame_skip_ratio = 1;
  float station_camera_resize_ratio = 0.3;
  int station_camera_jpeg_compression_ratio = 50;
  bool print_debug = false;
};


class GroundstationRos 
{

public:
  GroundstationRos(ros::NodeHandle &nh,ros::NodeHandle &pnh, int argc, char *argv[],std::string id);
  ~GroundstationRos();

  boost::thread thread_main;

  void thrMain();

  Groundstation *ground_station;

  ros::Subscriber sub_laser;
  ros::Subscriber sub_camera;
  
  ros::Subscriber sub_right_gripper_feedback;
  ros::Subscriber sub_left_gripper_feedback;
  ros::Subscriber sub_right_arm_feedback;
  ros::Subscriber sub_left_arm_feedback;
  ros::Subscriber sub_head_feedback;
  ros::Subscriber sub_battery_feedback;
  ros::Subscriber sub_z_axes_feedback;
  ros::Subscriber sub_battery;
  ros::Subscriber sub_block_position;
  ros::Subscriber sub_hand_pose;
  ros::Subscriber sub_end_effector_right;
  ros::Subscriber sub_end_effector_left;

  ros::Subscriber robot_status;
  ros::Subscriber sub_clusters;
  ros::Publisher pub_ft_right;
  ros::Publisher pub_ft_left;
  ros::Publisher pub_base_cmd;
  ros::Publisher pub_head_cmd;
  ros::Publisher pub_head_roll_cmd;
  ros::Publisher pub_linear_cmd;
  ros::Publisher pub_demonstration;
  ros::Publisher pub_QOS;

  ros::Publisher pub_voice_cmd;
  ros::Publisher pub_nav_cmd;
  ros::Publisher pub_home_arms;

  ros::Subscriber sub_right_force_render;
  ros::Subscriber sub_left_force_render;
  ros::Publisher pub_right_arm_haptic_control;
  ros::Publisher pub_left_arm_haptic_control;
  ros::Publisher pub_ml;

  ros::Publisher pub_diagram_force_network_rate;
  ros::Publisher pub_diagram_force_network_deadband_value;
  ros::Publisher pub_diagram_force_network_send;
  ros::Publisher pub_diagram_force_network_force;

  ros::Publisher pub_diagram_velocity_network_rate;
  ros::Publisher pub_diagram_velocity_network_deadband_value;
  ros::Publisher pub_diagram_velocity_network_send;
  ros::Publisher pub_diagram_velocity_network_command;

  ros::Publisher pub_ob1;
  ros::Publisher pub_ob2;
  ros::Publisher pub_ob3;
  ros::Publisher pub_ob1s;
  ros::Publisher pub_ob2s;

  ros::Publisher pub_arm_status;
  // ros::Publisher pub_ob3s;

  ros::Publisher pub_camera_color;

  std_msgs::Header head_header;
  std_msgs::Header base_header;
  std_msgs::Header z_header;
  std_msgs::Header right_hand_header;
  std_msgs::Header left_hand_header;

  std::mutex gMutex_;
  cv_bridge::CvImagePtr cv_ptr;

  sensor_msgs::LaserScan current_laser;
  movo_msgs::Battery current_battery;

  geometry_msgs::WrenchStamped right_offset;
  int right_offset_counter = 0;

  geometry_msgs::WrenchStamped left_offset;
  int left_offset_counter = 0;

  std::vector<float> left_ft_fx_list;
  std::vector<float> left_ft_fy_list;
  std::vector<float> left_ft_fz_list;
  std::vector<float> left_ft_tx_list;
  std::vector<float> left_ft_ty_list;
  std::vector<float> left_ft_tz_list;

  std::vector<float> right_ft_fx_list;
  std::vector<float> right_ft_fy_list;
  std::vector<float> right_ft_fz_list;
  std::vector<float> right_ft_tx_list;
  std::vector<float> right_ft_ty_list;
  std::vector<float> right_ft_tz_list;

  int left_window_size = 0;
  int right_window_size = 0;

  //----------------------------------------
  bool print_debug = false;
  bool send_camera_to_station = false;
  bool send_lidar_to_station = true;
  bool send_arm_to_station = true;

  int frame_skip = 0;
  float camera_resize_ratio = 0.3;
  int jpeg_compression_ratio = 50;
  int frame_skip_ratio = 1;
  int robot_navigation_control_mode = 0; //0 Teleoperation - 1 Autonomous

  // int frame_skip = 0;
  // float camera_resize_ratio = 0.2;
  // int jpeg_compression_ratio = 90;
  // int frame_skip_ratio = 1;

  //----------------------------------------

  int home_wait = 0;
  int test_time = 0;

  bool right_arm_updated = false;
  bool left_arm_updated = false;
  bool head_updated = false;
  bool right_gripper_updated = false;
  bool left_gripper_updated = false;
  bool z_axes_updated = false;
  bool battery_updated = false;

  void publishDataReduction();
  void compressAndSend(const sensor_msgs::Image::ConstPtr &msg);
  void compressAndSend2(const sensor_msgs::Image::ConstPtr &msg);
  void callbackLaser(const sensor_msgs::LaserScan::ConstPtr &msg);
  void callbackCamera(const sensor_msgs::Image::ConstPtr &msg);
  void callbackBattery(const movo_msgs::Battery::ConstPtr &msg);
 
  //Arm and Grippers
  void callbackRightGripper(const sensor_msgs::JointState::ConstPtr &msg);
  void callbackLeftGripper(const sensor_msgs::JointState::ConstPtr &msg);
  void callbackRightArm(const sensor_msgs::JointState::ConstPtr &msg);
  void callbackLeftArm(const sensor_msgs::JointState::ConstPtr &msg);
  void callbackHead(const sensor_msgs::JointState::ConstPtr &msg);
  void callbackZAxes(const sensor_msgs::JointState::ConstPtr &msg);

  void callbackGroundstation(MovoCommand cmd);
  void callbackHapticRight(HapticCommand cmd);
  void callbackHapticLeft(HapticCommand cmd);
  void callbackML(HapticCommand cmd);

  void playVoice(std::string text);
  void sendGoal(std::string text);
  void sendForceRight(HapticRender cmd);
  void sendForceLeft(HapticRender cmd);

  void callbackHapticRenderRightArm(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  void callbackHapticRenderLeftArm(const geometry_msgs::WrenchStamped::ConstPtr &msg);
  void callbackToolPos(const kinova_msgs::KinovaPose::ConstPtr &msg);
  
  void callbackClusters(const joint_control::ArmTrajectory::ConstPtr &msg);

  void sendHapticRenderRightArm(joint_control::HapticCommand msg);
  void sendHapticRenderLeftArm(joint_control::HapticCommand msg);
  void callbackBlock(const nav_msgs::Odometry::ConstPtr &msg);

  void callbackeefPoseRight(const geometry_msgs::Pose::ConstPtr &msg);
  void callbackeefPoseLeft(const geometry_msgs::Pose::ConstPtr &msg);
  
  sensor_msgs::JointState current_z_axes_state;
  sensor_msgs::JointState current_right_arm_state;
  sensor_msgs::JointState current_left_arm_state;
  sensor_msgs::JointState current_right_gripper_state;
  sensor_msgs::JointState current_left_gripper_state;
  sensor_msgs::JointState current_head_state;

  ros::ServiceClient cl_arm_home;
  ros::ServiceClient cl_arm_mode_none;
  ros::ServiceClient cl_arm_mode_autonomous;
  ros::ServiceClient cl_arm_mode_direct;
  ros::ServiceClient cl_arm_mode_shared;

  HapticRender msg_render_offset;

  std::mutex laser_lock;
  ros::Time last_cmd_vel_time;  
  ros::Time last_left_arm_time;
  ros::Time last_right_arm_time;
  bool arm_valid = false;
  bool set_stop;

  bool get_new;
  bool mutex;
  bool app_exit;
  int counter;
  int send_counter;
  std::string config_path = "";
  std::string old_cmd = "";
  YAML::Node m_config;
  settings m_settings;

  void sendOmni(double x,double y ,double w);
  void sendHead(double x,double y, double z);
  void sendObservation(ObservationRL data);
  void sendZ(int x);
  bool is_file_exist(const char *fileName);
  bool loadYaml();
  bool saveYaml();

  std::mutex mtx_z_axes;
  std::mutex mtx_right_arm;
  std::mutex mtx_left_arm;
  std::mutex mtx_right_gripper;
  std::mutex mtx_left_gripper;
  std::mutex mtx_head;
  std::mutex mtx_battery;
  std::mutex mtx_eff_right;
  std::mutex mtx_eff_left;

  float block_x = 0;
  float block_y = 0;
  float block_q1 = 0;
  float block_q2 = 0;
  float block_q3 = 0;
  float block_q4 = 0;

  float hand_pose_x = 0;
  float hand_pose_y = 0;
  float hand_pose_z = 0;
  float hand_rot_x = 0;
  float hand_rot_y = 0;
  float hand_rot_z = 0;

  geometry_msgs::Pose current_mpc_eef_right;
  geometry_msgs::Pose current_mpc_eef_left;

  double value = 0;
  double currentSample[7] = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
  double updatedSample[7] = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
  bool send_flag = true;
  double network_data_rate = 0;
  double pct = 0.007; 
  //double pct = 0; 
  bool enable_log = false;

  DeadbandDataReduction* Perceptionthresholds; 
  WriteParameter* wp;

  std::chrono::time_point<std::chrono::system_clock> time_start;
  std::chrono::time_point<std::chrono::system_clock> time_end;
  std::time_t old_delta_time;
  double data_rate_ps = 0;

  float force_network_rate = 0;
  float force_send = 0;
  float force_value = 0;
  
  float velocity_network_rate = 0;
  float velocity_send = 0;
  float velocity_value = 0;

  const static int order = 2; // 4th order (=2 biquads)
  const float samplingrate = 50; // Hz
  const float cutoff_frequency = 1; // Hz

  Iir::Butterworth::LowPass<order>* filter_right_fx;
  Iir::Butterworth::LowPass<order>* filter_right_fy;
  Iir::Butterworth::LowPass<order>* filter_right_fz;
  Iir::Butterworth::LowPass<order>* filter_right_tx;
  Iir::Butterworth::LowPass<order>* filter_right_ty;
  Iir::Butterworth::LowPass<order>* filter_right_tz;
  Iir::Butterworth::LowPass<order>* filter_left_fx;
  Iir::Butterworth::LowPass<order>* filter_left_fy;
  Iir::Butterworth::LowPass<order>* filter_left_fz;
  Iir::Butterworth::LowPass<order>* filter_left_tx;
  Iir::Butterworth::LowPass<order>* filter_left_ty;
  Iir::Butterworth::LowPass<order>* filter_left_tz;

};

} 

#endif 
