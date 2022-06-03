#ifndef _NET2_TEST_ROS_HH_
#define _NET2_TEST_ROS_HH_

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/subscriber.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <sstream>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <memory>

#include <dynamic_reconfigure/server.h>
#include <std_srvs/Empty.h>
#include <mutex>

//Net2
#include <Net2/Net2.h>
#include <Net2/Net2Base.h>

//Objects
#include <Net2/Net2Publisher.h>
#include <Net2/Net2Subscirber.h>
#include <Net2/Net2Service.h>
#include <Net2/Net2Client.h>
#include <Net2/Net2AsyncClient.h>

#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>

#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "movo_msgs/JacoJointCmd.h"

#include <yaml-cpp/yaml.h>
#include <mutex>

#include <dynamic_reconfigure/server.h>
#include <rrs_ros/ParamConfig.h>
#include "rrs_ros/PointList.h"

//Protobuf
#include <Scene.pb.h>
#include <movo_msgs/JacoAngularVelocityCmd7DOF.h>

#include <cpd/rigid.hpp>
#include <cpd/utils.hpp>
#include <cpd/nonrigid.hpp>
#include <cpd/affine.hpp>

#include <Net1/network.hh>

using namespace std;
using namespace lmt::Tools::Network;
using namespace roboland;

namespace lmt
{

struct settings
{
  std::string consul_network_address = "10.147.20.149";
  std::string local_network_address = "10.147.20.149";
  std::string consul_network_mask = "255.255.255.0";
  std::string consul_network_port = "8500";
  std::string ntp_server_host_name = "test";
};

class Net2TestROSCPD 
{
public:
  Net2TestROSCPD(ros::NodeHandle &nh,
              ros::NodeHandle &pnh,
              int argc,
              char *argv[]);

  void receiveCallback(char * data,int size,int index);
  void receiveCallbackCPD(char * data,int size,int index);

  std::vector<char> callbackDataNMPCLeftMarker(std::vector<char> buffer, uint64_t priority, std::string sender);
  std::vector<char> callbackDataNMPCRightMarker(std::vector<char> buffer, uint64_t priority, std::string sender);
  std::vector<char> callbackDataCameraColor(std::vector<char> buffer, uint64_t priority, std::string sender);
  std::vector<char> callbackDataCameraInfo(std::vector<char> buffer, uint64_t priority, std::string sender);
  std::vector<char> callbackDataCPDx(std::vector<char> buffer, uint64_t priority, std::string sender);
  void chatterCallbackVelLeft(const movo_msgs::JacoAngularVelocityCmd7DOF::ConstPtr& msg);
  void chatterCallbackVelRight(const movo_msgs::JacoAngularVelocityCmd7DOF::ConstPtr& msg);
  void initCPD();

  std::string path = "/home/rrs/workspace/src/CPD/cpd/tests/fixtures/";
  cpd::Matrix fixed;
  cpd::Matrix moving;

  std::shared_ptr<Net2Publisher> publisher_cpd;
  std::shared_ptr<Net2Publisher> publisher_joint_command_left;
  std::shared_ptr<Net2Publisher> publisher_joint_command_right;

  std::shared_ptr<Net2Subscriber> subscriber_cpd;
  std::shared_ptr<Net2Subscriber> subscriber_camera_color;
  std::shared_ptr<Net2Subscriber> subscriber_camera_info;
  std::shared_ptr<Net2Subscriber> subscriber_joint_state;
  std::shared_ptr<Net2Subscriber> subscriber_nmpc_right_marker;
  std::shared_ptr<Net2Subscriber> subscriber_nmpc_left_marker;

  ros::Publisher pub_camera_color;
  ros::Publisher pub_camera_info;
  ros::Publisher pub_joint_state;
  ros::Publisher pub_joint_state_right;
  ros::Publisher pub_joint_state_left;
  ros::Publisher pub_right_end_effector;
  ros::Publisher pub_right_end_effector_stamp;
  ros::Publisher pub_left_end_effector;
  ros::Publisher pub_left_end_effector_stamp;

  ros::Subscriber sub_rrs_command;
  ros::Subscriber sub_joint_command;

  std::shared_ptr<Net2Service> service;
  std::shared_ptr<Net2Client> client;
  std::shared_ptr<Net2AsyncClient> async_client;

  ros::Subscriber sub_jaco_right_vel;
  ros::Subscriber sub_jaco_left_vel;

  RRSRobot robot_protocol;

  cpd::Rigid rigid;
  cpd::Nonrigid nonrigid;
  cpd::Affine affine;

  cv::Mat last_color_frame;
  bool last_color_frame_updated = false;

  cv::Mat last_depth_frame;
  bool last_depth_frame_updated = false;

  double a = 0;
  bool cpd_busy = false;

  void chatterCallbackMarker(const visualization_msgs::Marker::ConstPtr& msg);
  void chatterCallbackRRSCommand(const std_msgs::String::ConstPtr& msg);
  void chatterCallbackNavigationStatus(const std_msgs::String::ConstPtr& msg);
  void chatterCallbackMarkerGoalArrow(const visualization_msgs::Marker::ConstPtr& msg);
  void chatterCallbackMarkerGoal(const visualization_msgs::Marker::ConstPtr& msg);
  void chatterCallbackCMD(const geometry_msgs::Twist::ConstPtr& msg);
  void chatterCallbackJointCommand(const movo_msgs::JacoJointCmd::ConstPtr& msg);
  std::vector<char> callbackDataJointState(std::vector<char> buffer, uint64_t priority, std::string sender);

  void publishCameraColor(char* data, int size);
  void publishCameraInfo(char* data, int size);
  void publishJointState(char* data, int size);

  bool is_file_exist(const char *fileName);
  bool loadYaml();
  bool saveYaml();

  std::string config_path = "";
  YAML::Node m_config;
  settings m_settings;

  void update();
  int test_step = 0;
  ~Net2TestROSCPD();
  void kill();
  Net2 *net2;

  Network *net_interface;  
  Network *net_interface_cpd;  


};

} // namespace rrs

#endif /* _NET2_TEST_ROS_HH_ */
