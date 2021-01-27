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

using namespace std;
using namespace lmt::Tools::Network;

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

class Net2TestROS 
{
public:
  Net2TestROS(ros::NodeHandle &nh,
              ros::NodeHandle &pnh,
              int argc,
              char *argv[]);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr MatToPoinXYZ();

  std::vector<char> callbackDataLidar1(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataLidar2(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataGroundtruth(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataCameraColor(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataCameraDepth(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataCameraNormal(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataCameraSegment(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataCameraInfo(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataTags(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataPoints(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataNavGoal(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataJointState(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataIMU(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataOdometry(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataTF(std::vector<char> buffer, unsigned int priority, std::string sender);
  void callback(rrs_ros::ParamConfig &config, uint32_t level);

  std::shared_ptr<Net2Publisher> publisher_cmd_vel;
  std::shared_ptr<Net2Publisher> publisher_planner_viz;
  std::shared_ptr<Net2Publisher> publisher_navigation_state;
  std::shared_ptr<Net2Publisher> publisher_rrs_command;
  std::shared_ptr<Net2Publisher> publisher_joint_command;

  std::shared_ptr<Net2Subscriber> subscriber_lidar_1;
  std::shared_ptr<Net2Subscriber> subscriber_lidar_2;
  std::shared_ptr<Net2Subscriber> subscriber_camera_color;
  std::shared_ptr<Net2Subscriber> subscriber_camera_depth;
  std::shared_ptr<Net2Subscriber> subscriber_camera_segment;
  std::shared_ptr<Net2Subscriber> subscriber_camera_normal;
  std::shared_ptr<Net2Subscriber> subscriber_camera_info;
  std::shared_ptr<Net2Subscriber> subscriber_groundtruth;
  std::shared_ptr<Net2Subscriber> subscriber_tag_points;
  std::shared_ptr<Net2Subscriber> subscriber_desire_points;
  std::shared_ptr<Net2Subscriber> subscriber_navigation_goal;
  std::shared_ptr<Net2Subscriber> subscriber_joint_state;
  std::shared_ptr<Net2Subscriber> subscriber_tf;
  std::shared_ptr<Net2Subscriber> subscriber_imu;
  std::shared_ptr<Net2Subscriber> subscriber_odometry;

  ros::Publisher pub_lidar_1;
  ros::Publisher pub_lidar_2;
  ros::Publisher pub_camera_color;
  ros::Publisher pub_camera_normal;
  ros::Publisher pub_camera_depth;
  ros::Publisher pub_camera_segment;
  ros::Publisher pub_camera_info;
  ros::Publisher pub_groundtruth;
  ros::Publisher pub_desire_points;
  ros::Publisher pub_tag_points;
  ros::Publisher pub_navigation_goal;
  ros::Publisher pub_imu;
  ros::Publisher pub_odometry;
  ros::Publisher pub_joint_state;
  ros::Publisher pub_camera_point; 
  
  ros::Subscriber sub_cmd_vel;
  ros::Subscriber sub_markers;
  ros::Subscriber sub_markers_goal;
  ros::Subscriber sub_markers_goal_arrow;
  ros::Subscriber sub_navigation_status;
  ros::Subscriber sub_rrs_command;
  ros::Subscriber sub_joint_command;

  std::shared_ptr<Net2Service> service;
  std::shared_ptr<Net2Client> client;
  std::shared_ptr<Net2AsyncClient> async_client;

  RRSRobot robot_protocol;
  movo_msgs::JacoJointCmd test_joint_command;

  cv::Mat last_color_frame;
  bool last_color_frame_updated = false;

  cv::Mat last_depth_frame;
  bool last_depth_frame_updated = false;

  double a = 0;

  dynamic_reconfigure::Server<rrs_ros::ParamConfig> server;
  dynamic_reconfigure::Server<rrs_ros::ParamConfig>::CallbackType f;

  float p_distance = 1;
  float p_fx = 550.0;
  float p_fy = 650.0;
  float p_cx = 400.0;
  float p_cy = 300.0;

  void chatterCallbackMarker(const visualization_msgs::Marker::ConstPtr& msg);
  void chatterCallbackRRSCommand(const std_msgs::String::ConstPtr& msg);
  void chatterCallbackNavigationStatus(const std_msgs::String::ConstPtr& msg);
  void chatterCallbackMarkerGoalArrow(const visualization_msgs::Marker::ConstPtr& msg);
  void chatterCallbackMarkerGoal(const visualization_msgs::Marker::ConstPtr& msg);
  void chatterCallbackCMD(const geometry_msgs::Twist::ConstPtr& msg);
  void chatterCallbackJointCommand(const movo_msgs::JacoJointCmd::ConstPtr& msg);

  void publishLidar1(char* data, int size);
  void publishLidar2(char* data, int size);
  void publishIMU(char* data, int size);
  void publishOdometry(char* data, int size);
  void publishCameraDepth(char* data, int size);
  void publishCameraColor(char* data, int size);
  void publishCameraInfo(char* data, int size);
  void publishCameraSegment(char* data, int size);
  void publishCameraNormal(char* data, int size);
  void publishGroundtruth(char* data, int size);
  void publishMapTags(char* data, int size);
  void publishMapPoints(char* data, int size);
  void publishNavigationGoal(char* data, int size);
  void publishJointState(char* data, int size);
  void publishTF(char* data, int size);
  void publishPointCloud();

  tf::TransformBroadcaster tf_broadcasters[1];
  tf::TransformBroadcaster odom_broadcaster;

  bool is_file_exist(const char *fileName);
  bool loadYaml();
  bool saveYaml();

  std::string config_path = "";
  YAML::Node m_config;
  settings m_settings;

  void update();
  int test_step = 0;
  ~Net2TestROS();
  void kill();
  Net2 *net2;


};

} // namespace rrs

#endif /* _NET2_TEST_ROS_HH_ */
