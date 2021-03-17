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
#include "sensor_msgs/Joy.h"
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

class BenchmarkROS 
{
public:
  BenchmarkROS(ros::NodeHandle &nh,
              ros::NodeHandle &pnh,
              int argc,
              char *argv[]);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr MatToPoinXYZ();

  std::vector<char> callbackDataLidar(std::vector<char> buffer, uint64_t priority, std::string sender);
  std::vector<char> callbackDataCameraColor(std::vector<char> buffer, uint64_t priority, std::string sender);
  std::vector<char> callbackDataCameraDepth(std::vector<char> buffer, uint64_t priority, std::string sender);
  std::vector<char> callbackDataIMU(std::vector<char> buffer, uint64_t priority, std::string sender);
  std::vector<char> callbackDataPoints(std::vector<char> buffer, uint64_t priority, std::string sender);
  
  void callback(rrs_ros::ParamConfig &config, uint32_t level);

  std::shared_ptr<Net2Subscriber> subscriber_lidar;
  std::shared_ptr<Net2Subscriber> subscriber_camera_color;
  std::shared_ptr<Net2Subscriber> subscriber_camera_depth;
  std::shared_ptr<Net2Subscriber> subscriber_imu;

  ros::Publisher pub_lidar;
  ros::Publisher pub_camera_color;
  ros::Publisher pub_camera_depth;
  ros::Publisher pub_imu;
  ros::Publisher pub_camera_point;

  ros::Subscriber sub_ros_sharp_lidar;
  ros::Subscriber sub_ros_sharp_camera_color;
  ros::Subscriber sub_ros_sharp_camera_depth;
  ros::Subscriber sub_ros_sharp_joy;

  ros::Subscriber sub_ros_2_lidar;
  ros::Subscriber sub_ros_2_camera_color;
  ros::Subscriber sub_ros_2_camera_depth;
  ros::Subscriber sub_ros_2_joy;

  void chatterCallbackImageColor(const sensor_msgs::CompressedImage::ConstPtr& msg);
  void chatterCallbackImageDepth(const sensor_msgs::CompressedImage::ConstPtr& msg);
  void chatterCallbackJoy(const sensor_msgs::Joy::ConstPtr& msg);
  void chatterCallbackLaser(const sensor_msgs::LaserScan::ConstPtr& msg);

  void chatterCallbackImageColorRRROS2(const sensor_msgs::CompressedImage::ConstPtr& msg);
  void chatterCallbackImageDepthRRROS2(const sensor_msgs::CompressedImage::ConstPtr& msg);
  void chatterCallbackJoyRRROS2(const sensor_msgs::Joy::ConstPtr& msg);
  void chatterCallbackLaserRRROS2(const sensor_msgs::LaserScan::ConstPtr& msg);

  cv::Mat last_color_frame;
  bool last_color_frame_updated = false;

  cv::Mat last_depth_frame;
  bool last_depth_frame_updated = false;

  uint64_t last_depth_frame_time = 0;

  double a = 0;

  float p_distance = 1;
  float p_fx = 550.0;
  float p_fy = 650.0;
  float p_cx = 400.0;
  float p_cy = 300.0;

  void publishLidar(char* data, int size,uint64_t xtime);
  void publishIMU(char* data, int size,uint64_t xtime);
  void publishCameraDepth(char* data, int size,uint64_t xtime);
  void publishCameraColor(char* data, int size,uint64_t xtime);
  void publishPointCloud();

  bool is_file_exist(const char *fileName);
  bool loadYaml();
  bool saveYaml();

  std::string config_path = "";
  YAML::Node m_config;
  settings m_settings;

  uint64_t delta_t_imu = 0;
  uint64_t delta_t_camera_rgb = 0;
  uint64_t delta_t_camera_depth = 0;
  uint64_t delta_t_point_cloud = 0;
  uint64_t delta_t_lidar = 0;

  uint64_t delta_rs_t_joy = 0;
  uint64_t delta_rs_t_camera_rgb = 0;
  uint64_t delta_rs_t_camera_depth = 0;
  uint64_t delta_rs_t_point_cloud = 0;
  uint64_t delta_rs_t_lidar = 0;
  
  uint64_t delta_ros2_t_joy = 0;
  uint64_t delta_ros2_t_camera_rgb = 0;
  uint64_t delta_ros2_t_camera_depth = 0;
  uint64_t delta_ros2_t_point_cloud = 0;
  uint64_t delta_ros2_t_lidar = 0;

  int size_imu = 0;
  int size_camera_rgb = 0;
  int size_camera_depth = 0;
  int size_point_cloud = 0;
  int size_lidar = 0;

  void update();
  int test_step = 0;
  ~BenchmarkROS();
  void kill();
  Net2 *net2;
  int benchmark_index = 0;
};

} // namespace rrs

#endif /* _NET2_TEST_ROS_HH_ */
