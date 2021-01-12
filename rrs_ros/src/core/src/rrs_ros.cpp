#include "rrs_ros.hh"

namespace roboland
{

  Net2TestROS::Net2TestROS(ros::NodeHandle &nh,
   ros::NodeHandle &pnh,
   int argc,
   char *argv[])
  {
    ROS_INFO("NET2 CPP ROS TEST HAS STARTED");

    f = boost::bind(&Net2TestROS::callback,this, _1, _2);
    server.setCallback(f);

    std::string path = ros::package::getPath("rrs_ros");
    path = path + "/cfg/config.yaml";

    config_path = path;

    ROS_INFO_STREAM("Config path is : " << config_path);

    //Creating the config file for first use or load it 
    loadYaml();
    saveYaml();

    //Config Print
    ROS_INFO_STREAM("consul network address :" << m_settings.consul_network_address );
    ROS_INFO_STREAM("local network address :" << m_settings.local_network_address );
    ROS_INFO_STREAM("consul network mask :" << m_settings.consul_network_mask );
    ROS_INFO_STREAM("consul network port :" << m_settings.consul_network_port);
    ROS_INFO_STREAM("ntp server host name : " << m_settings.ntp_server_host_name);

    //Config
    Net2Config config;

    config.consul_network_address = m_settings.consul_network_address;
    config.local_network_address = m_settings.local_network_address;
    config.consul_network_mask = m_settings.consul_network_mask;
    config.consul_network_port = m_settings.consul_network_port;
    config.ntp_server_host_name = m_settings.ntp_server_host_name;
    config.consul_mode = CLIENT;

    net2 = new Net2();

    //Init Net2 with config, namespace and station names
    this->net2->Init(config,"rrs_ros","test");

    this->test_step = 1;

    //RRS High Performance Bridge
    subscriber_lidar_1 = net2->subscriber();
    subscriber_lidar_1->delegateNewData = std::bind(&Net2TestROS::callbackDataLidar1, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result1 = subscriber_lidar_1->Start("rrs-lidar_front");

    subscriber_lidar_2 = net2->subscriber();
    subscriber_lidar_2->delegateNewData = std::bind(&Net2TestROS::callbackDataLidar2, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result2 = subscriber_lidar_2->Start("rrs-lidar_rear");

    subscriber_camera_color = net2->subscriber();
    subscriber_camera_color->delegateNewData = std::bind(&Net2TestROS::callbackDataCameraColor, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result3 = subscriber_camera_color->Start("rrs-camera_color");

    subscriber_camera_depth = net2->subscriber();
    subscriber_camera_depth->delegateNewData = std::bind(&Net2TestROS::callbackDataCameraDepth, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result4 = subscriber_camera_depth->Start("rrs-camera_depth");

    subscriber_camera_normal = net2->subscriber();
    subscriber_camera_normal->delegateNewData = std::bind(&Net2TestROS::callbackDataCameraNormal, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result44 = subscriber_camera_normal->Start("rrs-camera_normal");

    subscriber_camera_info = net2->subscriber();
    subscriber_camera_info->delegateNewData = std::bind(&Net2TestROS::callbackDataCameraInfo, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result5 = subscriber_camera_info->Start("rrs-camera_info");

    subscriber_camera_segment = net2->subscriber();
    subscriber_camera_segment->delegateNewData = std::bind(&Net2TestROS::callbackDataCameraSegment, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result6 = subscriber_camera_segment->Start("rrs-camera_segment");

    subscriber_groundtruth = net2->subscriber();
    subscriber_groundtruth->delegateNewData = std::bind(&Net2TestROS::callbackDataGroundtruth, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result7 = subscriber_groundtruth->Start("rrs-groundtruth");

    subscriber_desire_points = net2->subscriber();
    subscriber_desire_points->delegateNewData = std::bind(&Net2TestROS::callbackDataPoints, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result8 = subscriber_desire_points->Start("rrs-map_points");

    subscriber_tag_points = net2->subscriber();
    subscriber_tag_points->delegateNewData = std::bind(&Net2TestROS::callbackDataTags, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result9 = subscriber_tag_points->Start("rrs-map_tags");

    subscriber_navigation_goal = net2->subscriber();
    subscriber_navigation_goal->delegateNewData = std::bind(&Net2TestROS::callbackDataNavGoal, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result10 = subscriber_navigation_goal->Start("rrs-nav_goal");

    subscriber_joint_state = net2->subscriber();
    subscriber_joint_state->delegateNewData = std::bind(&Net2TestROS::callbackDataJointState, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result11 = subscriber_joint_state->Start("rrs-joint_state");

    subscriber_imu = net2->subscriber();
    subscriber_imu->delegateNewData = std::bind(&Net2TestROS::callbackDataIMU, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result12 = subscriber_imu->Start("rrs-imu");

    subscriber_odometry = net2->subscriber();
    subscriber_odometry->delegateNewData = std::bind(&Net2TestROS::callbackDataOdometry, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result13 = subscriber_odometry->Start("rrs-odometry");

    subscriber_tf = net2->subscriber();
    subscriber_tf->delegateNewData = std::bind(&Net2TestROS::callbackDataTF, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result14 = subscriber_tf->Start("rrs-tf");

    publisher_cmd_vel = net2->publisher("cmd_vel");
    publisher_cmd_vel->Start();

    publisher_planner_viz = net2->publisher("planner_viz");
    publisher_planner_viz->Start();

    publisher_joint_command = net2->publisher("joint_command");
    publisher_joint_command->Start();

    publisher_rrs_command = net2->publisher("rrs_command");
    publisher_rrs_command->Start();

    publisher_navigation_state = net2->publisher("navigation_state");
    publisher_navigation_state->Start();

    //ROS
    pub_lidar_1 = nh.advertise<sensor_msgs::LaserScan>("movo/front_scan", 1);
    pub_lidar_2 = nh.advertise<sensor_msgs::LaserScan>("movo/rear_scan", 1);
    pub_camera_color = nh.advertise<sensor_msgs::Image>("movo/camera/image_raw", 1);
    pub_camera_info = nh.advertise<sensor_msgs::CameraInfo>("movo/camera/camera_info", 1);
    pub_camera_depth = nh.advertise<sensor_msgs::Image>("movo/camera/depth_raw", 1);
    pub_camera_normal = nh.advertise<sensor_msgs::Image>("movo/camera/normal_raw", 1);
    pub_camera_segment = nh.advertise<sensor_msgs::Image>("movo/camera/segment_raw", 1);
    pub_odometry = nh.advertise<nav_msgs::Odometry>("movo/odometry",1);
    pub_imu = nh.advertise<sensor_msgs::Imu>("movo/imu",1);
    pub_camera_point = nh.advertise<pcl::PCLPointCloud2>("movo/camera/points", 1);
    
    pub_groundtruth = nh.advertise<geometry_msgs::PoseStamped>("movo/groundtruth",1);
    pub_desire_points = nh.advertise<rrs_ros::PointList>("points",1);
    pub_tag_points = nh.advertise<rrs_ros::PointList>("tags",1);
    pub_navigation_goal = nh.advertise<geometry_msgs::PoseStamped>("goal",1);
    pub_joint_state = nh.advertise<sensor_msgs::JointState>("rrs/joint_states",1);

    sub_cmd_vel = nh.subscribe("cmd_vel",1, &Net2TestROS::chatterCallbackCMD, this);
    sub_navigation_status = nh.subscribe("navigation/status",1, &Net2TestROS::chatterCallbackNavigationStatus, this);
    sub_rrs_command = nh.subscribe("rrs/scene/command",1, &Net2TestROS::chatterCallbackRRSCommand, this);
    sub_joint_command = nh.subscribe("rrs/joint_command",1, &Net2TestROS::chatterCallbackJointCommand, this);
    sub_markers = nh.subscribe("visualization_marker_steps",1,&Net2TestROS::chatterCallbackMarker, this);
    sub_markers_goal_arrow = nh.subscribe("visualization_marker_goals_arrow",1,&Net2TestROS::chatterCallbackMarkerGoalArrow, this);
    sub_markers_goal = nh.subscribe("visualization_marker_goals",1,&Net2TestROS::chatterCallbackMarkerGoal, this);

    for ( int i = 0 ; i < 7 ; i++)
    {
      test_joint_command.joint_cmds.push_back(0);
    }
  }
  
  void Net2TestROS::chatterCallbackJointCommand(const movo_msgs::JacoJointCmd::ConstPtr& msg)
  {
    RRSJointCommand cmd;
    int size = msg->joint_cmds.size();
   
    for ( int i = 0 ; i < size ; i++)
    {
      cmd.add_goal(msg->joint_cmds[i]);
    }

    int bsize = cmd.ByteSize();
    char buffer[bsize];
    cmd.SerializeToArray(buffer,bsize);

    publisher_joint_command->send(buffer,bsize,1);
  }

  void Net2TestROS::chatterCallbackMarker(const visualization_msgs::Marker::ConstPtr& msg)
  {
    robot_protocol.clear_path();

    for ( int i= 0 ; i < msg->points.size() ; i++ )
    {
      robot_protocol.add_path();
      robot_protocol.mutable_path(i)->set_x(msg->points.at(i).x);
      robot_protocol.mutable_path(i)->set_y(msg->points.at(i).y);
      robot_protocol.mutable_path(i)->set_theta(msg->points.at(i).z);
    }

    int size = robot_protocol.ByteSize();
    char buffer[size];
    robot_protocol.SerializeToArray(buffer,size);

    publisher_planner_viz->send(buffer,size,1);

    //ROS_INFO_STREAM("Get marker points" << msg->points.size() );
  }

  void Net2TestROS::chatterCallbackMarkerGoalArrow(const visualization_msgs::Marker::ConstPtr& msg)
  {
    //Later
  }

  void Net2TestROS::chatterCallbackMarkerGoal(const visualization_msgs::Marker::ConstPtr& msg)
  {
    //Later
  }

  void Net2TestROS::chatterCallbackRRSCommand(const std_msgs::String::ConstPtr& msg)
  {
    //Send Scene Command to RRS
  }

  void Net2TestROS::chatterCallbackNavigationStatus(const std_msgs::String::ConstPtr& msg)
  {
    //Send status to RRS
  }

  bool Net2TestROS::loadYaml()
  {
    try
    {
      if ( is_file_exist(config_path.c_str()) )
      {
       m_config = YAML::LoadFile(config_path.c_str());

       m_settings.consul_network_address = m_config["consul_network_address"].as<std::string>();
       m_settings.local_network_address = m_config["local_network_address"].as<std::string>();
       m_settings.consul_network_mask = m_config["consul_network_mask"].as<std::string>();
       m_settings.consul_network_port = m_config["consul_network_port"].as<std::string>();
       m_settings.ntp_server_host_name = m_config["ntp_server_host_name"].as<std::string>();

       return true;
     }
     else
     {
      ROS_WARN("Config file is not exist");
    }
  }
  catch (...)
  {
   ROS_ERROR("Could not parse YAML config, or not exist");
 }

 return false;
}

bool Net2TestROS::is_file_exist(const char *fileName)
{
  std::ifstream infile(fileName);
  return infile.good();
}

bool Net2TestROS::saveYaml()
{
  try
  {

    std::ofstream fout(config_path.c_str()); 

    m_config["consul_network_address"] = m_settings.consul_network_address;
    m_config["local_network_address"] = m_settings.local_network_address;
    m_config["consul_network_mask"] = m_settings.consul_network_mask;
    m_config["consul_network_port"] = m_settings.consul_network_port;
    m_config["ntp_server_host_name"] = m_settings.ntp_server_host_name;


    fout << m_config; 

    fout.flush();
    fout.close();

    return true;

  }
  catch (...)
  {
    ROS_ERROR("Could not save YAML config");
  }

  return false;
}

void Net2TestROS::chatterCallbackCMD(const geometry_msgs::Twist::ConstPtr& msg)
{
  //Convert ros to proto to data
 float x = msg->linear.x;
 float y = msg->linear.y;
 float w = msg->angular.z;

 //ROS_INFO_STREAM("Get cmd_vel " << x << " " << y << " " << w);
 RVector3 proto_msg;
 proto_msg.set_x(x);
 proto_msg.set_y(y);
 proto_msg.set_theta(w);

 int size = proto_msg.ByteSize();
 char buffer[size];
 proto_msg.SerializeToArray(buffer,size);

 publisher_cmd_vel->send(buffer,size,1);
}

void Net2TestROS::publishLidar1(char* data, int size)
{
  //1
 RSSLaser laser_msg;
 laser_msg.ParseFromArray(data,size);

 sensor_msgs::LaserScan scan_msg;

 scan_msg.header.stamp = ros::Time::now();
 scan_msg.header.frame_id = "front_laser_link";

 scan_msg.angle_min =  laser_msg.angel_min();
 scan_msg.angle_max =  laser_msg.angel_max();
 scan_msg.angle_increment = laser_msg.angel_increment();
 scan_msg.scan_time = laser_msg.scan_time();
 scan_msg.time_increment = laser_msg.time_increment();
 scan_msg.range_min = laser_msg.range_min();
 scan_msg.range_max = laser_msg.range_max();

 scan_msg.intensities.resize(laser_msg.ranges().size());
 scan_msg.ranges.resize(laser_msg.ranges().size());

 for (size_t i = 0; i < laser_msg.ranges().size(); i++) 
 {
  float read_value = laser_msg.ranges(i);

  if (read_value == 0.0)
    scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
  else
    scan_msg.ranges[i] = read_value;  
}

pub_lidar_1.publish(scan_msg);
}

void Net2TestROS::publishLidar2(char* data, int size)
{
  //2
 RSSLaser laser_msg;
 laser_msg.ParseFromArray(data,size);

 sensor_msgs::LaserScan scan_msg;

 scan_msg.header.stamp = ros::Time::now();
 scan_msg.header.frame_id = "rear_laser_link";

 scan_msg.angle_min =  laser_msg.angel_min();
 scan_msg.angle_max =  laser_msg.angel_max();
 scan_msg.angle_increment = laser_msg.angel_increment();
 scan_msg.scan_time = laser_msg.scan_time();
 scan_msg.time_increment = laser_msg.time_increment();
 scan_msg.range_min = laser_msg.range_min();
 scan_msg.range_max = laser_msg.range_max();

 scan_msg.intensities.resize(laser_msg.ranges().size());
 scan_msg.ranges.resize(laser_msg.ranges().size());

 for (size_t i = 0; i < laser_msg.ranges().size(); i++) 
 {
  float read_value = laser_msg.ranges(i);

  if (read_value == 0.0)
    scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
  else
    scan_msg.ranges[i] = read_value;  
}

pub_lidar_2.publish(scan_msg);
}

void Net2TestROS::publishCameraColor(char* data, int size)
{
  //3
	std::vector<unsigned char> data_byte;
	data_byte.assign(data,data + size);
	cv::Mat image(cv::imdecode(data_byte,1));

  if ( last_color_frame_updated == false)
  {
    this->last_color_frame = image.clone(); 
    last_color_frame_updated = true;
  }

	cv_bridge::CvImage out_msg;
	out_msg.encoding = sensor_msgs::image_encodings::BGR8;
	out_msg.image = image;
  out_msg.header.stamp = ros::Time::now();
  out_msg.header.frame_id = "kinect2_link";
  pub_camera_color.publish(out_msg.toImageMsg());
  
}

void Net2TestROS::publishCameraDepth(char* data, int size)
{
  //4
  std::vector<unsigned char> data_byte;
  data_byte.assign(data,data + size);
  cv::Mat image(cv::imdecode(data_byte,1));

  if ( last_depth_frame_updated == false)
  {
    this->last_depth_frame = image.clone(); 
    last_depth_frame_updated = true;
  }

  cv_bridge::CvImage out_msg;
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = image;
  out_msg.header.stamp = ros::Time::now();
  out_msg.header.frame_id = "kinect2_link";
  pub_camera_depth.publish(out_msg.toImageMsg());

}

void Net2TestROS::publishPointCloud()
{
  if ( last_color_frame_updated && last_depth_frame_updated)
  {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = MatToPoinXYZ();
      sensor_msgs::PointCloud2 point_cloud2;

      pcl::toROSMsg(*cloud, point_cloud2);

      point_cloud2.header.stamp = ros::Time::now();
      point_cloud2.header.frame_id = "kinect2_link";

      pub_camera_point.publish(point_cloud2);
      last_color_frame_updated = false;
      last_depth_frame_updated = false;
  }
}

void Net2TestROS::publishCameraNormal(char* data, int size)
{
  //4
  std::vector<unsigned char> data_byte;
  data_byte.assign(data,data + size);
  cv::Mat image(cv::imdecode(data_byte,1));
  cv_bridge::CvImage out_msg;
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = image;
  out_msg.header.stamp = ros::Time::now();
  out_msg.header.frame_id = "kinect2_link";
  pub_camera_normal.publish(out_msg.toImageMsg());
}

void Net2TestROS::callback(rrs_ros::ParamConfig &config, uint32_t level) 
{
  // ROS_INFO("Reconfigure Request: %f %f %f", 
  //           config.distance_param, 
  //           config.theta_h_param,
  //           config.theta_v_param);

            p_fx = config.fx_param;
            p_fy = config.fy_param;
            p_cx = config.cx_param;
            p_cy = config.cy_param;
            p_distance = config.distance_param;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Net2TestROS::MatToPoinXYZ()
{
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

      //Image h and w
      float rows = last_depth_frame.rows;
      float cols = last_depth_frame.cols;

      //default far and near clipping distance
      float zFar = 5;
      float zNear = 0.5;

      const double v0 = p_cx;
      const double u0 = p_cy;
      const double fy =  p_fy;
      const double fx = p_fx;

      cloud->is_dense = true;
     
      for (unsigned int u = 0; u < rows; ++u) 
      {
        for (unsigned int v = 0; v < cols; ++v) 
        {
            float Xw = 0, Yw = 0, Zw = 0;
            char R = 255 , G = 255 , B = 255;
          
            float d = last_depth_frame.at<cv::Vec3b>(u, v)[0];

            if (  d > 0 &&  d < 255 )
            {

            float distance = ( d / 255 ) * (zFar - zNear) + zNear ;
            distance = distance * p_distance;

            pcl::PointXYZRGB newPoint;

            Xw = (distance);

            // //calculate x-coordinate
            // float alpha_h = (M_PI - p_h) / 2;
            // float gamma_i_h = alpha_h + (float)v*(p_h / cols);
            // Yw = distance / tan(gamma_i_h);

            // //calculate y-coordinate
            // float alpha_v = 2 * M_PI - (p_v / 2);
            // float gamma_i_v = alpha_v + (float)u*(p_v / rows);
            // Zw = distance * tan(gamma_i_v)*-1;

            Yw = (float) ((v - v0) * Xw / fx);
            Zw = (float) ((u - u0) * Xw / fy);

            //ROS_INFO_STREAM(Xw << " " << Yw << " " << Zw);

            B = last_color_frame.at<cv::Vec3b>(u, v)[0];
            G = last_color_frame.at<cv::Vec3b>(u, v)[1];
            R = last_color_frame.at<cv::Vec3b>(u, v)[2];

            newPoint.x = Xw;
            newPoint.y = -Yw;
            newPoint.z = -Zw;
            newPoint.r = R;
            newPoint.g = G;
            newPoint.b = B;


            //Add the point to the cloud
            cloud->points.push_back(newPoint);

            }
          
        }
      }

    return cloud;
}

void Net2TestROS::publishCameraInfo(char* data, int size)
{
  //5
  RRSCameraInfo info_msg;
  info_msg.ParseFromArray(data,size);

  sensor_msgs::CameraInfo camera_info_msg;
  camera_info_msg.width = info_msg.width();
  camera_info_msg.height = info_msg.height();
  camera_info_msg.header.stamp = ros::Time::now();
  camera_info_msg.header.frame_id = "camera_link";

  for ( int i = 0 ; i < 5 ; i++)
  {
    camera_info_msg.D.push_back(info_msg.d(i));
  }

  for ( int i = 0 ; i < 9 ; i++)
  {
    camera_info_msg.K[i] = info_msg.k(i);
  }

  for ( int i = 0 ; i < 9 ; i++)
  {
    camera_info_msg.R[i] = info_msg.r(i);
  }

  for ( int i = 0 ; i < 12 ; i++)
  {
    camera_info_msg.P[i] = info_msg.p(i);
  }

  camera_info_msg.distortion_model = info_msg.distortion_model();

  pub_camera_info.publish(camera_info_msg);
}

void Net2TestROS::publishCameraSegment(char* data, int size)
{
  //6
  std::vector<unsigned char> data_byte;
  data_byte.assign(data,data + size);
  cv::Mat image(cv::imdecode(data_byte,1));
  cv_bridge::CvImage out_msg;
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = image;
  out_msg.header.stamp = ros::Time::now();
  out_msg.header.frame_id = "camera_link";
  pub_camera_segment.publish(out_msg.toImageMsg());
}

void Net2TestROS::publishGroundtruth(char* data, int size)
{
  //7
   RRSTransform transform_msg;
   transform_msg.ParseFromArray(data,size);

   geometry_msgs::PoseStamped gt_msg;

   gt_msg.header.stamp = ros::Time::now();
   gt_msg.header.frame_id = "map";

   gt_msg.pose.position.x = transform_msg.position().x();
   gt_msg.pose.position.y = transform_msg.position().y();
   gt_msg.pose.position.z = transform_msg.position().z();

   gt_msg.pose.orientation.x = transform_msg.orientation().x();
   gt_msg.pose.orientation.y = transform_msg.orientation().y();
   gt_msg.pose.orientation.z = transform_msg.orientation().z();
   gt_msg.pose.orientation.w = transform_msg.orientation().w();

   pub_groundtruth.publish(gt_msg);
}

void Net2TestROS::publishMapPoints(char* data, int size)
{
  //8
 RRSPointList points_msg;
 points_msg.ParseFromArray(data,size);

 rrs_ros::PointList ros_msg;

 ros_msg.header.stamp = ros::Time::now();

 for ( int i = 0 ; i < points_msg.points().size() ; i++)
 {
    geometry_msgs::Pose pose;

    pose.position.x = points_msg.points(i).position().x();
    pose.position.y = points_msg.points(i).position().y();
    pose.position.z = points_msg.points(i).position().z();

    pose.orientation.x = points_msg.points(i).orientation().x();
    pose.orientation.y = points_msg.points(i).orientation().y();
    pose.orientation.z = points_msg.points(i).orientation().z();
    pose.orientation.w = points_msg.points(i).orientation().w();

    ros_msg.poses.push_back(pose);
 }

 pub_desire_points.publish(ros_msg);
}

void Net2TestROS::publishMapTags(char* data, int size)
{
  //9
 RRSPointList points_msg;
 points_msg.ParseFromArray(data,size);

 rrs_ros::PointList ros_msg;

 ros_msg.header.stamp = ros::Time::now();

 for ( int i = 0 ; i < points_msg.points().size() ; i++)
 {
    geometry_msgs::Pose pose;

    pose.position.x = points_msg.points(i).position().x();
    pose.position.y = points_msg.points(i).position().y();
    pose.position.z = points_msg.points(i).position().z();

    pose.orientation.x = points_msg.points(i).orientation().x();
    pose.orientation.y = points_msg.points(i).orientation().y();
    pose.orientation.z = points_msg.points(i).orientation().z();
    pose.orientation.w = points_msg.points(i).orientation().w();

    ros_msg.poses.push_back(pose);
 }

 pub_tag_points.publish(ros_msg);
}

void Net2TestROS::publishNavigationGoal(char* data, int size)
{
  //10
 RRSNavGoal transform_msg;
 transform_msg.ParseFromArray(data,size);

 geometry_msgs::PoseStamped gt_msg;

 gt_msg.header.stamp = ros::Time::now();
 gt_msg.header.frame_id = "goal_link";

 ROS_INFO_STREAM("NAV GOAL " << transform_msg.transform().position().x() << " " << transform_msg.transform().position().y() << " " << transform_msg.transform().position().z());

 gt_msg.pose.position.x = transform_msg.transform().position().x();
 gt_msg.pose.position.y = transform_msg.transform().position().y();
 gt_msg.pose.position.z = transform_msg.transform().position().z();

 gt_msg.pose.orientation.x = transform_msg.transform().orientation().x();
 gt_msg.pose.orientation.y = transform_msg.transform().orientation().y();
 gt_msg.pose.orientation.z = transform_msg.transform().orientation().z();
 gt_msg.pose.orientation.w = transform_msg.transform().orientation().w();

 pub_navigation_goal.publish(gt_msg);
}


void Net2TestROS::publishJointState(char* data, int size)
{
  //11
  RRSJointState state_msg;
  state_msg.ParseFromArray(data,size);

  sensor_msgs::JointState ros_state_msg;
  
  for ( int i = 0 ; i < 19 ; i++)
  {
    ros_state_msg.name.push_back(state_msg.name(i));
    ros_state_msg.position.push_back(state_msg.position(i));
    ros_state_msg.velocity.push_back(state_msg.velocity(i));
    ros_state_msg.effort.push_back(state_msg.effort(i));
  }

  pub_joint_state.publish(ros_state_msg);
}


void Net2TestROS::publishOdometry(char* data, int size)
{
  //12
  RRSOdom transform_msg;
  transform_msg.ParseFromArray(data,size);

  //-
  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat;
  odom_quat.x = transform_msg.orientation().x();
  odom_quat.y = transform_msg.orientation().y();
  odom_quat.z = transform_msg.orientation().z();
  odom_quat.w = transform_msg.orientation().w();

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = "map";
  odom_trans.child_frame_id = "groundtruth";

  odom_trans.transform.translation.x = transform_msg.position().x();
  odom_trans.transform.translation.y = transform_msg.position().y();
  odom_trans.transform.translation.z = transform_msg.position().z();
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  //odom_broadcaster.sendTransform(odom_trans);

  //-

  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = transform_msg.position().x();
  odom.pose.pose.position.y = transform_msg.position().y();
  odom.pose.pose.position.z = transform_msg.position().z();
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = transform_msg.linear_speed().x();
  odom.twist.twist.linear.y = transform_msg.linear_speed().y();
  odom.twist.twist.angular.z = -transform_msg.angular_speed().z();

  //publish the message
  //pub_odometry.publish(odom);
}

void Net2TestROS::publishIMU(char* data, int size)
{
  //13
  RRSIMU imu_msg;
  imu_msg.ParseFromArray(data,size);

  sensor_msgs::Imu out_msg;

  out_msg.orientation.x = imu_msg.orientation().x();
  out_msg.orientation.y = imu_msg.orientation().y();
  out_msg.orientation.z = imu_msg.orientation().z();
  out_msg.orientation.w = imu_msg.orientation().w();

  for ( int i = 0 ; i < 9 ; i++)
  {
     out_msg.orientation_covariance[i] = imu_msg.orientation_covariance(i);
  }
 
  out_msg.angular_velocity.x = imu_msg.angular_velocity().x();
  out_msg.angular_velocity.y = imu_msg.angular_velocity().y();
  out_msg.angular_velocity.z = -imu_msg.angular_velocity().z();

  for ( int i = 0 ; i < 9 ; i++)
  {
     out_msg.angular_velocity_covariance[i] = imu_msg.angular_velocity_covariance(i);
  }
  
  out_msg.linear_acceleration.x = imu_msg.linear_acceleration().x();
  out_msg.linear_acceleration.y = imu_msg.linear_acceleration().y();
  out_msg.linear_acceleration.z = imu_msg.linear_acceleration().z();

  for ( int i = 0 ; i < 9 ; i++)
  {
     out_msg.linear_acceleration_covariance[i] = imu_msg.linear_acceleration_covariance(i);
  }

  out_msg.header.stamp = ros::Time::now();
  out_msg.header.frame_id = "sic_imu_frame"; 

  pub_imu.publish(out_msg);
}

void Net2TestROS::publishTF(char* data, int size)
{
  RRSTF tf_msg;
  tf_msg.ParseFromArray(data,size);

  tf::Transform transform;
  
  tf::Quaternion q(tf_msg.transforms(0).orientation().x(),tf_msg.transforms(0).orientation().y(),tf_msg.transforms(0).orientation().z(),tf_msg.transforms(0).orientation().w());
  transform.setOrigin( tf::Vector3(tf_msg.transforms(0).position().x(), tf_msg.transforms(0).position().y(), tf_msg.transforms(0).position().z()) );
  transform.setRotation(q);

  //ROS_WARN_STREAM("GET " << tf_msg.names(i));
  //tf_broadcasters[0].sendTransform(tf::StampedTransform(transform, ros::Time::now(), tf_msg.parents(0), tf_msg.names(0)));
  std::string name = tf_msg.names(0);   
 }

std::vector<char> Net2TestROS::callbackDataLidar1(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //1
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishLidar1(&buffer[0],buffer.size());
  return result;
}

std::vector<char> Net2TestROS::callbackDataLidar2(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //2
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishLidar2(&buffer[0],buffer.size());
  return result;
}

std::vector<char> Net2TestROS::callbackDataCameraColor(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //ROS_INFO("GET CAMERA COLOR");
  //3
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishCameraColor(&buffer[0],buffer.size());
  return result;
}

std::vector<char> Net2TestROS::callbackDataCameraDepth(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //4
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishCameraDepth(&buffer[0],buffer.size());
  return result;
}

std::vector<char> Net2TestROS::callbackDataCameraNormal(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //4r
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishCameraNormal(&buffer[0],buffer.size());
  return result;
}

std::vector<char> Net2TestROS::callbackDataCameraInfo(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //5
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishCameraInfo(&buffer[0],buffer.size());
  return result;
}

std::vector<char> Net2TestROS::callbackDataCameraSegment(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //6
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishCameraSegment(&buffer[0],buffer.size());
  return result;
}

std::vector<char> Net2TestROS::callbackDataGroundtruth(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //7
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishGroundtruth(&buffer[0],buffer.size());
  return result;
}

std::vector<char> Net2TestROS::callbackDataPoints(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //8
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishMapPoints(&buffer[0],buffer.size());
  return result;
}

std::vector<char> Net2TestROS::callbackDataTags(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //9
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishMapTags(&buffer[0],buffer.size());
  return result;
}

std::vector<char> Net2TestROS::callbackDataNavGoal(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //10
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishNavigationGoal(&buffer[0],buffer.size());
  return result;
}

std::vector<char> Net2TestROS::callbackDataJointState(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //11
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishJointState(&buffer[0],buffer.size());
  return result;
}

std::vector<char> Net2TestROS::callbackDataOdometry(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //12
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishOdometry(&buffer[0],buffer.size());
  return result;
}

std::vector<char> Net2TestROS::callbackDataIMU(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //13
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishIMU(&buffer[0],buffer.size());
  return result;
}

std::vector<char> Net2TestROS::callbackDataTF(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //14
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishTF(&buffer[0],buffer.size());
  return result;
}

void Net2TestROS::update()
{
   publishPointCloud();
}

void Net2TestROS::kill()
{
  this->publisher_cmd_vel->Stop();
  this->publisher_planner_viz->Stop();

  this->net2->Shutdown();
  ROS_INFO("RRS ROS TERMINATED");
}

Net2TestROS::~Net2TestROS()
{

}

} // namespace roboland
