#include "rrs_ros_benchmark.hh"

namespace lmt
{
  BenchmarkROS::BenchmarkROS(ros::NodeHandle &nh,
   ros::NodeHandle &pnh,
   int argc,
   char *argv[])
  {
    ROS_INFO("NET2 CPP ROS TEST HAS STARTED");

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
    subscriber_lidar = net2->subscriber();
    subscriber_lidar->delegateNewData = std::bind(&BenchmarkROS::callbackDataLidar, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result1 = subscriber_lidar->Start("rrs-lidar");

    subscriber_camera_color = net2->subscriber();
    subscriber_camera_color->delegateNewData = std::bind(&BenchmarkROS::callbackDataCameraColor, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result3 = subscriber_camera_color->Start("rrs-camera_color");

    subscriber_camera_depth = net2->subscriber();
    subscriber_camera_depth->delegateNewData = std::bind(&BenchmarkROS::callbackDataCameraDepth, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result4 = subscriber_camera_depth->Start("rrs-camera_depth");

    subscriber_imu = net2->subscriber();
    subscriber_imu->delegateNewData = std::bind(&BenchmarkROS::callbackDataIMU, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result12 = subscriber_imu->Start("rrs-imu");

    //ROS
    pub_lidar = nh.advertise<sensor_msgs::LaserScan>("net2/lidar", 1);
    pub_camera_color = nh.advertise<sensor_msgs::Image>("net2/camera/image_raw", 1);
    pub_camera_depth = nh.advertise<sensor_msgs::Image>("net2/camera/depth_raw", 1);
    pub_imu = nh.advertise<sensor_msgs::Imu>("net2/imu",1);
    pub_camera_point = nh.advertise<pcl::PCLPointCloud2>("net2/camera/points", 1);
  
  }
  
  bool BenchmarkROS::loadYaml()
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

bool BenchmarkROS::is_file_exist(const char *fileName)
{
  std::ifstream infile(fileName);
  return infile.good();
}

bool BenchmarkROS::saveYaml()
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

void BenchmarkROS::publishLidar(char* data, int size, uint64_t xtime)
{
  //1
 RSSLaser laser_msg;
 laser_msg.ParseFromArray(data,size);

 sensor_msgs::LaserScan scan_msg;

 scan_msg.header.stamp = ros::Time::now();
 scan_msg.header.frame_id = "base_link";

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

pub_lidar.publish(scan_msg);

delta_t_lidar = this->net2->getTime() - xtime;
size_lidar = size;

}

void BenchmarkROS::publishCameraColor(char* data, int size, uint64_t xtime)
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
  out_msg.header.frame_id = "base_link";
  pub_camera_color.publish(out_msg.toImageMsg());

  delta_t_camera_rgb = this->net2->getTime() - xtime;
  size_camera_rgb = size;
}

void BenchmarkROS::publishCameraDepth(char* data, int size, uint64_t xtime)
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
  out_msg.header.frame_id = "base_link";
  pub_camera_depth.publish(out_msg.toImageMsg());

  last_depth_frame_time = xtime;
  delta_t_camera_depth = this->net2->getTime() - xtime;
  size_camera_depth = size;
}

void BenchmarkROS::publishPointCloud()
{
  if ( last_color_frame_updated && last_depth_frame_updated)
  {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = MatToPoinXYZ();
      sensor_msgs::PointCloud2 point_cloud2;

      pcl::toROSMsg(*cloud, point_cloud2);

      point_cloud2.header.stamp = ros::Time::now();
      point_cloud2.header.frame_id = "base_link";

      pub_camera_point.publish(point_cloud2);
      last_color_frame_updated = false;
      last_depth_frame_updated = false;

      delta_t_point_cloud = this->net2->getTime() - last_depth_frame_time;
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr BenchmarkROS::MatToPoinXYZ()
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
    
    for (uint64_t u = 0; u < rows; ++u) 
    {
      for (uint64_t v = 0; v < cols; ++v) 
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

  size_point_cloud = cloud->points.size();
  return cloud;
}

void BenchmarkROS::publishIMU(char* data, int size, uint64_t xtime)
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
  out_msg.header.frame_id = "base_link"; 

  pub_imu.publish(out_msg);

  delta_t_imu = this->net2->getTime() - xtime;
  size_imu = size;
}

std::vector<char> BenchmarkROS::callbackDataLidar(std::vector<char> buffer, uint64_t priority, std::string sender)
{
  //1
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishLidar(&buffer[0],buffer.size(),priority);
  return result;
}

std::vector<char> BenchmarkROS::callbackDataCameraColor(std::vector<char> buffer, uint64_t priority, std::string sender)
{
  //ROS_INFO("GET CAMERA COLOR");
  //3
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishCameraColor(&buffer[0],buffer.size(),priority);
  return result;
}

std::vector<char> BenchmarkROS::callbackDataCameraDepth(std::vector<char> buffer, uint64_t priority, std::string sender)
{
  //4
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishCameraDepth(&buffer[0],buffer.size(),priority);
  return result;
}

std::vector<char> BenchmarkROS::callbackDataIMU(std::vector<char> buffer, uint64_t priority, std::string sender)
{
  //13
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishIMU(&buffer[0],buffer.size(),priority);
  return result;
}

void BenchmarkROS::update()
{
   publishPointCloud();

   benchmark_index++;

   if ( benchmark_index >= 100 )
   {
     benchmark_index = 0;

     ROS_INFO_STREAM("Time : " << this->net2->getTime());
     ROS_INFO_STREAM("Last depth time : " << last_depth_frame_time);

     ROS_INFO_STREAM("Latency imu : " << delta_t_imu);
     ROS_INFO_STREAM("Latency lidar : " << delta_t_lidar);
     ROS_INFO_STREAM("Latency camera rgb : " << delta_t_camera_rgb);
     ROS_INFO_STREAM("Latency camera depth : " << delta_t_camera_depth);
     ROS_INFO_STREAM("Latency point cloud : " << delta_t_point_cloud);

     ROS_INFO_STREAM("Size imu : " << size_imu);
     ROS_INFO_STREAM("Size lidar : " << size_lidar);
     ROS_INFO_STREAM("Size camera rgb : " << size_camera_rgb);
     ROS_INFO_STREAM("Size camera depth : " << size_camera_depth);
     ROS_INFO_STREAM("Point size point cloud : " << size_point_cloud);

     ROS_INFO_STREAM("======================");  
   }
}

void BenchmarkROS::kill()
{
  this->net2->Shutdown();
  ROS_INFO("RRS ROS TERMINATED");
}

BenchmarkROS::~BenchmarkROS()
{

}

} // namespace lmt
