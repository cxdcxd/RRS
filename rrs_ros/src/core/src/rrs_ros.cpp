#include "rrs_ros.hh"

namespace lmt
{

  Net2TestROS::Net2TestROS(ros::NodeHandle &nh,
   ros::NodeHandle &pnh,
   int argc,
   char *argv[])
  {
    ROS_INFO("NET2 CPP ROS STARTED");

    std::string path = ros::package::getPath("rrs_ros");
    path = path + "/cfg/config.yaml";

    config_path = path;

    ROS_INFO_STREAM("Config path is : " << config_path);

    loadYaml();
    saveYaml();

    ROS_INFO_STREAM("consul network address :" << m_settings.consul_network_address );
    ROS_INFO_STREAM("local network address :" << m_settings.local_network_address );
    ROS_INFO_STREAM("consul network mask :" << m_settings.consul_network_mask );
    ROS_INFO_STREAM("consul network port :" << m_settings.consul_network_port);
    ROS_INFO_STREAM("ntp server host name : " << m_settings.ntp_server_host_name);

    Net2Config config;

    config.consul_network_address = m_settings.consul_network_address;
    config.local_network_address = m_settings.local_network_address;
    config.consul_network_mask = m_settings.consul_network_mask;
    config.consul_network_port = m_settings.consul_network_port;
    config.ntp_server_host_name = m_settings.ntp_server_host_name;
    config.consul_mode = CLIENT;

    net2 = new Net2();

    this->net2->Init(config,"rrs_ros","test");

    this->test_step = 1;

    subscriber_camera_color = net2->subscriber();
    subscriber_camera_color->delegateNewData = std::bind(&Net2TestROS::callbackDataCameraColor, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result3 = subscriber_camera_color->Start("rrs-camera_color");

    subscriber_nmpc_right_marker = net2->subscriber();
    subscriber_nmpc_right_marker->delegateNewData = std::bind(&Net2TestROS::callbackDataNMPCRightMarker, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> resultNMPCR = subscriber_nmpc_right_marker->Start("rrs-nmpc_right_in");

    subscriber_nmpc_left_marker = net2->subscriber();
    subscriber_nmpc_left_marker->delegateNewData = std::bind(&Net2TestROS::callbackDataNMPCLeftMarker, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> resultNMPCL = subscriber_nmpc_left_marker->Start("rrs-nmpc_left_in");

    subscriber_camera_info = net2->subscriber();
    subscriber_camera_info->delegateNewData = std::bind(&Net2TestROS::callbackDataCameraInfo, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result5 = subscriber_camera_info->Start("rrs-camera_info");

    subscriber_joint_state = net2->subscriber();
    subscriber_joint_state->delegateNewData = std::bind(&Net2TestROS::callbackDataJointState, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result11 = subscriber_joint_state->Start("rrs-joint_state");

    publisher_joint_command_right = net2->publisher("joint_right");
    publisher_joint_command_right->Start();

    publisher_joint_command_left = net2->publisher("joint_left");
    publisher_joint_command_left->Start();

    //TO ROS
    pub_camera_color = nh.advertise<sensor_msgs::Image>("movo/camera/image_raw", 1);
    pub_camera_info = nh.advertise<sensor_msgs::CameraInfo>("movo/camera/camera_info", 1);
    pub_joint_state = nh.advertise<sensor_msgs::JointState>("joint_states",1);
    pub_joint_state_right = nh.advertise<sensor_msgs::JointState>("/movo/right_arm_controller/state",1);
    pub_joint_state_left = nh.advertise<sensor_msgs::JointState>("/movo/left_arm_controller/state",1);

    //TO NMPC
    pub_left_end_effector = nh.advertise<geometry_msgs::Pose>("left/nmpc_controller/in/goal", 1);
    pub_left_end_effector_stamp = nh.advertise<geometry_msgs::PoseStamped>("left/nmpc_controller/in/goal/stamp", 1);
    pub_right_end_effector = nh.advertise<geometry_msgs::Pose>("right/nmpc_controller/in/goal", 1);
    pub_right_end_effector_stamp = nh.advertise<geometry_msgs::PoseStamped>("right/nmpc_controller/in/goal/stamp", 1);

    //FROM NMPC
    sub_jaco_right_vel = nh.subscribe("/movo/right_arm/angular_vel_cmd",1, &Net2TestROS::chatterCallbackVelRight, this);
    sub_jaco_left_vel = nh.subscribe("/movo/left_arm/angular_vel_cmd",1, &Net2TestROS::chatterCallbackVelLeft, this);
  }
  
  std::vector<char> Net2TestROS::callbackDataCameraInfo(std::vector<char> buffer, unsigned int priority, std::string sender)
  {

    //5
    std::vector<char> result;
    if ( priority == 10 ) return result;
    publishCameraInfo(&buffer[0],buffer.size());
    return result;
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

  void Net2TestROS::chatterCallbackVelRight(const movo_msgs::JacoAngularVelocityCmd7DOF::ConstPtr& msg)
  {
    //ROS_INFO("Got velocity for right hand");

    RRSJointCommand cmd;
   
    cmd.add_goal(msg->theta_shoulder_pan_joint);
    cmd.add_goal(msg->theta_shoulder_lift_joint);
    cmd.add_goal(msg->theta_arm_half_joint);
    cmd.add_goal(msg->theta_elbow_joint);
    cmd.add_goal(msg->theta_wrist_spherical_1_joint);
    cmd.add_goal(msg->theta_wrist_spherical_2_joint);
    cmd.add_goal(msg->theta_wrist_3_joint);
  
    int bsize = cmd.ByteSize();
    char buffer[bsize];
    cmd.SerializeToArray(buffer,bsize);

    //ROS_WARN("Send Done");
    publisher_joint_command_right->send(buffer,bsize,1);
  }

  void Net2TestROS::chatterCallbackVelLeft(const movo_msgs::JacoAngularVelocityCmd7DOF::ConstPtr& msg)
  {
    //ROS_INFO("Got velocity for left hand");

    RRSJointCommand cmd;
   
    cmd.add_goal(msg->theta_shoulder_pan_joint);
    cmd.add_goal(msg->theta_shoulder_lift_joint);
    cmd.add_goal(msg->theta_arm_half_joint);
    cmd.add_goal(msg->theta_elbow_joint);
    cmd.add_goal(msg->theta_wrist_spherical_1_joint);
    cmd.add_goal(msg->theta_wrist_spherical_2_joint);
    cmd.add_goal(msg->theta_wrist_3_joint);
  
    int bsize = cmd.ByteSize();
    char buffer[bsize];
    cmd.SerializeToArray(buffer,bsize);

    //ROS_WARN("Send Done");
    publisher_joint_command_left->send(buffer,bsize,1);
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

void Net2TestROS::publishJointState(char* data, int size)
{
  //11
  RRSJointState state_msg;
  state_msg.ParseFromArray(data,size);

  sensor_msgs::JointState ros_state_msg;
  sensor_msgs::JointState ros_state_right_msg;
  sensor_msgs::JointState ros_state_left_msg;
  
  for ( int i = 0 ; i < 19 ; i++)
  {
    ros_state_msg.name.push_back(state_msg.name(i));
    ros_state_msg.position.push_back(state_msg.position(i));
    ros_state_msg.velocity.push_back(state_msg.velocity(i));
    ros_state_msg.effort.push_back(state_msg.effort(i));
  }

  for ( int i = 0 ; i < 7 ; i++)
  {
    ros_state_right_msg.name.push_back(state_msg.name(i));
    ros_state_right_msg.position.push_back(state_msg.position(i));
    ros_state_right_msg.velocity.push_back(state_msg.velocity(i));
    ros_state_right_msg.effort.push_back(state_msg.effort(i));
  }

  for ( int i = 8 ; i < 15 ; i++)
  {
    ros_state_left_msg.name.push_back(state_msg.name(i));
    ros_state_left_msg.position.push_back(state_msg.position(i));
    ros_state_left_msg.velocity.push_back(state_msg.velocity(i));
    ros_state_left_msg.effort.push_back(state_msg.effort(i));
  }

  pub_joint_state.publish(ros_state_msg);
  pub_joint_state_right.publish(ros_state_right_msg);
  pub_joint_state_left.publish(ros_state_left_msg);
}

std::vector<char> Net2TestROS::callbackDataNMPCRightMarker(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //ROS_INFO("Get right marker");

  std::vector<char> result;
  if ( priority == 10 ) return result;

  RVector7 rvector7_msg;
  rvector7_msg.ParseFromArray(&buffer[0],buffer.size());

  geometry_msgs::Pose msg;

  msg.position.x = rvector7_msg.x();
  msg.position.y = rvector7_msg.y();
  msg.position.z = rvector7_msg.z();

  msg.orientation.x = rvector7_msg.qx();
  msg.orientation.y = rvector7_msg.qy();
  msg.orientation.z = rvector7_msg.qz();
  msg.orientation.w = rvector7_msg.qw();

  pub_right_end_effector.publish(msg);

  geometry_msgs::PoseStamped msgs;

  msgs.pose.position.x = msg.position.x;
  msgs.pose.position.y = msg.position.y;
  msgs.pose.position.z = msg.position.z;

  msgs.pose.orientation.x = msg.orientation.x;
  msgs.pose.orientation.y = msg.orientation.y;
  msgs.pose.orientation.z = msg.orientation.z;
  msgs.pose.orientation.w = msg.orientation.w;

  msgs.header.frame_id = "base_link";

  pub_right_end_effector_stamp.publish(msgs);
}

std::vector<char> Net2TestROS::callbackDataNMPCLeftMarker(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //ROS_INFO("Get left marker");

  std::vector<char> result;
  if ( priority == 10 ) return result;

  RVector7 rvector7_msg;
  rvector7_msg.ParseFromArray(&buffer[0],buffer.size());

  geometry_msgs::Pose msg;

  msg.position.x = rvector7_msg.x();
  msg.position.y = rvector7_msg.y();
  msg.position.z = rvector7_msg.z();

  msg.orientation.x = rvector7_msg.qx();
  msg.orientation.y = rvector7_msg.qy();
  msg.orientation.z = rvector7_msg.qz();
  msg.orientation.w = rvector7_msg.qw();

  pub_left_end_effector.publish(msg);

  geometry_msgs::PoseStamped msgs;

  msgs.pose.position.x = msg.position.x;
  msgs.pose.position.y = msg.position.y;
  msgs.pose.position.z = msg.position.z;

  msgs.pose.orientation.x = msg.orientation.x;
  msgs.pose.orientation.y = msg.orientation.y;
  msgs.pose.orientation.z = msg.orientation.z;
  msgs.pose.orientation.w = msg.orientation.w;

  msgs.header.frame_id = "base_link";

  pub_left_end_effector_stamp.publish(msgs);
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

std::vector<char> Net2TestROS::callbackDataJointState(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  //11
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishJointState(&buffer[0],buffer.size());
  return result;
}

void Net2TestROS::update()
{
   
}

void Net2TestROS::kill()
{
  this->net2->Shutdown();
  ROS_INFO("RRS ROS TERMINATED");
}

Net2TestROS::~Net2TestROS()
{

}

} // namespace lmt
