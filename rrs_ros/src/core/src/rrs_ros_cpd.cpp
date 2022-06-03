#include "rrs_ros_cpd.hh"

namespace lmt
{

  Net2TestROSCPD::Net2TestROSCPD(ros::NodeHandle &nh,
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

    //subscriber_camera_color = net2->subscriber();
    //subscriber_camera_color->delegateNewData = std::bind(&Net2TestROSCPD::callbackDataCameraColor, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    //ProcessResult<int> result3 = subscriber_camera_color->Start("rrs-camera_color");
 
    //subscriber_cpd = net2->subscriber();
    //subscriber_cpd->delegateNewData = std::bind(&Net2TestROSCPD::callbackDataCPDx, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    //ProcessResult<int> result3 = subscriber_cpd->Start("rrs-cpd_command");

    subscriber_nmpc_right_marker = net2->subscriber();
    subscriber_nmpc_right_marker->delegateNewData = std::bind(&Net2TestROSCPD::callbackDataNMPCRightMarker, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> resultNMPCR = subscriber_nmpc_right_marker->Start("rrs-nmpc_right_in");

    subscriber_nmpc_left_marker = net2->subscriber();
    subscriber_nmpc_left_marker->delegateNewData = std::bind(&Net2TestROSCPD::callbackDataNMPCLeftMarker, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> resultNMPCL = subscriber_nmpc_left_marker->Start("rrs-nmpc_left_in");

    //subscriber_camera_info = net2->subscriber();
    //subscriber_camera_info->delegateNewData = std::bind(&Net2TestROSCPD::callbackDataCameraInfo, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    //ProcessResult<int> result5 = subscriber_camera_info->Start("rrs-camera_info");

    subscriber_joint_state = net2->subscriber();
    subscriber_joint_state->delegateNewData = std::bind(&Net2TestROSCPD::callbackDataJointState, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    ProcessResult<int> result11 = subscriber_joint_state->Start("rrs-joint_state");

    publisher_joint_command_right = net2->publisher("joint_right");
    publisher_joint_command_right->Start();

    publisher_joint_command_left = net2->publisher("joint_left");
    publisher_joint_command_left->Start();

    //publisher_cpd = net2->publisher("cpd_result");
    //publisher_cpd->Start();

    pub_camera_color = nh.advertise<sensor_msgs::Image>("movo/camera/image_raw", 1);
    pub_camera_info = nh.advertise<sensor_msgs::CameraInfo>("movo/camera/camera_info", 1);
    pub_joint_state = nh.advertise<sensor_msgs::JointState>("joint_states",1);
    pub_joint_state_right = nh.advertise<sensor_msgs::JointState>("/movo/right_arm_controller/state",1);
    pub_joint_state_left = nh.advertise<sensor_msgs::JointState>("/movo/left_arm_controller/state",1);

    pub_left_end_effector = nh.advertise<geometry_msgs::Pose>("left/nmpc_controller/in/goal", 1);
    pub_left_end_effector_stamp = nh.advertise<geometry_msgs::PoseStamped>("left/nmpc_controller/in/goal/stamp", 1);
    pub_right_end_effector = nh.advertise<geometry_msgs::Pose>("right/nmpc_controller/in/goal", 1);
    pub_right_end_effector_stamp = nh.advertise<geometry_msgs::PoseStamped>("right/nmpc_controller/in/goal/stamp", 1);

    sub_jaco_right_vel = nh.subscribe("/movo/right_arm/angular_vel_cmd",1, &Net2TestROSCPD::chatterCallbackVelRight, this);
    sub_jaco_left_vel = nh.subscribe("/movo/left_arm/angular_vel_cmd",1, &Net2TestROSCPD::chatterCallbackVelLeft, this);

    //Init Net1
    net_interface = new Network(9871,9870,0,"tele","127.0.0.1",false,5);
    net_interface->dataCallBackFunction = std::bind(&Net2TestROSCPD::receiveCallback, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3);

    net_interface_cpd = new Network(9873,9872,0,"cpd","127.0.0.1",false,5);
    net_interface_cpd->dataCallBackFunction = std::bind(&Net2TestROSCPD::receiveCallbackCPD, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3);
  }
  
void Net2TestROSCPD::receiveCallback(char * data,int size,int index)
{
  //ROS_WARN("TELE GET");

  net_interface->tcpWrite(data,size);
}

void Net2TestROSCPD::receiveCallbackCPD(char * data,int size,int index)
{
  //ROS_WARN("CPD GET");

  if ( cpd_busy == true ) return;

  cpd_busy = true;

  RRSCPDCommand cpd_command;
  cpd_command.ParseFromArray(data,size);

  ROS_INFO("Get CPD Command");

  size_t nrowsa = cpd_command.points_a().size();
  size_t nrowsb = cpd_command.points_b().size();
  size_t nrowsskill = cpd_command.points_skill().size();

  //cpd::Matrix matrixa = cpd::matrix_from_path(path + "face.csv");
  //cpd::Matrix matrixb = cpd::matrix_from_path(path + "face_distorted.csv");

  cpd::Matrix matrixa(nrowsa, 3);
  cpd::Matrix matrixb(nrowsb, 3);
  cpd::Matrix matrixc(nrowsskill, 3);

  for (size_t j = 0; j < nrowsa; ++j) 
  {
       matrixa(j, 0) = cpd_command.points_a(j).x();
       matrixa(j, 1) = cpd_command.points_a(j).y();
       matrixa(j, 2) = cpd_command.points_a(j).z();
  }

  for (size_t j = 0; j < nrowsb; ++j) 
  {
       matrixb(j, 0) = cpd_command.points_b(j).x();
       matrixb(j, 1) = cpd_command.points_b(j).y();
       matrixb(j, 2) = cpd_command.points_b(j).z();
  }

  for (size_t j = 0; j < nrowsskill; ++j) 
  {
       matrixc(j, 0) = cpd_command.points_skill(j).x();
       matrixc(j, 1) = cpd_command.points_skill(j).y();
       matrixc(j, 2) = cpd_command.points_skill(j).z();
  }

  if ( nrowsa != nrowsb )
  {
    RRSCPDResult cpd_result;
    cpd_result.set_result_iterations(0);

    int bsizex = cpd_result.ByteSize();
    char bufferx[bsizex];
    cpd_result.SerializeToArray(bufferx,bsizex);
    net_interface_cpd->tcpWrite(bufferx,bsizex);

    ROS_ERROR_STREAM("Invalid points for CPD, Ignore");
    cpd_busy = false;
    return;
  }

  if ( nrowsa < 10 )
  {
    RRSCPDResult cpd_result;
    cpd_result.set_result_iterations(0);

    int bsizex = cpd_result.ByteSize();
    char bufferx[bsizex];
    cpd_result.SerializeToArray(bufferx,bsizex);
    net_interface_cpd->tcpWrite(bufferx,bsizex);

    ROS_ERROR_STREAM("Invalid points for CPD, Ignore");
    cpd_busy = false;
    return;
  }

  ROS_INFO_STREAM("Get CPD Command " << matrixa.rows()  << " " << matrixb.rows() << " " << matrixc.rows());

  //rigid.max_iterations(10);
  //rigid.correspondence(true);

  //cpd::AffineResult result = this->affine.run(matrixa, matrixb);
  

  if ( cpd_command.mode() == 0)
  {
      ROS_WARN("Rigid mode");

      cpd::RigidResult result = this->rigid.run(matrixa, matrixb);
  
      //Affine CPD
      RRSCPDResult cpd_result;
      

      ROS_INFO_STREAM("CPD Result Ready " << result.points.rows());

      //We have the trasnform now lets apply it to the main skill
      //result.points = matrixc * result.transform.transpose() + result.translation.transpose().replicate(matrixc.rows(), 1);
      result.points = result.scale * matrixc * result.rotation.transpose() + result.translation.transpose().replicate(matrixc.rows(), 1);

      for ( int i = 0 ; i < result.points.rows() ; i++ )
      {
        cpd_result.add_result_points();

        cpd_result.mutable_result_points(i)->set_x(result.points(i,0));
        cpd_result.mutable_result_points(i)->set_y(result.points(i,1));
        cpd_result.mutable_result_points(i)->set_z(result.points(i,2));
      }

      cpd_result.set_result_1(result.scale);
      cpd_result.set_result_2(result.sigma2);
      cpd_result.set_result_iterations(result.iterations);

      //ROS_INFO_STREAM("Size " << result.rotation.rows() << " " <<  result.rotation.cols() );
      //ROS_INFO_STREAM("T " << result.translation(0) << " " << result.translation(1) << " "  <<  result.translation(2) );
      //ROS_INFO_STREAM("R1 " << result.rotation(0,0) << " " << result.rotation(0,1) << " "  <<  result.rotation(0,2) );
      //ROS_INFO_STREAM("R2 " << result.rotation(1,0) << " " << result.rotation(1,1) << " "  <<  result.rotation(1,2) );
      //ROS_INFO_STREAM("R3 " << result.rotation(2,0) << " " << result.rotation(2,1) << " "  <<  result.rotation(2,2) );
      //ROS_INFO_STREAM("Scale " << result.scale );

      int bsizex = cpd_result.ByteSize();
      char bufferx[bsizex];
      cpd_result.SerializeToArray(bufferx,bsizex);

      net_interface_cpd->tcpWrite(bufferx,bsizex);

      ROS_INFO("Rigid Send CPD Result");


      }
      else if ( cpd_command.mode() == 1)
      {

      ROS_WARN("Non-rigid mode"); 

      cpd::NonrigidResult result = this->nonrigid.run(matrixa, matrixc);
  
      //Affine CPD
      RRSCPDResult cpd_result;
    
      ROS_INFO_STREAM("CPD Result Ready " << result.points.rows());

      //We have the trasnform now lets apply it to the main skill
      //result.points = matrixc * result.transform.transpose() + result.translation.transpose().replicate(matrixc.rows(), 1);
      //result.points = result.scale * matrixc * result.rotation.transpose() + result.translation.transpose().replicate(matrixc.rows(), 1);

      for ( int i = 0 ; i < result.points.rows() ; i++ )
      {
        cpd_result.add_result_points();

        cpd_result.mutable_result_points(i)->set_x(result.points(i,0));
        cpd_result.mutable_result_points(i)->set_y(result.points(i,1));
        cpd_result.mutable_result_points(i)->set_z(result.points(i,2));
      }

      cpd_result.set_result_1(1);
      cpd_result.set_result_2(result.sigma2);
      cpd_result.set_result_iterations(result.iterations);

      //ROS_INFO_STREAM("Size " << result.rotation.rows() << " " <<  result.rotation.cols() );
      //ROS_INFO_STREAM("T " << result.translation(0) << " " << result.translation(1) << " "  <<  result.translation(2) );
      //ROS_INFO_STREAM("R1 " << result.rotation(0,0) << " " << result.rotation(0,1) << " "  <<  result.rotation(0,2) );
      //ROS_INFO_STREAM("R2 " << result.rotation(1,0) << " " << result.rotation(1,1) << " "  <<  result.rotation(1,2) );
      //ROS_INFO_STREAM("R3 " << result.rotation(2,0) << " " << result.rotation(2,1) << " "  <<  result.rotation(2,2) );
      //ROS_INFO_STREAM("Scale " << result.scale );

      int bsizex = cpd_result.ByteSize();
      char bufferx[bsizex];
      cpd_result.SerializeToArray(bufferx,bsizex);

      net_interface_cpd->tcpWrite(bufferx,bsizex);

      ROS_INFO("Rigid Send CPD Result");
      }

 

  cpd_busy = false;

  return;

}

std::vector<char> Net2TestROSCPD::callbackDataCameraInfo(std::vector<char> buffer, uint64_t priority, std::string sender)
{

  //5
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishCameraInfo(&buffer[0],buffer.size());
  return result;
}

  void Net2TestROSCPD::publishCameraInfo(char* data, int size)
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

  void Net2TestROSCPD::chatterCallbackVelRight(const movo_msgs::JacoAngularVelocityCmd7DOF::ConstPtr& msg)
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

  void Net2TestROSCPD::chatterCallbackVelLeft(const movo_msgs::JacoAngularVelocityCmd7DOF::ConstPtr& msg)
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
  
  
  bool Net2TestROSCPD::loadYaml()
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

bool Net2TestROSCPD::is_file_exist(const char *fileName)
{
  std::ifstream infile(fileName);
  return infile.good();
}

bool Net2TestROSCPD::saveYaml()
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

void Net2TestROSCPD::publishCameraColor(char* data, int size)
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

void Net2TestROSCPD::publishJointState(char* data, int size)
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

std::vector<char> Net2TestROSCPD::callbackDataNMPCRightMarker(std::vector<char> buffer, uint64_t priority, std::string sender)
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

std::vector<char> Net2TestROSCPD::callbackDataNMPCLeftMarker(std::vector<char> buffer, uint64_t priority, std::string sender)
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

std::vector<char> Net2TestROSCPD::callbackDataCPDx(std::vector<char> buffer, uint64_t priority, std::string sender)
{

  std::vector<char> results;
  if ( priority == 10 || cpd_busy == true ) return results;

  cpd_busy = true;

  RRSCPDCommand cpd_command;
  cpd_command.ParseFromArray(&buffer[0],buffer.size());

  ROS_INFO("Get CPD Command");

  size_t nrowsa = cpd_command.points_a().size();
  size_t nrowsb = cpd_command.points_b().size();
  size_t nrowsskill = cpd_command.points_skill().size();

  //cpd::Matrix matrixa = cpd::matrix_from_path(path + "face.csv");
  //cpd::Matrix matrixb = cpd::matrix_from_path(path + "face_distorted.csv");

  cpd::Matrix matrixa(nrowsa, 3);
  cpd::Matrix matrixb(nrowsb, 3);
  cpd::Matrix matrixc(nrowsskill, 3);

  for (size_t j = 0; j < nrowsa; ++j) 
  {
       matrixa(j, 0) = cpd_command.points_a(j).x();
       matrixa(j, 1) = cpd_command.points_a(j).y();
       matrixa(j, 2) = cpd_command.points_a(j).z();
  }

  for (size_t j = 0; j < nrowsb; ++j) 
  {
       matrixb(j, 0) = cpd_command.points_b(j).x();
       matrixb(j, 1) = cpd_command.points_b(j).y();
       matrixb(j, 2) = cpd_command.points_b(j).z();
  }

  for (size_t j = 0; j < nrowsskill; ++j) 
  {
       matrixc(j, 0) = cpd_command.points_skill(j).x();
       matrixc(j, 1) = cpd_command.points_skill(j).y();
       matrixc(j, 2) = cpd_command.points_skill(j).z();
  }

  if ( nrowsa != nrowsb )
  {
    ROS_ERROR_STREAM("Invalid points for CPD, Ignore");
    cpd_busy = false;
    return results;
  }

  if ( nrowsa < 10 )
  {
    ROS_ERROR_STREAM("Invalid points for CPD, Ignore");
    cpd_busy = false;
    return results;
  }

  ROS_INFO_STREAM("Get CPD Command " << matrixa.rows()  << " " << matrixb.rows() << " " << matrixc.rows());

  //rigid.max_iterations(10);
  //rigid.correspondence(true);

  //cpd::AffineResult result = this->affine.run(matrixa, matrixb);
  

  if ( cpd_command.mode() == 0)
  {
      ROS_WARN("Rigid mode");

      cpd::RigidResult result = this->rigid.run(matrixa, matrixb);
  
      //Affine CPD
      RRSCPDResult cpd_result;
      

      ROS_INFO_STREAM("CPD Result Ready " << result.points.rows());

      //We have the trasnform now lets apply it to the main skill
      //result.points = matrixc * result.transform.transpose() + result.translation.transpose().replicate(matrixc.rows(), 1);
      result.points = result.scale * matrixc * result.rotation.transpose() + result.translation.transpose().replicate(matrixc.rows(), 1);

      for ( int i = 0 ; i < result.points.rows() ; i++ )
      {
        cpd_result.add_result_points();

        cpd_result.mutable_result_points(i)->set_x(result.points(i,0));
        cpd_result.mutable_result_points(i)->set_y(result.points(i,1));
        cpd_result.mutable_result_points(i)->set_z(result.points(i,2));
      }

      cpd_result.set_result_1(result.scale);
      cpd_result.set_result_2(result.sigma2);
      cpd_result.set_result_iterations(result.iterations);

      //ROS_INFO_STREAM("Size " << result.rotation.rows() << " " <<  result.rotation.cols() );
      //ROS_INFO_STREAM("T " << result.translation(0) << " " << result.translation(1) << " "  <<  result.translation(2) );
      //ROS_INFO_STREAM("R1 " << result.rotation(0,0) << " " << result.rotation(0,1) << " "  <<  result.rotation(0,2) );
      //ROS_INFO_STREAM("R2 " << result.rotation(1,0) << " " << result.rotation(1,1) << " "  <<  result.rotation(1,2) );
      //ROS_INFO_STREAM("R3 " << result.rotation(2,0) << " " << result.rotation(2,1) << " "  <<  result.rotation(2,2) );
      //ROS_INFO_STREAM("Scale " << result.scale );

      int bsizex = cpd_result.ByteSize();
      char bufferx[bsizex];
      cpd_result.SerializeToArray(bufferx,bsizex);

      publisher_cpd->send(bufferx,bsizex,0);

      ROS_INFO("Rigid Send CPD Result");


      }
      else if ( cpd_command.mode() == 1)
      {

      ROS_WARN("Non-rigid mode"); 

      cpd::NonrigidResult result = this->nonrigid.run(matrixa, matrixc);
  
      //Affine CPD
      RRSCPDResult cpd_result;
      cpd_result.set_result_iterations(1);

      ROS_INFO_STREAM("CPD Result Ready " << result.points.rows());

      //We have the trasnform now lets apply it to the main skill
      //result.points = matrixc * result.transform.transpose() + result.translation.transpose().replicate(matrixc.rows(), 1);
      //result.points = result.scale * matrixc * result.rotation.transpose() + result.translation.transpose().replicate(matrixc.rows(), 1);

      for ( int i = 0 ; i < result.points.rows() ; i++ )
      {
        cpd_result.add_result_points();

        cpd_result.mutable_result_points(i)->set_x(result.points(i,0));
        cpd_result.mutable_result_points(i)->set_y(result.points(i,1));
        cpd_result.mutable_result_points(i)->set_z(result.points(i,2));
      }

      cpd_result.set_result_1(1);
      cpd_result.set_result_2(result.sigma2);
      cpd_result.set_result_iterations(result.iterations);

      //ROS_INFO_STREAM("Size " << result.rotation.rows() << " " <<  result.rotation.cols() );
      //ROS_INFO_STREAM("T " << result.translation(0) << " " << result.translation(1) << " "  <<  result.translation(2) );
      //ROS_INFO_STREAM("R1 " << result.rotation(0,0) << " " << result.rotation(0,1) << " "  <<  result.rotation(0,2) );
      //ROS_INFO_STREAM("R2 " << result.rotation(1,0) << " " << result.rotation(1,1) << " "  <<  result.rotation(1,2) );
      //ROS_INFO_STREAM("R3 " << result.rotation(2,0) << " " << result.rotation(2,1) << " "  <<  result.rotation(2,2) );
      //ROS_INFO_STREAM("Scale " << result.scale );

      int bsizex = cpd_result.ByteSize();
      char bufferx[bsizex];
      cpd_result.SerializeToArray(bufferx,bsizex);

      publisher_cpd->send(bufferx,bsizex,0);

      ROS_INFO("Rigid Send CPD Result");
      }

 

  cpd_busy = false;

  return results;
}

std::vector<char> Net2TestROSCPD::callbackDataCameraColor(std::vector<char> buffer, uint64_t priority, std::string sender)
{
  //ROS_INFO("GET CAMERA COLOR");
  //3
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishCameraColor(&buffer[0],buffer.size());
  return result;
}

std::vector<char> Net2TestROSCPD::callbackDataJointState(std::vector<char> buffer, uint64_t priority, std::string sender)
{
  //11
  std::vector<char> result;
  if ( priority == 10 ) return result;
  publishJointState(&buffer[0],buffer.size());
  return result;
}

void Net2TestROSCPD::update()
{
   
}

void Net2TestROSCPD::kill()
{
  this->net2->Shutdown();
  ROS_INFO("RRS ROS TERMINATED");
}

Net2TestROSCPD::~Net2TestROSCPD()
{

}

} // namespace lmt
