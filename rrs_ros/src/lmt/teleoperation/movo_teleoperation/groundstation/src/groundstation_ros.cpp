#include "robot_groundstation/groundstation_ros.hh"

namespace roboland
{

int counter = 0;
int plan_counter = 0;

GroundstationRos::GroundstationRos(ros::NodeHandle &nh,
                                   ros::NodeHandle &pnh,
                                   int argc,
                                   char *argv[],
                                   std::string id) :
    set_stop(false),
    thread_main(&GroundstationRos::thrMain,this),
    last_cmd_vel_time(ros::Time::now())
{
    std::string path = ros::package::getPath("robot_groundstation");
    path = path + "/config/config.yaml";

    config_path = path;

    ROS_INFO_STREAM("Config path is : " << config_path);

    //Creating the config file for first use or load it 
    loadYaml(); 
    string s;
    
    pnh.getParam("station_mode", s);
    
    ROS_INFO_STREAM("Param : " << s);

    if ( s != "" )
	    m_settings.station_mode = s;

    saveYaml();

    //Config Print
    ROS_INFO_STREAM("station 5G1 iP is :" << m_settings.station_5G1_ip );
    ROS_INFO_STREAM("station 5G2 ip is :" << m_settings.station_5G2_ip );
    ROS_INFO_STREAM("station WIFI iP is :" << m_settings.station_WIFI_ip );
    ROS_INFO_STREAM("station mode is :" << m_settings.station_mode);
    ROS_INFO_STREAM("station camera stream : " << m_settings.station_camera_stream);
    ROS_INFO_STREAM("station lidar stream: " << m_settings.station_lidar_stream);
    ROS_INFO_STREAM("station status stream: " << m_settings.station_status_stream);
    ROS_INFO_STREAM("station camera frame skip ratio: " << m_settings.station_camera_frame_skip_ratio);
    ROS_INFO_STREAM("station camera resize ratio: " << m_settings.station_camera_resize_ratio);
    ROS_INFO_STREAM("station camera jpeg compression ratio: " << m_settings.station_camera_jpeg_compression_ratio);
    ROS_INFO_STREAM("station print debug: " << m_settings.print_debug);

    //creating ground station network interface

    //10.66.171.182  internal
    //10.162.148.135 lkn
    //192.168.50.126 asus

	std::string ip = "";

	if ( m_settings.station_mode == "5G1")
	{
		 ip = m_settings.station_5G1_ip;
		 ROS_WARN_STREAM("Ip is " << ip << " [5G1 Mode]");
	}
	else if ( m_settings.station_mode == "5G2")
        {
		ip = m_settings.station_5G2_ip;
		ROS_WARN_STREAM("Ip is " << ip << " [5G2 Mode]");
	}
	else if ( m_settings.station_mode == "WIFI")
	{
		 ip = m_settings.station_WIFI_ip;
		 ROS_WARN_STREAM("Ip is " << ip << " [WIFI Mode]");
	}
	else 
	{
		ROS_ERROR("Config error, please set station mode (5G or WIFI)");
		return;
	}

	this->send_camera_to_station = m_settings.station_camera_stream;
	this->send_lidar_to_station = m_settings.station_lidar_stream;
	this->send_arm_to_station = m_settings.station_status_stream;
	this->camera_resize_ratio = m_settings.station_camera_resize_ratio;
	this->jpeg_compression_ratio = m_settings.station_camera_jpeg_compression_ratio;
	this->frame_skip_ratio = m_settings.station_camera_frame_skip_ratio;
	this->print_debug = m_settings.print_debug;
	this->frame_skip = 0;

  ground_station = new Groundstation(ip,argc,argv);
  ground_station->callBackCMD = std::bind(&GroundstationRos::callbackGroundstation, this, std::placeholders::_1);
  ground_station->callBackHapticPositionRight = std::bind(&GroundstationRos::callbackHapticRight, this, std::placeholders::_1);
  ground_station->callBackHapticPositionLeft = std::bind(&GroundstationRos::callbackHapticLeft, this, std::placeholders::_1);
  ground_station->callBackML = std::bind(&GroundstationRos::callbackML, this, std::placeholders::_1);

  mutex = false;
  app_exit = false;
  counter = 0;
  send_counter = 0;
  get_new = false;

  //20 hz
  pub_base_cmd = nh.advertise<geometry_msgs::Twist>("movo/teleop/cmd_vel", 1);
  pub_head_cmd = nh.advertise<movo_msgs::PanTiltCmd>("movo/head/cmd", 1);
  pub_linear_cmd = nh.advertise<movo_msgs::LinearActuatorCmd>("/movo/linear_actuator_cmd",1);
  pub_head_roll_cmd = nh.advertise<sensor_msgs::JointState>("/dxl/joints_goal",1);

  //500 hz
  sub_right_force_render = nh.subscribe("/ft_sensor_right/ft_compensated_right", 1, &GroundstationRos::callbackHapticRenderRightArm,this);
  pub_ft_right = nh.advertise<geometry_msgs::WrenchStamped>("movo/ft_right", 1);
  sub_left_force_render = nh.subscribe("/ft_sensor_left/ft_compensated_left", 1, &GroundstationRos::callbackHapticRenderLeftArm,this);
  pub_ft_left = nh.advertise<geometry_msgs::WrenchStamped>("movo/ft_left", 1);

  pub_right_arm_haptic_control = nh.advertise<joint_control::HapticCommand>("/movo/right_arm/control/haptic", 1);
  pub_left_arm_haptic_control = nh.advertise<joint_control::HapticCommand>("/movo/left_arm/control/haptic",1);

  pub_camera_color = nh.advertise<sensor_msgs::Image>("/kinect2/qhd/image_color/converted",1);

  //30 hz
  sub_camera = nh.subscribe("/rgb/image_raw", 1, &GroundstationRos::callbackCamera, this);
  sub_battery = nh.subscribe("/movo/feedback/battery", 1, &GroundstationRos::callbackBattery, this);
  sub_laser = nh.subscribe("/movo/scan_multi", 1, &GroundstationRos::callbackLaser,this);

  //1 hz
  cl_arm_home = nh.serviceClient<std_srvs::Empty>("move/joint_control/home_arms");

  cl_arm_mode_none = nh.serviceClient<std_srvs::Empty>("move/joint_control/mode/none");
  cl_arm_mode_autonomous = nh.serviceClient<std_srvs::Empty>("move/joint_control/mode/autonomous");
  cl_arm_mode_direct = nh.serviceClient<std_srvs::Empty>("move/joint_control/mode/direct");
  cl_arm_mode_shared = nh.serviceClient<std_srvs::Empty>("move/joint_control/mode/shared");

  pub_arm_status = nh.advertise<std_msgs::Bool>("/movo/arm/status",1);

  pub_voice_cmd = nh.advertise<std_msgs::String>("/movo/voice/text",1);
  pub_nav_cmd = nh.advertise<std_msgs::String>("goal_name",1);

  //sub_right_gripper_feedback = nh.subscribe("/movo/right_gripper/joint_states", 1, &GroundstationRos::callbackRightGripper,this);
  //sub_left_gripper_feedback = nh.subscribe("/movo/left_gripper/joint_states", 1, &GroundstationRos::callbackLeftGripper,this);

  sub_right_arm_feedback = nh.subscribe("/movo/right_arm/joint_states", 1, &GroundstationRos::callbackRightArm,this);
  sub_left_arm_feedback = nh.subscribe("/movo/left_arm/joint_states", 1, &GroundstationRos::callbackLeftArm,this);
  sub_head_feedback = nh.subscribe("/movo/head/joint_states", 1, &GroundstationRos::callbackHead,this);
  sub_z_axes_feedback = nh.subscribe("/movo/linear_actuator/joint_states", 1, &GroundstationRos::callbackZAxes,this);

  sub_clusters = nh.subscribe("/ml/clusters", 1, &GroundstationRos::callbackClusters,this);
  pub_ml = nh.advertise<joint_control::HapticCommand>("ml/settings", 1);
  
  sub_block_position = nh.subscribe("/ground_truth_node/groundtruth_odom",1,&GroundstationRos::callbackBlock,this);
  sub_hand_pose = nh.subscribe("/j2s7s300_driver/out/cartesian_command",1,&GroundstationRos::callbackToolPos,this);

  pub_diagram_force_network_rate =  nh.advertise<std_msgs::Float32>("/diagram/force_network", 1);
  pub_diagram_force_network_deadband_value =  nh.advertise<std_msgs::Float32>("/diagram/force_deadband_value", 1);
  pub_diagram_force_network_send = nh.advertise<std_msgs::Float32>("/diagram/force_send", 1);
  pub_diagram_force_network_force = nh.advertise<geometry_msgs::WrenchStamped>("/diagram/force_raw", 1);

  pub_diagram_velocity_network_rate =  nh.advertise<std_msgs::Float32>("/diagram/velocity_network", 1);
  pub_diagram_velocity_network_deadband_value =  nh.advertise<std_msgs::Float32>("/diagram/velocity_deadband_value", 1);
  pub_diagram_velocity_network_send = nh.advertise<std_msgs::Float32>("/diagram/velocity_send", 1);
  pub_diagram_velocity_network_command = nh.advertise<joint_control::HapticCommand>("/diagram/velocity_raw", 1);
  
  //--- MPC Joint Control Mode

  sub_end_effector_right = nh.subscribe("right/nmpc_controller/out/eef_pose", 1, &GroundstationRos::callbackeefPoseRight,this);
  sub_end_effector_left = nh.subscribe("left/nmpc_controller/out/eef_pose", 1, &GroundstationRos::callbackeefPoseLeft,this);

  pub_ob1 = nh.advertise<geometry_msgs::Pose>("/nmpc_controller/in/obstacle1", 1);
  pub_ob2 = nh.advertise<geometry_msgs::Pose>("/nmpc_controller/in/obstacle2", 1);
  pub_ob3 = nh.advertise<geometry_msgs::Pose>("/nmpc_controller/in/obstacle3", 1);

  pub_ob1s = nh.advertise<geometry_msgs::PoseStamped>("/nmpc_controller/in/obstacle1/stamp", 1);
  pub_ob2s = nh.advertise<geometry_msgs::PoseStamped>("/nmpc_controller/in/obstacle2/stamp", 1);

  pub_QOS = nh.advertise<std_msgs::Float64>("QoS", 1);
 
  wp = new WriteParameter();
  Perceptionthresholds = new DeadbandDataReduction(pct);

  if ( enable_log)
    wp->OpenFile();

  filter_right_tx = new Iir::Butterworth::LowPass<order>();
  filter_right_tx->setup(samplingrate,cutoff_frequency);

  filter_right_ty = new Iir::Butterworth::LowPass<order>();
  filter_right_ty->setup(samplingrate,cutoff_frequency);

  filter_right_tz = new Iir::Butterworth::LowPass<order>();
  filter_right_tz->setup(samplingrate,cutoff_frequency);

  filter_right_fx = new Iir::Butterworth::LowPass<order>();
  filter_right_fx->setup(samplingrate,cutoff_frequency);

  filter_right_fy = new Iir::Butterworth::LowPass<order>();
  filter_right_fy->setup(samplingrate,cutoff_frequency);

  filter_right_fz = new Iir::Butterworth::LowPass<order>();
  filter_right_fz->setup(samplingrate,cutoff_frequency);

  //==

  filter_left_fx = new Iir::Butterworth::LowPass<order>();
  filter_left_fx->setup(samplingrate,cutoff_frequency);

  filter_left_fy = new Iir::Butterworth::LowPass<order>();
  filter_left_fy->setup(samplingrate,cutoff_frequency);

  filter_left_fz = new Iir::Butterworth::LowPass<order>();
  filter_left_fz->setup(samplingrate,cutoff_frequency);

  filter_left_tx = new Iir::Butterworth::LowPass<order>();
  filter_left_tx->setup(samplingrate,cutoff_frequency);

  filter_left_ty = new Iir::Butterworth::LowPass<order>();
  filter_left_ty->setup(samplingrate,cutoff_frequency);

  filter_left_tz = new Iir::Butterworth::LowPass<order>();
  filter_left_tz->setup(samplingrate,cutoff_frequency);



}

void GroundstationRos::callbackeefPoseRight(const geometry_msgs::Pose::ConstPtr &msg)
{
  mtx_eff_right.lock();
  //ROS_INFO("Get Rgiht EEF");
  current_mpc_eef_right = *msg;
  mtx_eff_right.unlock();
}

void GroundstationRos::callbackeefPoseLeft(const geometry_msgs::Pose::ConstPtr &msg)
{
  mtx_eff_left.lock();
  //ROS_INFO("Get Left EEF");
  current_mpc_eef_left = *msg;
  mtx_eff_left.unlock();
}

void GroundstationRos::publishDataReduction()
{

   std_msgs::Float32 data1;

   data1.data = velocity_network_rate;
   pub_diagram_velocity_network_rate.publish(data1);

   data1.data = velocity_send;
   pub_diagram_velocity_network_send.publish(data1);

   data1.data = velocity_value;
   pub_diagram_velocity_network_deadband_value.publish(data1);

}

void GroundstationRos::callbackBlock(const nav_msgs::Odometry::ConstPtr &msg)
{
   block_x = msg->pose.pose.position.x;
   block_y = msg->pose.pose.position.y;

   block_q1 = msg->pose.pose.orientation.x;
   block_q2 = msg->pose.pose.orientation.y;
   block_q3 = msg->pose.pose.orientation.z;
   block_q4 = msg->pose.pose.orientation.x;
}

void GroundstationRos::callbackToolPos(const kinova_msgs::KinovaPose::ConstPtr &msg)
{
  hand_pose_x = msg->X;
  hand_pose_y = msg->Y;
  hand_pose_z = msg->Z;

  hand_rot_x = msg->ThetaX;
  hand_rot_y = msg->ThetaY;
  hand_rot_z = msg->ThetaZ;
}

void GroundstationRos::callbackClusters(const joint_control::ArmTrajectory::ConstPtr &msg)
{
  HapticCommands haptic_commands;

  for ( int i = 0 ; i < msg->positions.size() ; i++ )
  {
    RVector3* position = new RVector3();
    position->set_x(msg->positions.at(i).x);
    position->set_y(msg->positions.at(i).y);
    position->set_z(msg->positions.at(i).z);

    haptic_commands.add_list();
    haptic_commands.mutable_list(i)->set_allocated_position(position);
    haptic_commands.mutable_list(i)->set_cmd(std::to_string(msg->class_index.at(i)));
  }

  ground_station->sendML(haptic_commands);
}

void GroundstationRos::callbackBattery(const movo_msgs::Battery::ConstPtr &msg)
{
   mtx_battery.lock();
   current_battery = *msg;
   battery_updated = true;
   mtx_battery.unlock();
}


void GroundstationRos::callbackRightArm(const sensor_msgs::JointState::ConstPtr &msg)
{
   mtx_right_arm.lock();
   //ROS_INFO("GET RIGHT ARM");
   last_right_arm_time =  ros::Time::now();
   arm_valid = true;
   current_right_arm_state = *msg;
   right_arm_updated = true;
   mtx_right_arm.unlock();
}

void GroundstationRos::callbackLeftArm(const sensor_msgs::JointState::ConstPtr &msg)
{
   mtx_left_arm.lock();
   last_left_arm_time =  ros::Time::now();
   arm_valid = true;
   current_left_arm_state = *msg;
   left_arm_updated = true;
   mtx_left_arm.unlock();
}

void GroundstationRos::callbackRightGripper(const sensor_msgs::JointState::ConstPtr &msg)
{
  mtx_right_gripper.lock();
  //TODO update to Hande version
  //current_right_gripper_state = *msg;
  //right_gripper_updated = true;
  mtx_right_gripper.unlock();
}

void GroundstationRos::callbackLeftGripper(const sensor_msgs::JointState::ConstPtr &msg)
{
  mtx_left_gripper.lock();
  //TODO update to Hande version
  //current_left_gripper_state = *msg;
  //left_gripper_updated = true;
  mtx_left_gripper.unlock();
}

void GroundstationRos::callbackHead(const sensor_msgs::JointState::ConstPtr &msg)
{
  mtx_head.lock();
  current_head_state = *msg;
  head_updated = true;
  mtx_head.unlock();
}

void GroundstationRos::callbackZAxes(const sensor_msgs::JointState::ConstPtr &msg)
{
  mtx_z_axes.lock();
  current_z_axes_state = *msg;
  z_axes_updated = true;
  mtx_z_axes.unlock();
}


bool GroundstationRos::is_file_exist(const char *fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}

bool GroundstationRos::loadYaml()
{
  try
  {
    if ( is_file_exist(config_path.c_str()) )
    {
       m_config = YAML::LoadFile(config_path.c_str());

       m_settings.station_5G1_ip = m_config["station_5G1_ip"].as<std::string>();
       m_settings.station_5G2_ip = m_config["station_5G2_ip"].as<std::string>();
       m_settings.station_WIFI_ip = m_config["station_WIFI_ip"].as<std::string>();
       m_settings.station_mode = m_config["station_mode"].as<std::string>();
       m_settings.station_camera_stream = m_config["station_camera_stream"].as<bool>();
       m_settings.station_lidar_stream = m_config["station_lidar_stream"].as<bool>();
       m_settings.station_status_stream = m_config["station_status_stream"].as<bool>();
       m_settings.station_camera_frame_skip_ratio = m_config["station_camera_frame_skip_ratio"].as<int>();
       m_settings.station_camera_resize_ratio = m_config["station_camera_resize_ratio"].as<float>();
       m_settings.station_camera_jpeg_compression_ratio = m_config["station_camera_jpeg_compression_ratio"].as<int>();
       m_settings.print_debug = m_config["print_debug"].as<bool>();

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

bool GroundstationRos::saveYaml()
{
  try
  {

  std::ofstream fout(config_path.c_str()); 

  m_config["station_WIFI_ip"] = m_settings.station_WIFI_ip;
  m_config["station_5G1_ip"] = m_settings.station_5G1_ip;
  m_config["station_5G2_ip"] = m_settings.station_5G2_ip;
  m_config["station_mode"] = m_settings.station_mode;
  m_config["station_camera_stream"] = m_settings.station_camera_stream;
  m_config["station_lidar_stream"] = m_settings.station_lidar_stream;
  m_config["station_status_stream"] = m_settings.station_status_stream;
  m_config["station_camera_frame_skip_ratio"] = m_settings.station_camera_frame_skip_ratio;
  m_config["station_camera_resize_ratio"] = m_settings.station_camera_resize_ratio;
  m_config["station_camera_jpeg_compression_ratio"] = m_settings.station_camera_jpeg_compression_ratio;
  m_config["print_debug"] = m_settings.print_debug;

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

void GroundstationRos::callbackHapticRenderRightArm(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
  if ( right_offset_counter == 20)
  {
    float ax = 0;
    float ay = 0;
    float az = 0;
    float atx = 0;
    float aty = 0;
    float atz = 0;

      for ( int i = 0 ; i < 10 ; i++)
      {
          ax += right_ft_fx_list[i];
          ay += right_ft_fy_list[i];
          az += right_ft_fz_list[i];
          atx += right_ft_tx_list[i];
          aty += right_ft_ty_list[i];
          atz += right_ft_tz_list[i];
      }

      ax = ax / 10;
      ay = ay / 10;
      az = az / 10;
      atx = atx / 10;
      aty = aty / 10;
      atz = atz / 10;

    right_offset.wrench.force.x = ax;
    right_offset.wrench.force.y = ay;
    right_offset.wrench.force.z = az;

    right_offset.wrench.torque.x = atx;
    right_offset.wrench.torque.y = aty;
    right_offset.wrench.torque.z = atz;
  
    right_offset_counter++;
  }
  else if (right_offset_counter < 20)
  {
   right_offset_counter++;

   if ( right_offset_counter > 10)
   {
      right_ft_fx_list.push_back(msg->wrench.force.x);
      right_ft_fy_list.push_back(msg->wrench.force.y);
      right_ft_fz_list.push_back(msg->wrench.force.z);
      right_ft_tx_list.push_back(msg->wrench.torque.x);
      right_ft_ty_list.push_back(msg->wrench.torque.y);
      right_ft_tz_list.push_back(msg->wrench.torque.z);
      
      return;

   }



   return;
  }

  float fx = msg->wrench.force.x - right_offset.wrench.force.x;
  float fy = msg->wrench.force.y - right_offset.wrench.force.y;
  float fz = msg->wrench.force.z - right_offset.wrench.force.z;
  float tx = msg->wrench.torque.x - right_offset.wrench.torque.x;
  float ty = msg->wrench.torque.y - right_offset.wrench.torque.y;
  float tz = msg->wrench.torque.z - right_offset.wrench.torque.z;

  fx = filter_right_fx->filter(fx);
  fy = filter_right_fy->filter(fy);
  fz = filter_right_fz->filter(fz);
  tx = filter_right_tx->filter(tx);
  ty = filter_right_ty->filter(ty);
  tz = filter_right_tz->filter(tz);

  //ROS_INFO_STREAM("Get Force Feedback for Right Arm");
  HapticRender msg_render;
  msg_render.set_version(1);
  msg_render.set_time_span(test_time++);
  msg_render.set_gripper_force(0); //TBD

  RVector3 *force = new RVector3();

  //========================================

  geometry_msgs::Vector3 v1, res;
  v1.x = fx;
  v1.y = fy;
  v1.z = fz;
 
  geometry_msgs::TransformStamped trafo;
  trafo.transform.translation.x = 0;
  trafo.transform.translation.y = 0;
  trafo.transform.translation.z = 0;

  float bx = -1 * current_mpc_eef_right.orientation.z;
  float by = current_mpc_eef_right.orientation.x;
  float bz = -1 * current_mpc_eef_right.orientation.y;
  float bw = current_mpc_eef_right.orientation.w;

  trafo.transform.rotation = tf2::toMsg(tf2::Quaternion(bx,by,bz,bw));
 
  tf2::doTransform(v1, res, trafo);
   
  force->set_x(res.z / -3);
  force->set_y(res.y /  3);
  force->set_z(res.x /  3);

  //force->set_x(fx);
  //force->set_y(fy);
  //force->set_z(fz);

  RVector3 *torque = new RVector3();

  torque->set_x(tx / 3);
  torque->set_y(ty / 3);
  torque->set_z(tz / 3);

  //torque->set_x(tx);
  //torque->set_y(ty);
  //torque->set_z(tz);

    // ft_fx_list.push_back(res.z / -5);
    // ft_fy_list.push_back(res.y /  5);
    // ft_fz_list.push_back(res.x /  5);

    // ft_tx_list.push_back(tx / 10);
    // ft_ty_list.push_back(ty / 10);
    // ft_tz_list.push_back(tz / 10);
    
    // int size = 2;

    // if ( window_size >= size)
    // {
    // ft_fx_list.erase(std::next(ft_fx_list.begin()));
    // ft_fy_list.erase(std::next(ft_fy_list.begin()));
    // ft_fz_list.erase(std::next(ft_fz_list.begin()));
    // ft_tx_list.erase(std::next(ft_tx_list.begin()));
    // ft_ty_list.erase(std::next(ft_ty_list.begin()));
    // ft_tz_list.erase(std::next(ft_tz_list.begin()));
    // }
    // else
    // {
    //   window_size++;
    // }

    // if ( window_size >= size)
    // {
    //    float avg_fx = 1.0 * std::accumulate(ft_fx_list.begin(), ft_fx_list.end(), 0LL) / ft_fx_list.size();
    //    float avg_fy = 1.0 * std::accumulate(ft_fy_list.begin(), ft_fy_list.end(), 0LL) / ft_fy_list.size();
    //    float avg_fz = 1.0 * std::accumulate(ft_fz_list.begin(), ft_fz_list.end(), 0LL) / ft_fz_list.size();
    //    float avg_tx = 1.0 * std::accumulate(ft_tx_list.begin(), ft_tx_list.end(), 0LL) / ft_tx_list.size();
    //    float avg_ty = 1.0 * std::accumulate(ft_ty_list.begin(), ft_ty_list.end(), 0LL) / ft_ty_list.size();
    //    float avg_tz = 1.0 * std::accumulate(ft_tz_list.begin(), ft_tz_list.end(), 0LL) / ft_tz_list.size();

    //    //avg_fx = ((int)avg_fx);
    //    //avg_fy = ((int)avg_fy);
    //    //avg_fz = ((int)avg_fz);
    //    //avg_tx = ((int)avg_tx);
    //    //avg_ty = ((int)avg_ty);
    //    //avg_tz = ((int)avg_tz);

    //    force->set_x(res.z / -5);
    //    force->set_y(res.y /  5);
    //    force->set_z(res.x /  5);

    //    torque->set_x(tx / 10);
    //    torque->set_y(ty / 10);
    //    torque->set_z(tz / 10);

    msg_render.set_allocated_force(force);
    msg_render.set_allocated_torque(torque);

    geometry_msgs::WrenchStamped msg2;
    msg2.wrench.force.x = force->x() ;
    msg2.wrench.force.y = force->y() ;
    msg2.wrench.force.z = force->z() ;

    msg2.wrench.torque.x = torque->x() ;
    msg2.wrench.torque.y = torque->y() ;
    msg2.wrench.torque.z = torque->z() ;

    msg2.header.frame_id = msg->header.frame_id;
    msg2.header.stamp =  ros::Time::now();
    
    //ROS_INFO_STREAM("TIME " <<  ros::Time::now());
    pub_ft_right.publish(msg2);
    sendForceRight(msg_render);

      //if (send_flag)
      // {
          

          //if ( enable_log )
           // cout << "Sending Force...    d:" << pct << " v:" << value << endl;
      // }


   // }
  
  //--

  //Deadband JND 

  // currentSample[0] = res.z;
  // currentSample[1] = res.x;
  // currentSample[2] = res.y;

  // currentSample[3] = tx;
  // currentSample[4] = ty;
  // currentSample[5] = tz;

  // currentSample[6] = 0;

  // Perceptionthresholds->GetCurrentSample(currentSample);
  // Perceptionthresholds->ApplyZOHDeadband(updatedSample, &send_flag, &value);
  
 
  // return;

  // time_end = std::chrono::system_clock::now();
  // std::time_t delta_time = std::chrono::system_clock::to_time_t(time_end);

  // if (delta_time - old_delta_time != 0)
  // {
  //         std_msgs::Float32 data1;

  //     data1.data = force_network_rate;
  //     pub_diagram_force_network_rate.publish(data1);
      
  //     data1.data = force_send;
  //     pub_diagram_force_network_send.publish(data1);
      
  //     data1.data = force_value;
  //     pub_diagram_force_network_deadband_value.publish(data1);
  //     //1 second
  //     data_rate_ps = network_data_rate;
  //     network_data_rate = 0;
  // }

  // if ( enable_log)
  //   wp->Write(delta_time, fx, fy, fz, tx, ty, tz, 0, value, send_flag, data_rate_ps);

  // old_delta_time = delta_time;

  // force_network_rate = network_data_rate;
  // force_send = send_flag;
  // force_value = value;

  // pub_diagram_force_network_force.publish(*msg);

}

void GroundstationRos::callbackHapticRenderLeftArm(const geometry_msgs::WrenchStamped::ConstPtr &msg)
{
   if ( left_offset_counter == 20)
  {
    float ax = 0;
    float ay = 0;
    float az = 0;
    float atx = 0;
    float aty = 0;
    float atz = 0;

      for ( int i = 0 ; i < 10 ; i++)
      {
          ax += left_ft_fx_list[i];
          ay += left_ft_fy_list[i];
          az += left_ft_fz_list[i];
          atx += left_ft_tx_list[i];
          aty += left_ft_ty_list[i];
          atz += left_ft_tz_list[i];
      }

      ax = ax / 10;
      ay = ay / 10;
      az = az / 10;
      atx = atx / 10;
      aty = aty / 10;
      atz = atz / 10;

    left_offset.wrench.force.x = ax;
    left_offset.wrench.force.y = ay;
    left_offset.wrench.force.z = az;

    left_offset.wrench.torque.x = atx;
    left_offset.wrench.torque.y = aty;
    left_offset.wrench.torque.z = atz;

    left_offset_counter++;
  }
  else if (left_offset_counter < 20)
  {
   left_offset_counter++;

   if ( left_offset_counter > 10)
   {
      left_ft_fx_list.push_back(msg->wrench.force.x);
      left_ft_fy_list.push_back(msg->wrench.force.y);
      left_ft_fz_list.push_back(msg->wrench.force.z);
      left_ft_tx_list.push_back(msg->wrench.torque.x);
      left_ft_ty_list.push_back(msg->wrench.torque.y);
      left_ft_tz_list.push_back(msg->wrench.torque.z);
      
      return;

   }



   return;
  }

  float fx = msg->wrench.force.x - left_offset.wrench.force.x;
  float fy = msg->wrench.force.y - left_offset.wrench.force.y;
  float fz = msg->wrench.force.z - left_offset.wrench.force.z;
  float tx = msg->wrench.torque.x - left_offset.wrench.torque.x;
  float ty = msg->wrench.torque.y - left_offset.wrench.torque.y;
  float tz = msg->wrench.torque.z - left_offset.wrench.torque.z;
  
  fx = filter_left_fx->filter(fx);
  fy = filter_left_fy->filter(fy);
  fz = filter_left_fz->filter(fz);
  tx = filter_left_tx->filter(tx);
  ty = filter_left_ty->filter(ty);
  tz = filter_left_tz->filter(tz);

  //---
  // left_ft_fx_list.push_back(fx);
  // left_ft_fy_list.push_back(fy);
  // left_ft_fz_list.push_back(fz);

  // left_ft_tx_list.push_back(tx);
  // left_ft_ty_list.push_back(ty);
  // left_ft_tz_list.push_back(tz);
  
  // int size = 100;

  // if ( left_window_size >= size)
  // {
  //   left_ft_fx_list.erase(std::next(left_ft_fx_list.begin()));
  //   left_ft_fy_list.erase(std::next(left_ft_fy_list.begin()));
  //   left_ft_fz_list.erase(std::next(left_ft_fz_list.begin()));
  //   left_ft_tx_list.erase(std::next(left_ft_tx_list.begin()));
  //   left_ft_ty_list.erase(std::next(left_ft_ty_list.begin()));
  //   left_ft_tz_list.erase(std::next(left_ft_tz_list.begin()));
  // }
  // else
  // {
  //   left_window_size++;
  // }

  // if ( left_window_size >= size)
  // {
  //     float avg_fx = 1.0 * std::accumulate(left_ft_fx_list.begin(), left_ft_fx_list.end(), 0LL) / left_ft_fx_list.size();
  //     float avg_fy = 1.0 * std::accumulate(left_ft_fy_list.begin(), left_ft_fy_list.end(), 0LL) / left_ft_fy_list.size();
  //     float avg_fz = 1.0 * std::accumulate(left_ft_fz_list.begin(), left_ft_fz_list.end(), 0LL) / left_ft_fz_list.size();
  //     float avg_tx = 1.0 * std::accumulate(left_ft_tx_list.begin(), left_ft_tx_list.end(), 0LL) / left_ft_tx_list.size();
  //     float avg_ty = 1.0 * std::accumulate(left_ft_ty_list.begin(), left_ft_ty_list.end(), 0LL) / left_ft_ty_list.size();
  //     float avg_tz = 1.0 * std::accumulate(left_ft_tz_list.begin(), left_ft_tz_list.end(), 0LL) / left_ft_tz_list.size();

  //     float norm1 = sqrt(fx * fx + fy * fy + fz * fz);
  //     float norm2 = sqrt(tx * tx + ty * ty + tz * tz);

  //     //norm2 = norm2 / 0.17;

  //     ROS_INFO_STREAM("Left weight F: " << ( norm1 / 9.8 ) * 1000 << " T: " << ( norm2 / 9.8 ) * 1000 << " g");
  //     ROS_INFO("===");
  // }

  HapticRender msg_render;
  msg_render.set_version(1);
  msg_render.set_time_span(test_time++);
  msg_render.set_gripper_force(0); //TBD

  RVector3 *force = new RVector3();

  force->set_x(fx);
  force->set_y(fy);
  force->set_z(fz);

  RVector3 *torque = new RVector3();

  torque->set_x(tx);
  torque->set_y(ty);
  torque->set_z(tz);

  msg_render.set_allocated_force(force);
  msg_render.set_allocated_torque(torque);

  geometry_msgs::WrenchStamped msg2;
  msg2.wrench.force.x = force->x() ;
  msg2.wrench.force.y = force->y() ;
  msg2.wrench.force.z = force->z() ;

  msg2.wrench.torque.x = torque->x() ;
  msg2.wrench.torque.y = torque->y() ;
  msg2.wrench.torque.z = torque->z() ;

  msg2.header.frame_id = msg->header.frame_id;
  msg2.header.stamp =  ros::Time::now();
  
  //ROS_INFO_STREAM("TIME " <<  ros::Time::now());
  pub_ft_left.publish(msg2);
  sendForceLeft(msg_render);
}

void GroundstationRos::playVoice(std::string text)
{
    ROS_WARN_STREAM("Play voice : " << text);
    std_msgs::String msg;
    msg.data =  text;
    pub_voice_cmd.publish(msg);
}

void GroundstationRos::sendGoal(std::string text)
{
	std::string voice_nav = "Navigation Goal";
	voice_nav += " " + text;

	std_msgs::String msg1;
  msg1.data =  voice_nav;
	pub_voice_cmd.publish(msg1);

	ROS_WARN_STREAM("Send navigation goal : " << text);

	if ( text == "cancel")
	{
		robot_navigation_control_mode = 0; // Manual
	}
	else
	{
		robot_navigation_control_mode = 1; // Autonomous
	}
	
    std_msgs::String msg;
    msg.data =  text;
    pub_nav_cmd.publish(msg);
}


void GroundstationRos::callbackHapticRight(HapticCommand cmd)
{
     bool debug = false;

     if ( debug )
     {
        ROS_INFO_STREAM("Get Haptic Command from Master [RIGHT HAND] " << cmd.version());
        ROS_INFO_STREAM("Position " << cmd.position().x() << " " << cmd.position().y() << " " << cmd.position().z());
        ROS_INFO_STREAM("Rotation " << cmd.rotation().x() << " " << cmd.rotation().y() << " " << cmd.rotation().z());
        ROS_INFO_STREAM("Linear Velocity " << cmd.velocity_linear().x() << " " << cmd.velocity_linear().y() << " " << cmd.velocity_linear().z());
        ROS_INFO_STREAM("Angular Velocity " << cmd.velocity_angular().x() << " " << cmd.velocity_angular().y() << " " << cmd.velocity_angular().z());
        ROS_INFO_STREAM("Gripper angle " << cmd.gripper_angle());
        ROS_INFO_STREAM("Gripper gap " << cmd.gripper_gap());
        ROS_INFO_STREAM("Gripper linear velocity " << cmd.velocity_linear_gripper());
        ROS_INFO_STREAM("Gripper angular velocity " << cmd.velocity_angular_gripper());
        ROS_INFO("===" );
     }

    joint_control::HapticCommand right_position_command_msg;

    //Position
    right_position_command_msg.position.push_back(cmd.position().x());
    right_position_command_msg.position.push_back(cmd.position().y());
    right_position_command_msg.position.push_back(cmd.position().z());

    //Rotation
    right_position_command_msg.rotation.push_back(cmd.rotation().x());
    right_position_command_msg.rotation.push_back(cmd.rotation().y());
    right_position_command_msg.rotation.push_back(cmd.rotation().z());

    //Quaternion
    right_position_command_msg.rotation_q.push_back(cmd.rotation_q().x());
    right_position_command_msg.rotation_q.push_back(cmd.rotation_q().y());
    right_position_command_msg.rotation_q.push_back(cmd.rotation_q().z());
    right_position_command_msg.rotation_q.push_back(cmd.rotation_q().w());

    for ( int i = 0; i < 9 ; i++)
    right_position_command_msg.rotation_matrix.push_back(cmd.rotation_matrix(i));

    right_position_command_msg.linear_velocity.push_back(cmd.velocity_linear().x());
    right_position_command_msg.linear_velocity.push_back(cmd.velocity_linear().y());
    right_position_command_msg.linear_velocity.push_back(cmd.velocity_linear().z());
    right_position_command_msg.angular_velocity.push_back(cmd.velocity_angular().x());
    right_position_command_msg.angular_velocity.push_back(cmd.velocity_angular().y());
    right_position_command_msg.angular_velocity.push_back(cmd.velocity_angular().z());
    right_position_command_msg.gripper_angle = cmd.gripper_angle();
    right_position_command_msg.gripper_gap = cmd.gripper_gap();
    right_position_command_msg.gripper_linear_velocity = cmd.velocity_linear_gripper();
    right_position_command_msg.gripper_angular_velocity = cmd.velocity_angular_gripper();
    right_position_command_msg.cmd = cmd.cmd();

    pub_right_arm_haptic_control.publish(right_position_command_msg);
}

void GroundstationRos::callbackHapticLeft(HapticCommand cmd)
{
    bool debug = false;

    if ( debug )
    {
    ROS_INFO_STREAM("Get Haptic Command from Master [LEFT HAND] " << cmd.version());
    ROS_INFO_STREAM("Position " << cmd.position().x() << " " << cmd.position().y() << " " << cmd.position().z());
    ROS_INFO_STREAM("Rotation " << cmd.rotation().x() << " " << cmd.rotation().y() << " " << cmd.rotation().z());
    ROS_INFO_STREAM("Linear Velocity " << cmd.velocity_linear().x() << " " << cmd.velocity_linear().y() << " " << cmd.velocity_linear().z());
    ROS_INFO_STREAM("Angular Velocity " << cmd.velocity_angular().x() << " " << cmd.velocity_angular().y() << " " << cmd.velocity_angular().z());
    ROS_INFO_STREAM("Gripper angle " << cmd.gripper_angle());
    ROS_INFO_STREAM("Gripper gap " << cmd.gripper_gap());
    ROS_INFO_STREAM("Gripper linear velocity " << cmd.velocity_linear_gripper());
    ROS_INFO_STREAM("Gripper angular velocity " << cmd.velocity_angular_gripper());
    ROS_INFO("===" );
    }

    joint_control::HapticCommand left_position_command_msg;

    //Position
    left_position_command_msg.position.push_back(cmd.position().x());
    left_position_command_msg.position.push_back(cmd.position().y());
    left_position_command_msg.position.push_back(cmd.position().z());

    //Rotation
    left_position_command_msg.rotation.push_back(cmd.rotation().x());
    left_position_command_msg.rotation.push_back(cmd.rotation().y());
    left_position_command_msg.rotation.push_back(cmd.rotation().z());

    //Quaternion
    left_position_command_msg.rotation_q.push_back(cmd.rotation_q().x());
    left_position_command_msg.rotation_q.push_back(cmd.rotation_q().y());
    left_position_command_msg.rotation_q.push_back(cmd.rotation_q().z());
    left_position_command_msg.rotation_q.push_back(cmd.rotation_q().w());

    for ( int i = 0; i < 9 ; i++)
    left_position_command_msg.rotation_matrix.push_back(cmd.rotation_matrix(i));

    left_position_command_msg.linear_velocity.push_back(cmd.velocity_linear().x());
    left_position_command_msg.linear_velocity.push_back(cmd.velocity_linear().y());
    left_position_command_msg.linear_velocity.push_back(cmd.velocity_linear().z());
    left_position_command_msg.angular_velocity.push_back(cmd.velocity_angular().x());
    left_position_command_msg.angular_velocity.push_back(cmd.velocity_angular().y());
    left_position_command_msg.angular_velocity.push_back(cmd.velocity_angular().z());
    left_position_command_msg.gripper_angle = cmd.gripper_angle();
    left_position_command_msg.gripper_gap = cmd.gripper_gap();
    left_position_command_msg.gripper_linear_velocity = cmd.velocity_linear_gripper();
    left_position_command_msg.gripper_angular_velocity = cmd.velocity_angular_gripper();
    left_position_command_msg.cmd = cmd.cmd();
    
    pub_left_arm_haptic_control.publish(left_position_command_msg);

    pub_diagram_velocity_network_command.publish(left_position_command_msg);
}

void GroundstationRos::sendForceRight(HapticRender cmd)
{
  int size = cmd.ByteSize();
  network_data_rate += size;
  ground_station->sendForceRight(cmd);
}

void GroundstationRos::sendForceLeft(HapticRender cmd)
{
  ground_station->sendForceLeft(cmd);
}

void GroundstationRos::callbackML(HapticCommand cmd)
{
   float size = cmd.position().x();
   float distance = cmd.position().y();

   joint_control::HapticCommand command;
   command.position.push_back(size);
   command.position.push_back(distance);

   pub_ml.publish(command);

   //ROS_INFO_STREAM("New ML Settings " << size << " " << distance);
   
}

void GroundstationRos::callbackGroundstation(MovoCommand cmd)
{
    get_new = true;
    counter = 0;

    last_cmd_vel_time = ros::Time::now();
    set_stop = false;

    if ( home_wait == 0)
    {

      if ( cmd.voice() == 1 ) playVoice("Hello, my name is MOVO.");
      if ( cmd.voice() == 2 ) playVoice("Welcome to 5G research hub Munich. I hope you will enjoy my demos.");
     
      if ( cmd.voice() == 10) playVoice("Recording");
      if ( cmd.voice() == 101) playVoice("Pause");
      if ( cmd.voice() == 102) playVoice("stop");

      if ( cmd.voice() == 11) playVoice("Reverse Playback");
      if ( cmd.voice() == 111) playVoice("Pause");
      if ( cmd.voice() == 112) playVoice("stop");

      if ( cmd.voice() == 12) playVoice("Playback");
      if ( cmd.voice() == 121) playVoice("Pause");
      if ( cmd.voice() == 122) playVoice("stop");

      if ( cmd.voice() == 100) playVoice("PourNet Control");
      if ( cmd.voice() == 101) playVoice("Sigma Control");
      if ( cmd.voice() == 200) playVoice("Joystick Control");
      if ( cmd.voice() == 300) 
      {
        playVoice("Get Offset");
        
        right_ft_fx_list.clear();
        right_ft_fy_list.clear();
        right_ft_fz_list.clear();
        right_ft_tx_list.clear();
        right_ft_ty_list.clear();
        right_ft_tz_list.clear();

        left_ft_fx_list.clear();
        left_ft_fy_list.clear();
        left_ft_fz_list.clear();
        left_ft_tx_list.clear();
        left_ft_ty_list.clear();
        left_ft_tz_list.clear();

        right_offset_counter = 0;
        left_offset_counter = 0;

      }

      if ( cmd.cmd() != old_cmd && cmd.cmd() == "home" && cmd.cmd() != "") 
      {
        std_srvs::Empty srv;
        cl_arm_home.call(srv);

        ROS_ERROR("Send home for arms");
        playVoice("Moving my arms to the home position, please wait 15 seconds");
        home_wait = 15000;
      }

      if ( cmd.cmd() != old_cmd && cmd.cmd() != "home" && cmd.cmd() != "auto" && cmd.cmd() != "tele" && cmd.cmd() != "" ) 
      {
        sendGoal(cmd.cmd());
      }

      if ( cmd.cmd() != old_cmd && cmd.cmd() != "home" && cmd.cmd() == "auto" && cmd.cmd() != "" ) 
      {
      	ROS_WARN("Change to Auto navigation mode");
        robot_navigation_control_mode = 1; // Autonomous
      }

      if ( cmd.cmd() != old_cmd && cmd.cmd() != "home" && cmd.cmd() == "tele" && cmd.cmd() != "" ) 
      {
      	ROS_WARN("Change to Tele navigation mode");
        robot_navigation_control_mode = 0; // Manual
      }

    sendOmni(cmd.speed_x(),cmd.speed_y(),cmd.speed_w());
    sendHead(cmd.speed_head_x(),cmd.speed_head_y(),cmd.speed_head_w());
    sendZ(cmd.zposition());
    sendObservation(cmd.observation());

    }

    velocity_network_rate = cmd.haptic_reduction_monitor().x();
    velocity_value = cmd.haptic_reduction_monitor().y();;
    velocity_send = cmd.haptic_reduction_monitor().z();;

    old_cmd = cmd.cmd();
}

void GroundstationRos::sendObservation(ObservationRL data)
{
    geometry_msgs::Pose obj1;
    geometry_msgs::Pose obj2;

    obj1.position.x = data.a_position().x();
    obj1.position.y = data.a_position().y();
    obj1.position.z = data.a_position().z();

    obj1.orientation.x = data.a_rotation_q().x();
    obj1.orientation.y = data.a_rotation_q().y();
    obj1.orientation.z = data.a_rotation_q().z();
    obj1.orientation.w = data.a_rotation_q().w();

    obj2.position.x = data.b_position().x();
    obj2.position.y = data.b_position().y();
    obj2.position.z = data.b_position().z();

    obj2.orientation.x = data.b_rotation_q().x();
    obj2.orientation.y = data.b_rotation_q().y();
    obj2.orientation.z = data.b_rotation_q().z();
    obj2.orientation.w = data.b_rotation_q().w();

    Eigen::Translation3d translation1(data.a_position().x(),data.a_position().y(),data.a_position().z());
    Eigen::Quaterniond quat1(data.a_rotation_q().w(),data.a_rotation_q().x(),data.a_rotation_q().y(),data.a_rotation_q().z());
    Eigen::Isometry3d transform1 = translation1*quat1;

    Eigen::Translation3d translation2(data.b_position().x(),data.b_position().y(),data.b_position().z()); 
    Eigen::Quaterniond quat2(data.b_rotation_q().w(),data.b_rotation_q().x(),data.b_rotation_q().y(),data.b_rotation_q().z());
    Eigen::Isometry3d transform2 = translation2*quat2;
    auto tranform2_to1 = transform1.inverse()*transform2;

    Eigen::Isometry3d transform1_to_upper_base = Eigen::Isometry3d::Identity();
    transform1_to_upper_base.linear() << 0,0,1,0,-1,0,1,0,0;
    transform1_to_upper_base.translation() << -0.0771402,0,0.64;

    auto transform2_to_upper_base = transform1_to_upper_base*tranform2_to1;
    Eigen::Quaterniond quat2_to_upper_base(transform2_to_upper_base.linear());
    const auto& translation2_to_upper_base = transform2_to_upper_base.translation();


    geometry_msgs::Pose msg3;
    msg3.position.x = translation2_to_upper_base.x();
    msg3.position.y = translation2_to_upper_base.y();
    msg3.position.z = translation2_to_upper_base.z();

    msg3.orientation.x = quat2_to_upper_base.x();
    msg3.orientation.y = quat2_to_upper_base.y();
    msg3.orientation.z = quat2_to_upper_base.z();
    msg3.orientation.w = quat2_to_upper_base.w();

    // msgs3.header.frame_id = "upper_body_link";
    pub_ob3.publish(msg3);
    



    // tf::Transform transform1;
    // transform1.setOrigin( tf::Vector3(obj1.position.x, obj1.position.y, obj1.position.z) );
    // transform1.setRotation( tf::Quaternion(obj1.orientation.x, obj1.orientation.y, obj1.orientation.z, obj1.orientation.w) );

    // tf::Transform transform2;
    // transform2.setOrigin( tf::Vector3(obj2.position.x, obj2.position.y, obj2.position.z) );
    // transform2.setRotation( tf::Quaternion(obj2.orientation.x, obj2.orientation.y, obj2.orientation.z, obj2.orientation.w) );

    // tf::Transform transform3 = transform1.inverseTimes(transform2);

    // obj2.position.x = transform3.getOrigin().getX();
    // obj2.position.y = transform3.getOrigin().getY();
    // obj2.position.z = transform3.getOrigin().getZ();

    // obj2.orientation.x = transform3.getRotation().x();
    // obj2.orientation.y = transform3.getRotation().y();
    // obj2.orientation.z = transform3.getRotation().z();
    // obj2.orientation.w = transform3.getRotation().w();

    geometry_msgs::PoseStamped msgs1;

    msgs1.pose.position.x = obj1.position.x;
    msgs1.pose.position.y = obj1.position.y;
    msgs1.pose.position.z = obj1.position.z;

    msgs1.pose.orientation.x = obj1.orientation.x;
    msgs1.pose.orientation.y = obj1.orientation.y;
    msgs1.pose.orientation.z = obj1.orientation.z;
    msgs1.pose.orientation.w = obj1.orientation.w;

    msgs1.header.frame_id = "base_link";

    geometry_msgs::PoseStamped msgs2;

    msgs2.pose.position.x =  obj2.position.x;
    msgs2.pose.position.y =  obj2.position.y;
    msgs2.pose.position.z =  obj2.position.z;

    msgs2.pose.orientation.x = obj2.orientation.x;
    msgs2.pose.orientation.y = obj2.orientation.y;
    msgs2.pose.orientation.z = obj2.orientation.z;
    msgs2.pose.orientation.w = obj2.orientation.w;

    msgs2.header.frame_id = "base_link";

    pub_ob1.publish(obj1);
    pub_ob2.publish(obj2);

    pub_ob1s.publish(msgs1);
    pub_ob2s.publish(msgs2);
}

void GroundstationRos::thrMain()
{
    MovoStatus status;

    status.set_version(1);

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    
    last_cmd_vel_time = ros::Time::now();
    
    while(ros::ok() && app_exit == false)
    {
        float qos_ok = 1;

        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        long t2 = (ros::Time::now() - last_right_arm_time).toNSec();
        long t3 = (ros::Time::now() - last_left_arm_time).toNSec();

        t2 = t2 / 1000000;
        t3 = t3 / 1000000;

        long t = (ros::Time::now() - last_cmd_vel_time).toNSec();
        t = t / 1000000;
        if ( t > 500 && set_stop == false)
        {
            set_stop = true;
            ROS_WARN("Detect message lost, no data after 0.5 second, send zero to navigation system");
            //playVoice("Disconnected from teleoperation station");

            //Reset robot speeds
            sendOmni(0,0,0);
		    
            //Arm
            joint_control::HapticCommand haptic_position_command_msg;

            haptic_position_command_msg.position.push_back(0); //invalid
            haptic_position_command_msg.position.push_back(0); //invalid
            haptic_position_command_msg.position.push_back(0); //invalid
            haptic_position_command_msg.rotation.push_back(0); //invalid
            haptic_position_command_msg.rotation.push_back(0); //invalid
            haptic_position_command_msg.rotation.push_back(0); //invalid
            haptic_position_command_msg.linear_velocity.push_back(0);
            haptic_position_command_msg.linear_velocity.push_back(0);
            haptic_position_command_msg.linear_velocity.push_back(0);
            haptic_position_command_msg.angular_velocity.push_back(0);
            haptic_position_command_msg.angular_velocity.push_back(0);
            haptic_position_command_msg.angular_velocity.push_back(0);
            haptic_position_command_msg.gripper_angle = 0; //invalid
            haptic_position_command_msg.gripper_gap = 0;   //invalid
            haptic_position_command_msg.gripper_linear_velocity = 0;
            haptic_position_command_msg.gripper_angular_velocity = 0;
            haptic_position_command_msg.cmd = "";
            
            pub_right_arm_haptic_control.publish(haptic_position_command_msg);
            pub_left_arm_haptic_control.publish(haptic_position_command_msg);
        }

        if ( t > 100)
        {
           qos_ok = 0;
        }

        if (t2 > 2000 || t3 > 2000 )
        {
          arm_valid = false;
        }

        Laser *laser_data = new Laser();

        if ( send_lidar_to_station )
        {
         
          laser_data->set_robot_id(0);
          laser_data->clear_ranges();

          for ( int i = 0 ; i < current_laser.ranges.size() ; i++)
          {
              //if ( i % 5 == 0)
              laser_data->add_ranges(current_laser.ranges.at(i));
          }


          status.set_allocated_laser(laser_data);
        }

        if ( send_arm_to_station )
        {

        if ( right_arm_updated && left_arm_updated && head_updated && z_axes_updated)
        {
          //set arms status
          MovoArm *right_arm = new MovoArm();
          MovoArm *left_arm = new MovoArm();
          MovoHead *head = new MovoHead();

          for ( int i = 0; i < 7 ; i++)
          {
             right_arm->add_joints();
             left_arm->add_joints();

             right_arm->mutable_joints(i)->set_position(current_right_arm_state.position[i]);
             right_arm->mutable_joints(i)->set_effort(current_right_arm_state.effort[i]);
             right_arm->mutable_joints(i)->set_velocity(current_right_arm_state.velocity[i]);

             left_arm->mutable_joints(i)->set_position(current_left_arm_state.position[i]);
             left_arm->mutable_joints(i)->set_effort(current_left_arm_state.effort[i]);
             left_arm->mutable_joints(i)->set_velocity(current_left_arm_state.velocity[i]);
          }

          for ( int i = 0 ; i < 2; i++)
          {
              head->add_joints();

              head->mutable_joints(i)->set_position(current_head_state.position[i]);
              head->mutable_joints(i)->set_effort(current_head_state.effort[i]);
              head->mutable_joints(i)->set_velocity(current_head_state.velocity[i]);
          }

          //MovoJoint *right_gripper = new MovoJoint();
          //MovoJoint *left_gripper = new MovoJoint();

          //right_gripper->set_position((current_right_gripper_state.position[0] + current_right_gripper_state.position[1] + current_right_gripper_state.position[2]) / 3);
          //right_gripper->set_effort((current_right_gripper_state.effort[0] + current_right_gripper_state.effort[1] + current_right_gripper_state.effort[2]) / 3);
          //right_gripper->set_velocity((current_right_gripper_state.velocity[0] +  current_right_gripper_state.velocity[1] + current_right_gripper_state.velocity[2]) / 3);

          //left_gripper->set_position((current_left_gripper_state.position[0] + current_left_gripper_state.position[1] + current_left_gripper_state.position[2]) / 3);
          //left_gripper->set_effort((current_left_gripper_state.effort[0] + current_left_gripper_state.effort[1] + current_left_gripper_state.effort[2]) / 3);
          //left_gripper->set_velocity((current_left_gripper_state.velocity[0] +  current_left_gripper_state.velocity[1] + current_left_gripper_state.velocity[2]) / 3);

          //right_arm->set_allocated_gripper(right_gripper);
          //left_arm->set_allocated_gripper(left_gripper);

          //--

          status.set_z_axes(current_z_axes_state.position[0]);
          status.set_allocated_right_arm(right_arm);
          status.set_allocated_left_arm(left_arm);
          status.set_allocated_head(head);

         
          //ROS_INFO("NEW ARM STATUS UPDATED");
        }
        else
        {
          //ROS_INFO_STREAM("1 " << right_arm_updated);
          //ROS_INFO_STREAM("2 " << left_arm_updated);
          //ROS_INFO_STREAM("3 " << right_gripper_updated);
          //ROS_INFO_STREAM("4 " << left_gripper_updated);
          //ROS_INFO_STREAM("5 " << z_axes_updated);
          //ROS_INFO_STREAM("6 " << head_updated);
        }

        }

        status.set_battery(current_battery.battery_soc);

        //Add eef position (MPC version)
        RVector7 *q_left_arm_end_effector = new RVector7();
       

        mtx_eff_left.lock();

        q_left_arm_end_effector->set_x(current_mpc_eef_left.position.x);
        q_left_arm_end_effector->set_y(current_mpc_eef_left.position.y);
        q_left_arm_end_effector->set_z(current_mpc_eef_left.position.z);

        // tf::Quaternion q(current_mpc_eef_left.orientation.x,current_mpc_eef_left.orientation.y,current_mpc_eef_left.orientation.z,current_mpc_eef_left.orientation.w);
        // tf::Matrix3x3 m(q);
        // double roll, pitch, yaw;
        // m.getRPY(roll, pitch, yaw);

        //ROS_INFO_STREAM("Q  " << q[0] << " " << q[1] << " " << q[2] << " " << q[3]);
        //ROS_INFO_STREAM("Q TO E " << roll << " " << pitch << " " << yaw);

        //tf::Quaternion q2;
        //q2.setEulerZYX(yaw,pitch,roll);

        //ROS_INFO_STREAM("Q back " << q2[0] << " " << q2[1] << " " << q2[2] << " " << q2[3]);

        q_left_arm_end_effector->set_qx(current_mpc_eef_left.orientation.x);
        q_left_arm_end_effector->set_qy(current_mpc_eef_left.orientation.y);
        q_left_arm_end_effector->set_qz(current_mpc_eef_left.orientation.z);
        q_left_arm_end_effector->set_qw(current_mpc_eef_left.orientation.w);

        mtx_eff_left.unlock();


        RVector7 *q_right_arm_end_effector = new RVector7();

        mtx_eff_right.lock();

        q_right_arm_end_effector->set_x(current_mpc_eef_right.position.x);
        q_right_arm_end_effector->set_y(current_mpc_eef_right.position.y);
        q_right_arm_end_effector->set_z(current_mpc_eef_right.position.z);

        q_right_arm_end_effector->set_qx(current_mpc_eef_right.orientation.x);
        q_right_arm_end_effector->set_qy(current_mpc_eef_right.orientation.y);
        q_right_arm_end_effector->set_qz(current_mpc_eef_right.orientation.z);
        q_right_arm_end_effector->set_qw(current_mpc_eef_right.orientation.w);

        mtx_eff_right.unlock();
        //Add block position (RL)
        //RVector3 *temp_target_location = new RVector3();
        //SVector4 *temp_target_rotation = new SVector4();

        //temp_target_location->set_x(block_x);
        //temp_target_location->set_y(block_y);

        //temp_target_rotation->set_x(block_q1);
        //temp_target_rotation->set_y(block_q2);
        //temp_target_rotation->set_z(block_q3);
        //temp_target_rotation->set_w(block_q4);

        //status.set_allocated_temp_target_location(temp_target_location);
        //status.set_allocated_temp_target_rotation(temp_target_rotation);

        status.set_allocated_q_left_arm_end_effector(q_left_arm_end_effector);
        status.set_allocated_q_right_arm_end_effector(q_right_arm_end_effector);

        ground_station->sendStatus(status);

        laser_data = NULL;
        delete laser_data;
        

        if ( home_wait > 0)
        {
            home_wait = home_wait - 100;
            ROS_WARN_STREAM("Waiting  " << home_wait);
        }

        publishDataReduction();

        //ROS_WARN_STREAM("Status message size is " << status.ByteSize());
        std_msgs::Bool msg_b;
        msg_b.data = arm_valid;
        pub_arm_status.publish(msg_b);

        std_msgs::Float64 msg_qos;
        msg_qos.data = qos_ok;
        pub_QOS.publish(msg_qos);
    }
}

void GroundstationRos::sendOmni(double x,double y ,double w)
{
    geometry_msgs::Twist msg;
    
    x = x / 250;
    y = -1 * y / 250;
    w = -1 * w / 100;

    msg.linear.x = x;
    msg.linear.y = y;
    msg.angular.z = w;

    if ( home_wait == 0)
    {
    	if ( robot_navigation_control_mode == 0) // only manual
      			pub_base_cmd.publish(msg);

      //ROS_INFO_STREAM("Sending value to MOVO base " << " " << x << " " << y << " " << w );
    }
}

void GroundstationRos::sendZ(int x)
{
    float f_x = (float)x / 1000;

    if ( f_x > 0.45 ) f_x = 0.45;
    if ( f_x < 0.05 ) f_x = 0.05;

    movo_msgs::LinearActuatorCmd msg;
    msg.desired_position_m = f_x;
    msg.fdfwd_vel_mps = 0;

    z_header.stamp = ros::Time::now();
    z_header.seq++;

    msg.header = z_header;

    if ( home_wait == 0)
    {
      pub_linear_cmd.publish(msg);
      //ROS_INFO_STREAM("Sending value to MOVO Z (m) " << " " << f_x  );
    }
}

void GroundstationRos::sendHead(double x,double y,double z)
{
  //Invalid head (don't control the head, other systems such as HMD will control the head)
  if ( x == -1000 && y == -1000 && z == -1000 ) return;

  movo_msgs::PanTiltCmd msg;

  movo_msgs::PVA pan_pva_msg;
  movo_msgs::PVA tilt_pva_msg;

  head_header.stamp = ros::Time::now();
  head_header.seq++;

  pan_pva_msg.pos_rad = 0.0174533 * x;
  pan_pva_msg.vel_rps = 2;
  pan_pva_msg.acc_rps2 = 0;

  tilt_pva_msg.pos_rad = 0.0174533 * y;
  tilt_pva_msg.vel_rps = 2;
  tilt_pva_msg.acc_rps2 = 0;

  msg.header = head_header;
  msg.pan_cmd = pan_pva_msg;
  msg.tilt_cmd = tilt_pva_msg;

  //--

  sensor_msgs::JointState msg_roll;

  msg_roll.name.push_back("joint-101");
  msg_roll.velocity.push_back(8);
  msg_roll.position.push_back(z * 0.0174533);
  msg_roll.effort.push_back(2);

  //ROS_INFO_STREAM("HEAD ROLL " << z * 0.0174533 << " " );
  
  if ( home_wait == 0)
  {
    pub_head_cmd.publish(msg);
    pub_head_roll_cmd.publish(msg_roll);
    //ROS_INFO_STREAM("Sending value to MOVO head " << " " << pan_pva_msg.pos_rad << " " << tilt_pva_msg.pos_rad );
  }
}

void GroundstationRos::callbackLaser(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_lock.lock();
    current_laser = *msg; 
    //ROS_INFO_STREAM("Laser ranges" << msg->ranges.size());
    laser_lock.unlock();
}

void GroundstationRos::compressAndSend(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat resized_mat;

    //ROS_WARN_STREAM("Camera size ratio " << camera_resize_ratio);
    cv::resize(cv_ptr->image, resized_mat, cv::Size(), camera_resize_ratio, camera_resize_ratio);

    std::vector<uchar> buff;//buffer for coding
    std::vector<int> param(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = jpeg_compression_ratio;//default(95) 0-100
    cv::imencode(".jpg", resized_mat, buff, param);

    if ( send_camera_to_station )
    {
      ground_station->sendCamera((char *)&buff[0],buff.size());
      //ROS_WARN_STREAM("Camera Packet size " << buff.size());
    }
}

void GroundstationRos::compressAndSend2(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat resized_mat;

    //ROS_WARN_STREAM("Camera size ratio " << camera_resize_ratio);
    cv::resize(cv_ptr->image, resized_mat, cv::Size(), 0.2, 0.2);

    cv::cvtColor(resized_mat, resized_mat, cv::COLOR_BGR2RGB);

	cv_bridge::CvImage out_msg;
	out_msg.encoding = sensor_msgs::image_encodings::RGB8;
	out_msg.image = resized_mat;
    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = msg->header.frame_id;
    pub_camera_color.publish(out_msg.toImageMsg());
}

void GroundstationRos::callbackCamera(const sensor_msgs::Image::ConstPtr &msg)
{
   //ROS_INFO("update");

   gMutex_.lock();

   if (msg->width != 0)
   {
      try
      {
	       if ( frame_skip == frame_skip_ratio )
	       {
	          frame_skip = 0;
 	          compressAndSend(msg);
 	          compressAndSend2(msg);
	       }
       
	       frame_skip++;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge camera exception: %s", e.what());
        return;
      }
    }
    else
    {
        ROS_ERROR("Camera frame ignored (Invalid width)");
    }

    gMutex_.unlock();
}

GroundstationRos::~GroundstationRos()
{
    if ( enable_log )
        wp->CloseFile();

    app_exit = true;
    thread_main.interrupt();
    thread_main.join();
}

}
