#include "joint_control/joint_control_ros.hh"

namespace roboland
{

  int counter = 0;
  int plan_counter = 0;

  JointControlRos::JointControlRos(ros::NodeHandle &nh,
                                   ros::NodeHandle &pnh,
                                   int argc,
                                   char *argv[]) : set_stop(false),
                                                   thread_main(&JointControlRos::thrMain, this)
  {
    joint_control = new JointControl();

    mutex = false;
    app_exit = false;
    counter = 0;
    send_counter = 0;
    get_new = false;
    
    //Gripper Feedback
    //TODO

    //Gripper Control
    pub_right_gripper_cmd = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput_right", 1);
    pub_left_gripper_cmd = nh.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput_left", 1);

    //Voice
    pub_voice_cmd = nh.advertise<std_msgs::String>("/movo/voice/text", 1);

    //Arm Feedback 
    sub_right_arm_feedback = nh.subscribe("/movo/right_arm/joint_states", 1, &JointControlRos::callbackRightArm, this);
    sub_left_arm_feedback = nh.subscribe("/movo/left_arm/joint_states", 1, &JointControlRos::callbackLeftArm, this);
    
    //Arm Control
    sub_right_arm_haptic_control = nh.subscribe("/movo/right_arm/control/haptic", 1, &JointControlRos::callbackRightControlHaptic, this);
    sub_left_arm_haptic_control = nh.subscribe("/movo/left_arm/control/haptic", 1, &JointControlRos::callbackLeftControlHaptic, this);
  
    //Force Render
    pub_right_force_render = nh.advertise<joint_control::HapticRender>("movo/right_arm/force_render", 1);
    pub_left_force_render = nh.advertise<joint_control::HapticRender>("movo/left_arm/force_render", 1);
    
    //nmpc version
    pub_left_end_effector = nh.advertise<geometry_msgs::Pose>("left/nmpc_controller/in/goal", 1);
    pub_left_end_effector_stamp = nh.advertise<geometry_msgs::PoseStamped>("left/nmpc_controller/in/goal/stamp", 1);
    pub_right_end_effector = nh.advertise<geometry_msgs::Pose>("right/nmpc_controller/in/goal", 1);
    pub_right_end_effector_stamp = nh.advertise<geometry_msgs::PoseStamped>("right/nmpc_controller/in/goal/stamp", 1);

    sub_r_end_effector = nh.subscribe("/right/nmpc_controller/out/eef_pose", 1, &JointControlRos::callbackNmpcREefPose, this);
    sub_l_end_effector = nh.subscribe("/left/nmpc_controller/out/eef_pose", 1, &JointControlRos::callbackNmpcLEefPose, this);

    desire_right_haptic.gripper_angle = 0;
    desire_left_haptic.gripper_angle = 0;

    //Open the gripper
    ROS_INFO("LMT Shared Joint Control Ready Version: 29/12/21");
  }

  void JointControlRos::callbackNmpcREefPose(const geometry_msgs::Pose::ConstPtr &msg)
  {
    mtx_right_arm_nmpc.lock();

    nmpc_right_current.position.x = msg->position.x;
    nmpc_right_current.position.y = msg->position.y;
    nmpc_right_current.position.z = msg->position.z;

    nmpc_right_current.orientation.x = msg->orientation.x;
    nmpc_right_current.orientation.y = msg->orientation.y;
    nmpc_right_current.orientation.z = msg->orientation.z;
    nmpc_right_current.orientation.w = msg->orientation.w;

    mtx_right_arm_nmpc.unlock();
  }

  void JointControlRos::callbackNmpcLEefPose(const geometry_msgs::Pose::ConstPtr &msg)
  {
    mtx_left_arm_nmpc.lock();

    nmpc_left_current.position.x = msg->position.x;
    nmpc_left_current.position.y = msg->position.y;
    nmpc_left_current.position.z = msg->position.z;

    nmpc_left_current.orientation.x = msg->orientation.x;
    nmpc_left_current.orientation.y = msg->orientation.y;
    nmpc_left_current.orientation.z = msg->orientation.z;
    nmpc_left_current.orientation.w = msg->orientation.w;

    mtx_left_arm_nmpc.unlock();
  }

  void JointControlRos::thrMain()
  {
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    ROS_INFO("Control Thread Ready");

    float speed_gripper = 0;

    while (ros::ok() && app_exit == false)
    {
      if (home_wait > 0)
      {
        home_wait = home_wait - 1;
        ROS_WARN_STREAM("Waiting  " << home_wait);

        //1 hz
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
      }
      else
      {
        if (desire_left_haptic_updated)
        {
          desire_left_haptic_updated = false;
          float desire_gripper = 0;

          //Linear Velocity
          float speed_x = 0;
          float speed_y = 0;
          float speed_z = 0;

          //Angular Velocity
          float speed_rx = 0;
          float speed_ry = 0;
          float speed_rz = 0;

          //Position
          float pose_x = 0;
          float pose_y = 0;
          float pose_z = 0;

          //Rotation
          float rot_x = 0;
          float rot_y = 0;
          float rot_z = 0;

          float rot_q_x = 0;
          float rot_q_y = 0;
          float rot_q_z = 0;
          float rot_q_w = 0;

          float rot_matrix[3][3];

          mtx_left_desire_haptic.lock();

          pose_x = desire_left_haptic.position[0];
          pose_y = desire_left_haptic.position[1];
          pose_z = desire_left_haptic.position[2];

          rot_x = desire_left_haptic.rotation[0];
          rot_y = desire_left_haptic.rotation[1];
          rot_z = desire_left_haptic.rotation[2];

          speed_x = desire_left_haptic.linear_velocity[0];
          speed_y = desire_left_haptic.linear_velocity[1];
          speed_z = desire_left_haptic.linear_velocity[2];

          speed_rx = desire_left_haptic.angular_velocity[0];
          speed_ry = desire_left_haptic.angular_velocity[1];
          speed_rz = desire_left_haptic.angular_velocity[2];

          rot_matrix[0][0] = desire_left_haptic.rotation_matrix[0];
          rot_matrix[0][1] = desire_left_haptic.rotation_matrix[1];
          rot_matrix[0][2] = desire_left_haptic.rotation_matrix[2];

          rot_matrix[1][0] = desire_left_haptic.rotation_matrix[3];
          rot_matrix[1][1] = desire_left_haptic.rotation_matrix[4];
          rot_matrix[1][2] = desire_left_haptic.rotation_matrix[5];

          rot_matrix[2][0] = desire_left_haptic.rotation_matrix[6];
          rot_matrix[2][1] = desire_left_haptic.rotation_matrix[7];
          rot_matrix[2][2] = desire_left_haptic.rotation_matrix[8];

          // convert to pose message
          geometry_msgs::PoseStamped pose_msg;
          KDL::Rotation rot;
          for (int r = 0; r < 3; ++r)
          {
            for (int c = 0; c < 3; ++c)
            {
              rot(r, c) = rot_matrix[r][c];
            }
          }

          tf::poseKDLToMsg(KDL::Frame(rot, KDL::Vector(pose_x, pose_y, pose_z)),
                           pose_msg.pose);

          desire_gripper = desire_left_haptic.gripper_angle;

          mtx_left_desire_haptic.unlock();

          float current_gripper = 0;
          
          if (desire_left_haptic.cmd == "init")
          {
              init_offset_left = false;

              mtx_left_arm_nmpc.lock();

              nmpc_left_offset.position.x = nmpc_left_current.position.x;
              nmpc_left_offset.position.y = nmpc_left_current.position.y;
              nmpc_left_offset.position.z = nmpc_left_current.position.z;

              nmpc_left_offset.orientation.x = nmpc_left_current.orientation.x;
              nmpc_left_offset.orientation.y = nmpc_left_current.orientation.y;
              nmpc_left_offset.orientation.z = nmpc_left_current.orientation.z;
              nmpc_left_offset.orientation.w = nmpc_left_current.orientation.w;

              sigma_left_offset.position.x = pose_x;
              sigma_left_offset.position.y = pose_y;
              sigma_left_offset.position.z = pose_z;

              mtx_left_arm_nmpc.unlock();

              //sendLeftGripperPos(desire_gripper);
              sendLeftJacoArmNMPCQ(pose_x, pose_y, pose_z, desire_left_haptic.rotation_q[0], desire_left_haptic.rotation_q[1], desire_left_haptic.rotation_q[2], desire_left_haptic.rotation_q[3], desire_left_haptic.cmd);
          }
          else if (desire_left_haptic.cmd == "" || desire_right_haptic.cmd == "joy")
          {
            sendLeftGripperVel(desire_gripper);

            init_offset_left = false;

            //mtx_left_arm_nmpc.lock();

            //nmpc_left_offset.position.x = nmpc_left_current.position.x;
            //nmpc_left_offset.position.y = nmpc_left_current.position.y;
            //nmpc_left_offset.position.z = nmpc_left_current.position.z;

            //nmpc_left_offset.orientation.x = nmpc_left_current.orientation.x;
            //nmpc_left_offset.orientation.y = nmpc_left_current.orientation.y;
            //nmpc_left_offset.orientation.z = nmpc_left_current.orientation.z;
            //nmpc_left_offset.orientation.w = nmpc_left_current.orientation.w;

            //sigma_left_offset.position.x = pose_x;
            //sigma_left_offset.position.y = pose_y;
            //sigma_left_offset.position.z = pose_z;

            //mtx_left_arm_nmpc.unlock();


            //Convert Speed data to position
            tf::Quaternion q(nmpc_left_offset.orientation.x,nmpc_left_offset.orientation.y,nmpc_left_offset.orientation.z,nmpc_left_offset.orientation.w);
            tf::Matrix3x3 m(q);
              
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            float new_x = nmpc_left_offset.position.x - speed_y;
            float new_y = nmpc_left_offset.position.y - speed_z;
            float new_z = nmpc_left_offset.position.z + speed_x;

            ROS_INFO_STREAM("LS " << speed_x << " " << speed_y << " " << speed_z << " " << speed_rx << " "  << speed_ry << " "  << speed_rz);

            roll = roll + speed_rx;
            pitch = pitch + speed_ry;
            yaw = yaw + speed_rz;

            tf::Quaternion NewQuaternion;
            NewQuaternion.setRPY( roll, pitch, yaw );

            ROS_INFO_STREAM("LF " << new_x << " " << new_y << " " << new_z << " " << NewQuaternion.x() << " "  << NewQuaternion.y() << " "  << NewQuaternion.z() << " " << NewQuaternion.w() << " " << desire_left_haptic.cmd);
            
            if (speed_x !=0 || speed_y !=0 ||  speed_z !=0 ||  speed_rx !=0 ||  speed_ry !=0 ||  speed_rz !=0 )
            {
              sendLeftJacoArmNMPCQ(new_x, new_y, new_z, NewQuaternion.x(), NewQuaternion.y(), NewQuaternion.z(), NewQuaternion.w(), desire_left_haptic.cmd);

              nmpc_left_offset.position.x = new_x;
              nmpc_left_offset.position.y = new_y;
              nmpc_left_offset.position.z = new_z;

              nmpc_left_offset.orientation.x = NewQuaternion.x();
              nmpc_left_offset.orientation.y = NewQuaternion.y();
              nmpc_left_offset.orientation.z = NewQuaternion.z();
              nmpc_left_offset.orientation.w = NewQuaternion.w();
            }
            
          }
          else if (desire_left_haptic.cmd == "sigma7" || desire_left_haptic.cmd == "vive" || desire_left_haptic.cmd == "ml")
          {

            if (desire_left_haptic.cmd == "sigma7")
              ROS_INFO("Left SIGMA");
            else if (desire_left_haptic.cmd == "vive")
              ROS_INFO("Left vive");
              else if (desire_left_haptic.cmd == "ml")
              ROS_INFO("Left ml");

            //POSE AND ROTATION 6D

            float x = 0;
            float y = 0;
            float z = 0;
            float rx = 0;
            float ry = 0;
            float rz = 0;
            float rw = 0;
            float x_scale = 3;

            if (desire_left_haptic.cmd == "vive")
              x_scale = 1;

            mtx_left_arm_nmpc.lock();

            if (init_offset_left == false)
            {
              sigma_left_offset.position.y = pose_y;
              sigma_left_offset.position.x = pose_x;
              sigma_left_offset.position.z = pose_z;

              init_offset_left = true;
            }

            if (desire_left_haptic.cmd == "sigma7")
            {
              x = nmpc_left_offset.position.x - (sigma_left_offset.position.y - pose_y) * x_scale;
              y = nmpc_left_offset.position.y + (sigma_left_offset.position.z - pose_z) * x_scale;
              z = nmpc_left_offset.position.z + (sigma_left_offset.position.x - pose_x) * x_scale;

              rx = pose_msg.pose.orientation.x;
              ry = pose_msg.pose.orientation.y;
              rz = pose_msg.pose.orientation.z;
              rw = pose_msg.pose.orientation.w;
            }
            else if (desire_left_haptic.cmd == "vive")
            {
              x = nmpc_left_offset.position.x - (sigma_left_offset.position.x - pose_x) * x_scale;
              y = nmpc_left_offset.position.y + (sigma_left_offset.position.y - pose_y) * x_scale;
              z = nmpc_left_offset.position.z - (sigma_left_offset.position.z - pose_z) * x_scale;

              rx = desire_left_haptic.rotation_q[0];
              ry = desire_left_haptic.rotation_q[1];
              rz = desire_left_haptic.rotation_q[2];
              rw = desire_left_haptic.rotation_q[3];
            }
            else if (desire_left_haptic.cmd == "ml")
            {
              x = pose_x;
              y = pose_y;
              z = pose_z;

              rx = desire_left_haptic.rotation_q[0];
              ry = desire_left_haptic.rotation_q[1];
              rz = desire_left_haptic.rotation_q[2];
              rw = desire_left_haptic.rotation_q[3];
            }

            mtx_left_arm_nmpc.unlock();

            if (x == 0 && y == 0 && z == 0)
            {
              //skip
              ROS_ERROR("NMPC invalid position, skipping the goal for left arm");
            }
            else
              sendLeftJacoArmNMPCQ(x, y, z, rx, ry, rz, rw, desire_left_haptic.cmd);

            //Left GRIPPER
            if (desire_left_haptic.cmd == "sigma7")
            {
                int gripper_pos = desire_gripper * 10;
                if ( gripper_pos < 1) gripper_pos = 1;
                if ( gripper_pos > 250) gripper_pos = 250;
                
                gripper_pos = 251 - gripper_pos;

                sendLeftGripperPos(gripper_pos);

            }
            else if (desire_right_haptic.cmd == "vive")
            {
              sendLeftGripperVel(desire_gripper);
            }
          }
          else
          {
            //Joystick
            sendLeftGripperVel(desire_gripper);
          }
        }

        //============================================================================================
        //RIGHT MANIPULATOR LOGIC
        //============================================================================================

        if (desire_right_haptic_updated)
        {
          desire_right_haptic_updated = false;
          float desire_gripper = 0;

          //Linear Velocity
          float speed_x = 0;
          float speed_y = 0;
          float speed_z = 0;

          //Angular Velocity
          float speed_rx = 0;
          float speed_ry = 0;
          float speed_rz = 0;

          //Position
          float pose_x = 0;
          float pose_y = 0;
          float pose_z = 0;

          //Rotation
          float rot_x = 0;
          float rot_y = 0;
          float rot_z = 0;

          float rot_q_x = 0;
          float rot_q_y = 0;
          float rot_q_z = 0;
          float rot_q_w = 0;

          float rot_matrix[3][3];

          mtx_right_desire_haptic.lock();

          pose_x = desire_right_haptic.position[0];
          pose_y = desire_right_haptic.position[1];
          pose_z = desire_right_haptic.position[2];

          rot_x = desire_right_haptic.rotation[0];
          rot_y = desire_right_haptic.rotation[1];
          rot_z = desire_right_haptic.rotation[2];

          speed_x = desire_right_haptic.linear_velocity[0];
          speed_y = desire_right_haptic.linear_velocity[1];
          speed_z = desire_right_haptic.linear_velocity[2];

          speed_rx = desire_right_haptic.angular_velocity[0];
          speed_ry = desire_right_haptic.angular_velocity[1];
          speed_rz = desire_right_haptic.angular_velocity[2];

          rot_matrix[0][0] = desire_right_haptic.rotation_matrix[0];
          rot_matrix[0][1] = desire_right_haptic.rotation_matrix[1];
          rot_matrix[0][2] = desire_right_haptic.rotation_matrix[2];

          rot_matrix[1][0] = desire_right_haptic.rotation_matrix[3];
          rot_matrix[1][1] = desire_right_haptic.rotation_matrix[4];
          rot_matrix[1][2] = desire_right_haptic.rotation_matrix[5];

          rot_matrix[2][0] = desire_right_haptic.rotation_matrix[6];
          rot_matrix[2][1] = desire_right_haptic.rotation_matrix[7];
          rot_matrix[2][2] = desire_right_haptic.rotation_matrix[8];

          // convert to pose message
          geometry_msgs::PoseStamped pose_msg;
          KDL::Rotation rot;
          for (int r = 0; r < 3; ++r)
          {
            for (int c = 0; c < 3; ++c)
            {
              rot(r, c) = rot_matrix[r][c];
            }
          }

          tf::poseKDLToMsg(KDL::Frame(rot, KDL::Vector(pose_x, pose_y, pose_z)),
                           pose_msg.pose);

          desire_gripper = desire_right_haptic.gripper_angle;

          mtx_right_desire_haptic.unlock();

          float current_gripper = 0;

          if (desire_right_haptic.cmd == "init")
          {
              init_offset_right = false;

              mtx_right_arm_nmpc.lock();

              nmpc_right_offset.position.x = nmpc_right_current.position.x;
              nmpc_right_offset.position.y = nmpc_right_current.position.y;
              nmpc_right_offset.position.z = nmpc_right_current.position.z;

              nmpc_right_offset.orientation.x = nmpc_right_current.orientation.x;
              nmpc_right_offset.orientation.y = nmpc_right_current.orientation.y;
              nmpc_right_offset.orientation.z = nmpc_right_current.orientation.z;
              nmpc_right_offset.orientation.w = nmpc_right_current.orientation.w;

              sigma_right_offset.position.x = pose_x;
              sigma_right_offset.position.y = pose_y;
              sigma_right_offset.position.z = pose_z;

              mtx_right_arm_nmpc.unlock();

              //sendRightGripperPos(desire_gripper);
              sendRightJacoArmNMPCQ(pose_x, pose_y, pose_z, desire_right_haptic.rotation_q[0], desire_right_haptic.rotation_q[1], desire_right_haptic.rotation_q[2], desire_right_haptic.rotation_q[3], desire_right_haptic.cmd);
          }
          else
          if (desire_right_haptic.cmd == "" || desire_right_haptic.cmd == "joy")
          {
            //ROS_INFO_STREAM("Right Gripper " << desire_gripper);
            sendRightGripperVel(desire_gripper);
           
            init_offset_right = false;

            mtx_right_arm_nmpc.lock();

            nmpc_right_offset.position.x = nmpc_right_current.position.x;
            nmpc_right_offset.position.y = nmpc_right_current.position.y;
            nmpc_right_offset.position.z = nmpc_right_current.position.z;

            nmpc_right_offset.orientation.x = nmpc_right_current.orientation.x;
            nmpc_right_offset.orientation.y = nmpc_right_current.orientation.y;
            nmpc_right_offset.orientation.z = nmpc_right_current.orientation.z;
            nmpc_right_offset.orientation.w = nmpc_right_current.orientation.w;

            sigma_right_offset.position.x = pose_x;
            sigma_right_offset.position.y = pose_y;
            sigma_right_offset.position.z = pose_z;

            mtx_right_arm_nmpc.unlock();
            
            //Convert Speed data to position
            tf::Quaternion q(nmpc_right_offset.orientation.x,nmpc_right_offset.orientation.y,nmpc_right_offset.orientation.z,nmpc_right_offset.orientation.w);
            tf::Matrix3x3 m(q);
              
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            float new_x = nmpc_right_offset.position.x - speed_y;
            float new_y = nmpc_right_offset.position.y - speed_z;
            float new_z = nmpc_right_offset.position.z + speed_x;

            ROS_INFO_STREAM("RS " << speed_x << " " << speed_y << " " << speed_z << " " << speed_rx << " "  << speed_ry << " "  << speed_rz);

            roll = roll + speed_rx;
            pitch = pitch + speed_ry;
            yaw = yaw + speed_rz;

            tf::Quaternion NewQuaternion;
            NewQuaternion.setRPY( roll, pitch, yaw );

            ROS_INFO_STREAM("RF " << new_x << " " << new_y << " " << new_z << " " << NewQuaternion.x() << " "  << NewQuaternion.y() << " "  << NewQuaternion.z() << " " << NewQuaternion.w() << " " << desire_right_haptic.cmd);
            
            if (speed_x !=0 || speed_y !=0 ||  speed_z !=0 ||  speed_rx !=0 ||  speed_ry !=0 ||  speed_rz !=0 )
            {
            sendRightJacoArmNMPCQ(new_x, new_y, new_z, NewQuaternion.x(), NewQuaternion.y(), NewQuaternion.z(), NewQuaternion.w(), desire_right_haptic.cmd);

            nmpc_right_offset.position.x = new_x;
            nmpc_right_offset.position.y = new_y;
            nmpc_right_offset.position.z = new_z;

            nmpc_right_offset.orientation.x = NewQuaternion.x();
            nmpc_right_offset.orientation.y = NewQuaternion.y();
            nmpc_right_offset.orientation.z = NewQuaternion.z();
            nmpc_right_offset.orientation.w = NewQuaternion.w();
            }


          }
          else if (desire_right_haptic.cmd == "sigma7" || desire_right_haptic.cmd == "vive" || desire_right_haptic.cmd == "ml")
          {
            if (desire_right_haptic.cmd == "sigma7")
              ROS_INFO("Right SIGMA");
            else if (desire_right_haptic.cmd == "vive")
              ROS_INFO("Right vive");
            else if (desire_right_haptic.cmd == "ml")
              ROS_INFO("Right ml");

            //POSE AND ROTATION 6D
            float x = 0;
            float y = 0;
            float z = 0;
            float rx = 0;
            float ry = 0;
            float rz = 0;
            float rw = 0;
            float x_scale = 3;

            if (desire_right_haptic.cmd == "vive")
              x_scale = 1;

            mtx_right_arm_nmpc.lock();

            if (init_offset_right == false)
            {
              sigma_right_offset.position.y = pose_y;
              sigma_right_offset.position.x = pose_x;
              sigma_right_offset.position.z = pose_z;

              init_offset_right = true;
            }

            if (desire_right_haptic.cmd == "sigma7")
            {
              x = nmpc_right_offset.position.x - (sigma_right_offset.position.y - pose_y) * x_scale;
              y = nmpc_right_offset.position.y + (sigma_right_offset.position.z - pose_z) * x_scale;
              z = nmpc_right_offset.position.z + (sigma_right_offset.position.x - pose_x) * x_scale;

              rx = pose_msg.pose.orientation.x;
              ry = pose_msg.pose.orientation.y;
              rz = pose_msg.pose.orientation.z;
              rw = pose_msg.pose.orientation.w;
            }
            else if (desire_right_haptic.cmd == "ml")
            {
              x = pose_x;
              y = pose_y;
              z = pose_z;

              rx = desire_right_haptic.rotation_q[0];
              ry = desire_right_haptic.rotation_q[1];
              rz = desire_right_haptic.rotation_q[2];
              rw = desire_right_haptic.rotation_q[3];
            }

            mtx_right_arm_nmpc.unlock();

            if (x == 0 && y == 0 && z == 0)
            {
              ROS_ERROR("NMPC invalid position, skipping the goal for right arm");
            }
            else
              sendRightJacoArmNMPCQ(x, y, z, rx, ry, rz, rw, desire_right_haptic.cmd);

            //Right Gripper
            if (desire_right_haptic.cmd == "sigma7")
            {
                int gripper_pos = desire_gripper * 10;
                if ( gripper_pos < 1) gripper_pos = 1;
                if ( gripper_pos > 250) gripper_pos = 250;
                
                gripper_pos = 251 - gripper_pos;

                sendRightGripperPos(gripper_pos);

            }
            else if (desire_right_haptic.cmd == "vive")
            {
              sendRightGripperVel(desire_gripper);
            }
          }
          else
          {
            //Joystick
            sendRightGripperVel(desire_gripper);
          }

        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
      }
    }
  }

  void JointControlRos::callbackRightArmPose(const kinova_msgs::KinovaPose::ConstPtr &msg)
  {
    mtx_right_arm_pose.lock();
    current_right_arm_pose = *msg;
    mtx_right_arm_pose.unlock();
  }

  bool JointControlRos::callbackArmHome(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    sendHomeArms();
    return true;
  }

  void JointControlRos::callbackRightArm(const sensor_msgs::JointState::ConstPtr &msg)
  {
    mtx_right_arm.lock();
    current_right_arm_state = *msg;
    mtx_right_arm.unlock();
  }

  void JointControlRos::callbackLeftArm(const sensor_msgs::JointState::ConstPtr &msg)
  {
    mtx_left_arm.lock();
    current_left_arm_state = *msg;
    mtx_left_arm.unlock();
  }

  void JointControlRos::callbackRightGripper(const sensor_msgs::JointState::ConstPtr &msg)
  {
    mtx_right_gripper.lock();
    //current_right_gripper_state = *msg;
    mtx_right_gripper.unlock();
  }

  void JointControlRos::callbackLeftGripper(const sensor_msgs::JointState::ConstPtr &msg)
  {
    mtx_left_gripper.lock();
    //current_left_gripper_state = *msg;
    mtx_left_gripper.unlock();
  }

  void JointControlRos::callbackRightControlHaptic(const joint_control::HapticCommand::ConstPtr &msg)
  {
    mtx_right_desire_haptic.lock();
    desire_right_haptic = *msg;
    desire_right_haptic_updated = true;
    mtx_right_desire_haptic.unlock();
  }

  void JointControlRos::callbackLeftControlHaptic(const joint_control::HapticCommand::ConstPtr &msg)
  {
    mtx_left_desire_haptic.lock();
    desire_left_haptic = *msg;
    desire_left_haptic_updated = true;
    mtx_left_desire_haptic.unlock();
  }

  void JointControlRos::playVoice(std::string text)
  {
    ROS_WARN_STREAM("Play voice : " << text);
    std_msgs::String msg;
    msg.data = text;
    pub_voice_cmd.publish(msg);
  }

  void JointControlRos::sendHomeArms()
  {
    if (home_wait == 0)
    {
      ROS_ERROR("Send home for arms");
      playVoice("Moving my arms to the home position, please wait 15 seconds");

      home_wait = 15;

      std_msgs::Bool msg;
      msg.data = true;
      //pub_home_arms.publish(msg); //We dont have this in NMPC version
    }
  }

  void JointControlRos::sendRightJacoArmNMPCQ(double x, double y, double z, double rx, double ry, double rz, double rw, string mode)
  {
    if ( x == 0 && y == 0 & z == 0 ) return;
    if ( isnan(rx) || isnan(ry) || isnan(rz)  || isnan(rw) ) return;

    geometry_msgs::Pose msg;

    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;

    if (mode == "sigma7")
    {
      msg.orientation.x = ry;
      msg.orientation.y = -1 * rz;
      msg.orientation.z = -1 * rx;
      msg.orientation.w = rw;

      //msg.orientation.x = rx;
      //msg.orientation.y = ry;
      //msg.orientation.z = rz;
      //msg.orientation.w = rw;

    }
    else if (mode == "vive")
    {
      msg.orientation.x = -1 * rx;
      msg.orientation.y = ry;
      msg.orientation.z = -1 * rz;
      msg.orientation.w = rw;
    }
    else
    {
      msg.orientation.x = rx;
      msg.orientation.y = ry;
      msg.orientation.z = rz;
      msg.orientation.w = rw;
    }
    
    //ROS_INFO_STREAM("NMPC DR qx: " << msg.orientation.x << " qy: " << msg.orientation.y << " qz: " << msg.orientation.z << " qw: " << msg.orientation.w);
    //ROS_INFO_STREAM("NMPC DP x: " << msg.position.x << " y: " << msg.position.y << " z: " << msg.position.z );
    //ROS_INFO_STREAM("NMC cx: " << nmpc_left_current.position.x << " cy: " << nmpc_left_current.position.y << " cz: " << nmpc_left_current.position.z );

    if (nmpc_active)
      pub_right_end_effector.publish(msg);

    geometry_msgs::PoseStamped msgs;

    msgs.pose.position.x = x;
    msgs.pose.position.y = y;
    msgs.pose.position.z = z;

    msgs.pose.orientation.x = msg.orientation.x;
    msgs.pose.orientation.y = msg.orientation.y;
    msgs.pose.orientation.z = msg.orientation.z;
    msgs.pose.orientation.w = msg.orientation.w;

    msgs.header.frame_id = "base_link";

    pub_right_end_effector_stamp.publish(msgs);
  }

  void JointControlRos::sendLeftJacoArmNMPCQ(double x, double y, double z, double rx, double ry, double rz, double rw, string mode)
  {
    if ( x == 0 && y == 0 & z == 0 ) return;
    if ( isnan(rx) || isnan(ry) || isnan(rz)  || isnan(rw) ) return;

    geometry_msgs::Pose msg;

    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;

    if (mode == "sigma7")
    {
      msg.orientation.x = ry;
      msg.orientation.y = -1 * rz;
      msg.orientation.z = -1 * rx;
      msg.orientation.w = rw;
     
      //msg.orientation.x = rx;
      //msg.orientation.y = ry;
      //msg.orientation.z = rz;
      //msg.orientation.w = rw;
    }
    else if (mode == "vive")
    {
      msg.orientation.x = -1 * rx;
      msg.orientation.y = ry;
      msg.orientation.z = -1 * rz;
      msg.orientation.w = rw;
    }
    else
    {
      msg.orientation.x = rx;
      msg.orientation.y = ry;
      msg.orientation.z = rz;
      msg.orientation.w = rw;
    }

    //ROS_INFO_STREAM("NMC  L qx: " << msg.orientation.x << " qy: " << msg.orientation.y << " qz: " << msg.orientation.z << " qw: " << msg.orientation.w);
    //ROS_INFO_STREAM("NMPC L x: " << msg.position.x << " y: " << msg.position.y << " z: " << msg.position.z );
    //ROS_INFO_STREAM("NMC cx: " << nmpc_left_current.position.x << " cy: " << nmpc_left_current.position.y << " cz: " << nmpc_left_current.position.z );

    if (nmpc_active)
      pub_left_end_effector.publish(msg);

    geometry_msgs::PoseStamped msgs;

    msgs.pose.position.x = x;
    msgs.pose.position.y = y;
    msgs.pose.position.z = z;

    msgs.pose.orientation.x = msg.orientation.x;
    msgs.pose.orientation.y = msg.orientation.y;
    msgs.pose.orientation.z = msg.orientation.z;
    msgs.pose.orientation.w = msg.orientation.w;

    msgs.header.frame_id = "base_link";

    pub_left_end_effector_stamp.publish(msgs);
  }

  void JointControlRos::sendRightGripperPos(int x)
  {
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output g_msg;

    if ( x < 0 ) x = 0;
    if ( x > 255) x = 255;

    g_msg.rPR = x;
    g_msg.rACT = 1;
    g_msg.rGTO = 1;
    g_msg.rSP  = 100;
    g_msg.rFR  = 150;

    right_current_gripper_pos = x;

    if (home_wait == 0)
    {
      pub_right_gripper_cmd.publish(g_msg);
    }
  }

  void JointControlRos::sendRightGripperVel(int x)
  {
     robotiq_2f_gripper_control::Robotiq2FGripper_robot_output g_msg;

    if ( x > 0 ) right_current_gripper_pos++;
    if ( x < 0 ) right_current_gripper_pos--;

    if ( right_current_gripper_pos < 0 ) right_current_gripper_pos = 0;
    if ( right_current_gripper_pos > 255) right_current_gripper_pos = 255;

    //ROS_INFO_STREAM("Current pos " << current_gripper_pos);

    g_msg.rPR = right_current_gripper_pos;
    g_msg.rACT = 1;
    g_msg.rGTO = 1;
    g_msg.rSP  = 1;
    g_msg.rFR  = 1;

    if (home_wait == 0)
    {
      ROS_INFO_STREAM("publish right " << right_current_gripper_pos);
      pub_right_gripper_cmd.publish(g_msg);
    }
  }

  void JointControlRos::sendLeftGripperPos(int x)
  {
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output g_msg;

    if ( x < 0 ) x = 0;
    if ( x > 255) x = 255;

    g_msg.rPR = x;
    g_msg.rACT = 1;
    g_msg.rGTO = 1;
    g_msg.rSP  = 100;
    g_msg.rFR  = 150;

    left_current_gripper_pos = x;

    if (home_wait == 0)
    {
      pub_left_gripper_cmd.publish(g_msg);
    }
  }

  void JointControlRos::sendLeftGripperVel(int x)
  {
    robotiq_2f_gripper_control::Robotiq2FGripper_robot_output g_msg;

    if ( x > 0 ) left_current_gripper_pos++;
    if ( x < 0 ) left_current_gripper_pos--;

    if ( left_current_gripper_pos < 0 ) left_current_gripper_pos = 0;
    if ( left_current_gripper_pos > 255) left_current_gripper_pos = 255;

    //ROS_INFO_STREAM("Current pos " << current_gripper_pos);

    g_msg.rPR = left_current_gripper_pos;
    g_msg.rACT = 1;
    g_msg.rGTO = 1;
    g_msg.rSP  = 1;
    g_msg.rFR  = 1;

    if (home_wait == 0)
    {
      ROS_INFO_STREAM("publish left " << left_current_gripper_pos);
      pub_left_gripper_cmd.publish(g_msg);
    }
  }

  JointControlRos::~JointControlRos()
  {
    app_exit = true;

    thread_main.interrupt();
    thread_main.join();
  }

} // namespace roboland
