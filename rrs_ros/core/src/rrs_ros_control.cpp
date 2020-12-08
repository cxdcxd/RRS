#include "rrs_ros.hh"

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include "movo_hardware.hh"

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot() 
 { 
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_1("right_shoulder_pan_joint", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_1);

   hardware_interface::JointStateHandle state_handle_2("right_shoulder_lift_joint", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_2);

   hardware_interface::JointStateHandle state_handle_3("right_arm_half_joint", &pos[2], &vel[2], &eff[2]);
   jnt_state_interface.registerHandle(state_handle_3);

   hardware_interface::JointStateHandle state_handle_4("right_elbow_joint", &pos[3], &vel[3], &eff[3]);
   jnt_state_interface.registerHandle(state_handle_4);

   hardware_interface::JointStateHandle state_handle_5("right_wrist_spherical_1_joint", &pos[4], &vel[4], &eff[4]);
   jnt_state_interface.registerHandle(state_handle_5);

   hardware_interface::JointStateHandle state_handle_6("right_wrist_spherical_2_joint", &pos[5], &vel[5], &eff[5]);
   jnt_state_interface.registerHandle(state_handle_6);

   hardware_interface::JointStateHandle state_handle_7("right_wrist_3_joint", &pos[6], &vel[6], &eff[6]);
   jnt_state_interface.registerHandle(state_handle_7);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_1(jnt_state_interface.getHandle("right_shoulder_pan_joint"), &cmd[0]);
   jnt_pos_interface.registerHandle(pos_handle_1);

   hardware_interface::JointHandle pos_handle_2(jnt_state_interface.getHandle("right_shoulder_lift_joint"), &cmd[1]);
   jnt_pos_interface.registerHandle(pos_handle_2);

   hardware_interface::JointHandle pos_handle_3(jnt_state_interface.getHandle("right_arm_half_joint"), &cmd[2]);
   jnt_pos_interface.registerHandle(pos_handle_3);

   hardware_interface::JointHandle pos_handle_4(jnt_state_interface.getHandle("right_elbow_joint"), &cmd[3]);
   jnt_pos_interface.registerHandle(pos_handle_4);

   hardware_interface::JointHandle pos_handle_5(jnt_state_interface.getHandle("right_wrist_spherical_1_joint"), &cmd[4]);
   jnt_pos_interface.registerHandle(pos_handle_5);

   hardware_interface::JointHandle pos_handle_6(jnt_state_interface.getHandle("right_wrist_spherical_2_joint"), &cmd[5]);
   jnt_pos_interface.registerHandle(pos_handle_6);

   hardware_interface::JointHandle pos_handle_7(jnt_state_interface.getHandle("right_wrist_3_joint"), &cmd[6]);
   jnt_pos_interface.registerHandle(pos_handle_7);

   //sub_joint_state = nh.subscriber("jointstate",1,);
   //pub_joint_command = nh.subscribe("joint_command",1, &Net2TestROS::chatterCallbackJointCommand, this);

   registerInterface(&jnt_pos_interface);
  }

void read()
{

}

void write()
{

}

void update()
{
  
}

void get_time()
{

}

void init()
{
  // MyRobot movo;
  // controller_manager::ControllerManager cm(&movo);

  // ros::Rate loop(10);

  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   loop.sleep();

  //   movo.read();
  //   //cm.update(movo.get_time(), movo.get_period());
  //   movo.write();
    
  // }

}

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[7];
  double pos[7];
  double vel[7];
  double eff[7];
};


