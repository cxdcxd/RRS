#ifndef ROS_CONTROL__movo_HARDWARE_INTERFACE_H
#define ROS_CONTROL__movo_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
// #include <movocpp/movo.h>
// #include <movocpp/arm.h>
// #include <movocpp/joint.h>
#include <movo_hardware_interface/movo_hardware.h>
#include "sensor_msgs/JointState.h"
#include "movo_msgs/JacoJointCmd.h"
#include <sstream>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;


namespace movo_hardware_interface
{
	static const double POSITION_STEP_FACTOR = 10;
	static const double VELOCITY_STEP_FACTOR = 10;

	class movoHardwareInterface: public movo_hardware_interface::movoHardware
	{
		public:
			movoHardwareInterface(ros::NodeHandle& nh);
			~movoHardwareInterface();
			void init();
			void update(const ros::TimerEvent& e);
			void read();
			void write(ros::Duration elapsed_time);
			void chatterCallbackJointState (const sensor_msgs::JointState::ConstPtr& msg);
			sensor_msgs::JointState current_joint_state;
			std::mutex mtx_status;

		protected:
			ros::NodeHandle nh_;
			ros::Timer non_realtime_loop_;
			ros::Duration control_period_;
			ros::Duration elapsed_time_;
			JointStateInterface jointStateInterface;
			PositionJointInterface positionJointInterface;
			PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
			double loop_hz_;
			boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
			double p_error_, v_error_, e_error_;
			std::string _logInfo;
			ros::Subscriber sub_joint_state;
			ros::Publisher pub_joint_command;

	};

}

#endif
