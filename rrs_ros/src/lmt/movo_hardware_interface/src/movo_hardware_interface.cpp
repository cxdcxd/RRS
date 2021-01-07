
#include <movo_hardware_interface/movo_hardware_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>


using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

namespace movo_hardware_interface
{
	movoHardwareInterface::movoHardwareInterface(ros::NodeHandle& nh) \
		: nh_(nh)
	{
		init();
		controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

		nh_.param("/movo/hardware_interface/loop_hz", loop_hz_, 0.1);
		ROS_DEBUG_STREAM_NAMED("constructor","Using loop freqency of " << loop_hz_ << " hz");
		ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
		non_realtime_loop_ = nh_.createTimer(update_freq, &movoHardwareInterface::update, this);

		ROS_INFO_NAMED("hardware_interface", "Loaded movo_hardware_interface.");

		pub_joint_command = nh.advertise<movo_msgs::JacoJointCmd>("rrs/joint_command",1);
		sub_joint_state = nh.subscribe("rrs/joint_states",1,&movoHardwareInterface::chatterCallbackJointState, this);
	}

	movoHardwareInterface::~movoHardwareInterface()
	{

	}

	void movoHardwareInterface::chatterCallbackJointState (const sensor_msgs::JointState::ConstPtr& msg)
	{
        mtx_status.lock();
        current_joint_state = *msg;
        mtx_status.unlock();
	}

	void movoHardwareInterface::init()
	{
		nh_.getParam("/movo/hardware_interface/joints", joint_names_);

		if (joint_names_.size() != 19)
		{
		  ROS_FATAL_STREAM_NAMED("init","No joints found on parameter server for controller. Did you load the proper yaml file?");
		}

		num_joints_ = joint_names_.size();

		ROS_INFO_STREAM("Found " << num_joints_ << " Joints");

		// Resize vectors
		joint_position_.resize(num_joints_);
		joint_velocity_.resize(num_joints_);
		joint_effort_.resize(num_joints_);
		joint_position_command_.resize(num_joints_);
		joint_velocity_command_.resize(num_joints_);
		joint_effort_command_.resize(num_joints_);

		// Initialize controller
		for (int i = 0; i < num_joints_; i++)
		{
			ROS_INFO_STREAM("Joint Name " << joint_names_[i]);
		
		    // Create joint state interface
		 	JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
		    joint_state_interface_.registerHandle(jointStateHandle);

		    // Create position joint interface
		 	JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
			position_joint_interface_.registerHandle(jointPositionHandle);
		}

		ROS_INFO("Registering interfaces... ");
		
		registerInterface(&position_joint_interface_);
		registerInterface(&joint_state_interface_);
		
		ROS_INFO_STREAM("Init Done");
	}

	void movoHardwareInterface::update(const ros::TimerEvent& e)
	{
		elapsed_time_ = ros::Duration(e.current_real - e.last_real);

		read();
		controller_manager_->update(ros::Time::now(), elapsed_time_);
		write(elapsed_time_);
	}

	void movoHardwareInterface::read()
	{
        mtx_status.lock();

        if ( current_joint_state.position.size() != 0)
		{
			for (int i = 0; i < num_joints_; i++)
			{
				
			joint_position_[i] = current_joint_state.position[i];
			joint_velocity_[i] = current_joint_state.velocity[i];
			joint_effort_[i] = current_joint_state.effort[i];

			}
		}

        mtx_status.unlock();
	}

	void movoHardwareInterface::write(ros::Duration elapsed_time)
	{
        movo_msgs::JacoJointCmd cmd;

		for (int i = 0; i < num_joints_; i++)
		{
		   cmd.joint_cmds.push_back( joint_position_command_[i]);
		}

	    pub_joint_command.publish(cmd);
	}
}

