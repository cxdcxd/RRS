// #include <nmpc_nlopt.h>
// #include <kinova_msgs/JointVelocity.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <movo_msgs/JacoAngularVelocityCmd7DOF.h>
static bool is_shut_down = true;

bool shut_down_cb(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)
{
	is_shut_down = true;
	return true;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "RealRobotVelocitySender");
	ros::NodeHandle n;
    ros::Rate loop_rate(100);
	ros::Rate loop_rate_sleep(1);
	is_shut_down = false;


	ros::ServiceServer shut_down_receiver =  n.advertiseService("/j2s7s300/shut_down",shut_down_cb);

	// NmpcNlopt nlmpc_controller(n);
	// ros::Publisher velocity_pub = n.advertise<kinova_msgs::JointVelocity>("/j2s7s300_driver/in/joint_velocity",10);
	ros::Publisher velocity_pub = n.advertise<movo_msgs::JacoAngularVelocityCmd7DOF>("/movo/left_arm/angular_vel_cmd",10);
	loop_rate_sleep.sleep();
	ros::spinOnce();
	// kinova_msgs::JointVelocity velocity_msg;
 //    velocity_msg.joint7 = 10;
	movo_msgs::JacoAngularVelocityCmd7DOF msg;
	msg.theta_shoulder_pan_joint = 0.0;
	msg.theta_shoulder_lift_joint = 0.0;
	msg.theta_arm_half_joint = 0.0;
	msg.theta_elbow_joint = 0.0;
	msg.theta_wrist_spherical_1_joint = 0.0;
	msg.theta_wrist_spherical_2_joint = 0.0;
	msg.theta_wrist_3_joint = 1;



  while(ros::ok())
	{
		ROS_INFO("msg sent out");
		velocity_pub.publish(msg);
		// if(is_shut_down)
		// 	break;
		ros::spinOnce();
		loop_rate.sleep();
	}


//  velocity_msg.joint7=0;
//  for(int i=0;i<10;i++)
//  {
//    ROS_INFO("zero msg sent out");
//    velocity_pub.publish(velocity_msg);
//    // if(is_shut_down)
//    // 	break;
//    loop_rate.sleep();
//    ros::spinOnce();
//  }


	ROS_INFO("System shutdown");
	ros::shutdown();
}
