
#include <ros/package.h>
#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"
#include "map_server/image_loader.h"
#include "nav_msgs/MapMetaData.h"
#include "yaml-cpp/yaml.h"
#include "std_srvs/Empty.h"

#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"
#include "std_srvs/Empty.h"

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

ros::Publisher pub_slam_reset;


void reset_hector_slam()
{
	std_msgs::String _msg;
	_msg.data = "reset";
	pub_slam_reset.publish(_msg);
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "hectorreseter");

   ROS_INFO("hectorreseter started");


   ros::NodeHandle node_handles;
   pub_slam_reset = node_handles.advertise<std_msgs::String>("syscommand", 1);

 
   ros::Rate loop_rate(2);
   while (ros::ok() )
    {
        reset_hector_slam();
        ros::spinOnce();
        loop_rate.sleep();
        std::cout<<"Send Reset"<<std::endl;
    }

    
    return 0;
}