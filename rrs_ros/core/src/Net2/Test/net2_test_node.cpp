#include <ros/ros.h>

#include <sstream>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>

#include <dynamic_reconfigure/server.h>
#include "net2_test_ros.hh"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "net2_test_node");

  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle pnh = ros::NodeHandle("~");

  boost::shared_ptr<roboland::Net2TestROS> node(new roboland::Net2TestROS(nh,pnh,argc,argv));

  ros::Rate loop(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop.sleep();
    node->update();
  }

  ROS_INFO("Exiting...");

  node->kill();

  return 0;
}
