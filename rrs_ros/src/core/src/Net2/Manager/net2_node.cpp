#include <ros/ros.h>

#include <sstream>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>

#include "net2_ros.hh"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "net2_node");

  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle pnh = ros::NodeHandle("~");

  //ROS_INFO("Creating ros node");
  boost::shared_ptr<roboland::Net2ROS> node(new roboland::Net2ROS(nh,pnh,argc,argv));

  ros::Rate loop(1); //1 hz

  //ROS_INFO("WHILE");

  while (ros::ok())
  {
    //ROS_INFO("ROS LOOP");
    ros::spinOnce();
    loop.sleep();
    node->update();
  }

  //ROS_INFO("Exiting...");

  node->kill();

  return 0;
}
