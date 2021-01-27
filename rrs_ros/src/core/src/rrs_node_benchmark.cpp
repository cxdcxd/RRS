#include <sstream>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "rrs_ros_benchmark.hh"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rrs_benchmark");

  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle pnh = ros::NodeHandle("~");

  boost::shared_ptr<lmt::BenchmarkROS> node(new lmt::BenchmarkROS(nh,pnh,argc,argv));

  ros::Rate loop(100); //100 Hz

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
