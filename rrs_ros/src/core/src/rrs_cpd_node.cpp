#include <ros/ros.h>

#include <sstream>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>

#include <dynamic_reconfigure/server.h>
#include "rrs_ros_cpd.hh"
#include "rrs_net_test.hh"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rrs_cpd_ros");

  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle pnh = ros::NodeHandle("~");

  boost::shared_ptr<lmt::Net2TestROSCPD> node(new lmt::Net2TestROSCPD(nh,pnh,argc,argv));

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
