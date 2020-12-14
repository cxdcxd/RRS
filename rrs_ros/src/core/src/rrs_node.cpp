#include <ros/ros.h>

#include <sstream>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>

#include <dynamic_reconfigure/server.h>
#include "rrs_ros.hh"
#include "rrs_net_test.hh"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "rrs_ros");

  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle pnh = ros::NodeHandle("~");

  boost::shared_ptr<roboland::Net2TestROS> node(new roboland::Net2TestROS(nh,pnh,argc,argv));
  //boost::shared_ptr<roboland::RRSNetTest> node(new roboland::RRSNetTest(nh,pnh,argc,argv));

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
