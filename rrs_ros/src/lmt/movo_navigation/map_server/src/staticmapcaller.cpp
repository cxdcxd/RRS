
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

ros::ServiceClient client_makeplan;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "staticmapcaller");

    ROS_INFO("staticmapcaller started");

    ros::NodeHandle node_handles;
    client_makeplan = node_handles.serviceClient<std_srvs::Empty>("sepantamapengenine/load");

    std_srvs::Empty _srv;
    client_makeplan.call(_srv);
 
    ros::spinOnce();

    
    return 0;
}