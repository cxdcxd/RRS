#ifndef _NET2_ROS_HH_
#define _NET2_ROS_HH_

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/subscriber.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <sstream>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>

#include <dynamic_reconfigure/server.h>
#include <std_srvs/Empty.h>
#include <mutex>

//Net2
#include "Net2/Net2.h"
#include "Net2/Net2Base.h"

//Objects
#include "Net2/Net2Publisher.h"
#include "Net2/Net2Subscirber.h"
#include "Net2/Net2Service.h"
#include "Net2/Net2Client.h"
#include "Net2/Net2AsyncClient.h"

#include <fstream>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Roboland::Tools::Network;

namespace roboland
{

struct settings
{
  std::string consul_network_address = "10.147.20.149";
  std::string consul_network_mask = "255.255.255.0";
  std::string consul_network_port = "8500";
  std::string ntp_server_host_name = "test";
};

class Net2ROS 
{
public:
  Net2ROS(ros::NodeHandle &nh,
              ros::NodeHandle &pnh,
              int argc,
              char *argv[]);

  bool is_file_exist(const char *fileName);
  bool loadYaml();
  bool saveYaml();

  std::string config_path = "";
  std::string old_cmd = "";
  YAML::Node m_config;
  settings m_settings;

  ~Net2ROS();
  void kill();
  void update();
  Net2 *net2;
};

} // namespace roboland

#endif /* _NET2_ROS_HH_ */
