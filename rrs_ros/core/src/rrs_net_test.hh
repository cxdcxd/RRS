#ifndef _RRS_NET_TEST_HH_
#define _RRS_NET_TEST_HH_

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
#include <memory>
#include <fstream>

#include <std_srvs/Empty.h>
#include <mutex>

//Net2
#include <Net2/Net2.h>
#include <Net2/Net2Base.h>

//Objects
#include <Net2/Net2Publisher.h>
#include <Net2/Net2Subscirber.h>
#include <Net2/Net2Service.h>
#include <Net2/Net2Client.h>
#include <Net2/Net2AsyncClient.h>

#include <yaml-cpp/yaml.h>
#include <Scene.pb.h>

using namespace std;
using namespace Roboland::Tools::Network;

namespace roboland
{

struct settingstest
{
  std::string consul_network_address = "10.147.20.149";
  std::string local_network_address = "10.147.20.149";
  std::string consul_network_mask = "255.255.255.0";
  std::string consul_network_port = "8500";
  std::string ntp_server_host_name = "test";
};

class RRSNetTest 
{
public:
  RRSNetTest(ros::NodeHandle &nh,
              ros::NodeHandle &pnh,
              int argc,
              char *argv[]);

  bool is_file_exist(const char *fileName);
  bool loadYaml();
  bool saveYaml();

  static const int pub_count = 1;
  static const int sub_count = 5;

  std::shared_ptr<Net2Publisher> publishers[pub_count];
  std::shared_ptr<Net2Subscriber> subscribers[sub_count];


  void send(int index);

  std::string config_path = "";
  YAML::Node m_config;
  settingstest m_settings;

  void update();
  ~RRSNetTest();
  void kill();
  Net2 *net2;

  std::vector<char> callbackDataSub1(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataSub2(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataSub3(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataSub4(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataSub5(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataSub6(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataSub7(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataSub8(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataSub9(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackDataSub10(std::vector<char> buffer, unsigned int priority, std::string sender);

  
};

} // namespace RRSNetTest

#endif /* _RRS_NET_TEST_HH_ */
