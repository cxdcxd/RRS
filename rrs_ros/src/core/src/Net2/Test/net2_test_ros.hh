#ifndef _NET2_TEST_ROS_HH_
#define _NET2_TEST_ROS_HH_

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

using namespace std;
using namespace lmt::Tools::Network;

namespace lmt
{

class Net2TestROS 
{
public:
  Net2TestROS(ros::NodeHandle &nh,
              ros::NodeHandle &pnh,
              int argc,
              char *argv[]);

  std::vector<char> callbackData(std::vector<char> buffer, unsigned int priority, std::string sender);
  std::vector<char> callbackService(std::vector<char> buffer, unsigned int priority, std::string sender);

  std::shared_ptr<Net2Publisher> publisher;
  std::shared_ptr<Net2Subscriber> subscriber;
  std::shared_ptr<Net2Service> service;
  std::shared_ptr<Net2Client> client;
  std::shared_ptr<Net2AsyncClient> async_client;

  void update();
  int test_step = 0;
  ~Net2TestROS();
  void kill();
  Net2 *net2;
};

} // namespace lmt

#endif /* _NET2_TEST_ROS_HH_ */
