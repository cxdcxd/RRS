#ifndef NET2_HH_
#define NET2_HH_

#include <string>
#include <unordered_map>
#include <vector>
#include <stdexcept>
#include <memory>

#include "bridge_msgs/Scene.pb.h"

#include "Enum.h"

//ZMQ V3
#include "zmq.h"

#include <mutex>
#include <thread>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <boost/asio.hpp>

#include "ProcessResult.h"

#include "Net2Publisher.h"
#include "Net2Subscirber.h"
#include "Net2Service.h"
#include "Net2Client.h"
#include "Net2AsyncClient.h"

#include "Net2Base.h"
#include "Net2Consul.h"
#include "Net2Helper.h"

#include "ros/ros.h"

namespace Roboland
{
namespace Tools
{
namespace Network
{

class Net2
{
  std::function<void (std::string sender)> delegateStateChanged;
  std::function<void (const std::string &log_message, LogType log_type, const std::string &section)> delegateNewLog;
  std::function<std::vector<char> (std::vector<char> buffer, unsigned int priority, std::string sender)> delegateNewData;

  bool is_handle_intternal_service_calls = true;
public:

  Net2State state = Net2State::STOPPED;
  std::string name_space = "";
  std::string host_name = "";
  std::shared_ptr<Net2Consul> net2_consul;
  std::vector<std::shared_ptr<Net2Base>> channels_list;

  void* context;

  boost::asio::io_service io_service;
  boost::asio::deadline_timer *timer;
  std::thread  *thread;

  void boostThread();
  long getTime();
  ProcessResult<ppconsul::ServiceInfo> getService(std::string name);

  static std::shared_ptr<Net2> instance;
  boost::mutex mutex;
  std::shared_ptr<Net2Service> net2_service;

public:
  Net2();

  Net2Helper *net2_helper;
  std::string getNameSpace() const;

  std::string getHostName() const;
  Net2Config net2_config;
  std::shared_ptr<Net2Publisher> publisher(const std::string &name);
  std::shared_ptr<Net2Service> service(const std::string &name);
  std::shared_ptr<Net2Subscriber> subscriber();
  std::shared_ptr<Net2Client> clinet();
  std::shared_ptr<Net2AsyncClient> asynclient();

private:
  void addChannel(std::shared_ptr<Net2Base> item);

public:
  void Init(const Net2Config &config, const std::string &vname_space, const std::string &vhost_name = "");

private:
  void Net2_service_delegateStateChanged(std::string sender);
  void Net2_service_delegateNewLog(const std::string &log_message, LogType log_type, const std::string &section);
  std::vector<char> Net2_service_delegateNewData(std::vector<char> buffer, unsigned int priority, std::string sender);
  void Net2_consul_delegateGetEvent(ProcessResult<int> result, const std::string &key);

public:
  void removeService(const std::string &name);

  ProcessResult<int> getAsyncService(const std::string &name);

  void advertiseService(const std::string &name, unsigned int local_port);

private:
  void Main_timer_Elapsed();

public:
  void Shutdown();
};

}
}
}

#endif /* NET2_HH_ */
