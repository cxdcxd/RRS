#ifndef NET2_CONSUL_
#define NET2_CONSUL_

#include <string>
#include <unordered_map>
#include <vector>
#include <stdexcept>
#include <memory>

#include "Enum.h"

#include "ppconsul/agent.h"
#include "ppconsul/consul.h"
#include "ppconsul/kv.h"
#include "ppconsul/ppconsul.h"

#include <thread>
#include <mutex>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/mutex.hpp>

#include "ProcessResult.h"
#include "ros/ros.h"

using ppconsul::Consul;
using namespace ppconsul::agent;

namespace lmt
{
namespace Tools
{
namespace Network
{

class Net2Consul : public std::enable_shared_from_this<Net2Consul>
{

private:
  std::vector<ppconsul::ServiceInfo> updated_list;
  std::vector<ppconsul::ServiceInfo> temp_updated_list;
  std::vector<ppconsul::ServiceInfo> sending_queue;
  void msdelay(int ms);
  bool is_thread_exit = false;
  ProcessResult<int> getServiceSync();
  void mThread();

  Consul *consul;
  Agent   *agent;

  boost::mutex mutex_send;
  boost::mutex mutex_get;
  std::thread  *thread;
  Net2ConsulMode consul_mode;

public:
  Net2Consul(const std::string &uri,Net2ConsulMode mode);
  void advertiseService(const std::string &name, unsigned int port, const std::string &address, std::__cxx11::string time);
  void removeService(const std::string &name);
  void shutdown();
  ppconsul::ServiceInfo getServiceInfo(const std::string &name);

};
}
}
}

#endif /* NET2_CONSUL_ */
