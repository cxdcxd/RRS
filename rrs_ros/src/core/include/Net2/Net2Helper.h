#ifndef NET2_HELPER_
#define NET2_HELPER_

#include <string>
#include <unordered_map>
#include <vector>
#include <stdexcept>
#include <memory>
#include <iostream>
#include <time.h>

//BOOST
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/asio.hpp>

#include "Enum.h"
#include "NTPClient.hh"

namespace Roboland
{
namespace Tools
{
namespace Network
{

class Net2Helper : public std::enable_shared_from_this<Net2Helper>
{

public:
  Net2Config net2_config;
  Net2Helper(const Net2Config &config,std::string settings_local_ip);
  ~Net2Helper();
  std::string getStationIp();
  std::vector<std::string> getAllInterfaceIps();
  NTPClient *ntp_client;
  long getTime();
  long syncTime();
  std::string local_ip;

private:
  std::string toBinary(std::__cxx11::string value);
  std::string toAND(std::string a,std::string b);
};
}
}
}

#endif /* NET2_HELPER_ */
