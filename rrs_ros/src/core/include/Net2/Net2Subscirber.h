#ifndef NET2_SUBSCRIBER_
#define NET2_SUBSCRIBER_

#include "Net2Base.h"
#include <string>
#include <stdexcept>
#include <memory>
#include "ProcessResult.h"

namespace lmt
{
namespace Tools
{
namespace Network
{
class Net2Subscriber : public Net2Base
{

public:
  Net2Subscriber(void* context);
  ProcessResult<int> Start(const std::string &name) override;
  ProcessResult<int> Start(const std::string &remote_ip, int remote_port) override;
};
}
}
}

#endif /* NET2_SUBSCRIBER_ */
