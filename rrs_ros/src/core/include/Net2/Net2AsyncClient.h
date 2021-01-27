#ifndef NET2_ASYNC_CLIENT_
#define NET2_ASYNC_CLIENT_

#include "Net2Base.h"
#include <string>
#include <stdexcept>
#include <memory>

namespace lmt
{
namespace Tools
{
namespace Network
{
class Net2AsyncClient : public Net2Base
{

public:
  Net2AsyncClient(void* context);
  ProcessResult<int> Start(const std::string &name) override;
  ProcessResult<int> Start(const std::string &remote_ip, int remote_port) override;
};
}
}
}

#endif /* NET2_ASYNC_CLIENT_ */

