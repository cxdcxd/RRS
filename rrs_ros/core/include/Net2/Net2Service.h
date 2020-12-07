#ifndef NET2_SERVICE_
#define NET2_SERVICE_

#include "Net2Base.h"
#include <string>
#include <vector>
#include <stdexcept>
#include <memory>

namespace Roboland
{
namespace Tools
{
namespace Network
{
class Net2Service : public Net2Base
{

public:
  Net2Service(const std::string &name, void* context);
  ProcessResult<int> Start() override;
};
}
}
}

#endif /* NET2_SERVICE_ */
