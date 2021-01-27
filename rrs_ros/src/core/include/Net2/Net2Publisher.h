#ifndef NET2_PUBLISHER_
#define NET2_PUBLISHER_

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
class Net2Publisher : public Net2Base
{

public:
  Net2Publisher(const std::string &name, void* context);
  ProcessResult<int> Start() override;
};
}
}
}

#endif /* NET2_PUBLISHER_ */
