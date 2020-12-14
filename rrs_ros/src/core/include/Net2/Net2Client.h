#ifndef NET2_CLIENT_
#define NET2_CLIENT_

#include "Net2Base.h"
#include <string>
#include <vector>
#include <stdexcept>
#include <boost/thread/mutex.hpp>
#include <memory>

namespace Roboland
{
namespace Tools
{
namespace Network
{
class Net2Client : public Net2Base
{
private:
  unsigned int time_out = 1000;
  unsigned int time_out_counter = 0;
  std::shared_ptr<Message> result_message = nullptr;
  bool wait_for_reply = false;
  boost::mutex mutex;

public:
  Net2Client(void* context);

protected:
  void reportData(Message packet, std::string sender) override;

public:
  ProcessResult<int> Start(const std::string &name) override;
  ProcessResult<std::shared_ptr<Message>> sendSync(char* buffer,int size, unsigned int priority = 0, unsigned int send_time_out = 0) override;
  ProcessResult<int> Start(const std::string &remote_ip, int remote_port) override;
};
}
}
}

#endif /* NET2_CLIENT_ */

