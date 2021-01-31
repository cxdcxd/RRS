#ifndef NET2_CLIENT_
#define NET2_CLIENT_

#include "Net2Base.h"
#include <string>
#include <vector>
#include <stdexcept>
#include <boost/thread/mutex.hpp>
#include <memory>

namespace lmt
{
namespace Tools
{
namespace Network
{
class Net2Client : public Net2Base
{
private:
  uint64_t time_out = 1000;
  uint64_t time_out_counter = 0;
  std::shared_ptr<Message> result_message = nullptr;
  bool wait_for_reply = false;
  boost::mutex mutex;

public:
  Net2Client(void* context);

protected:
  void reportData(Message packet, std::string sender) override;

public:
  ProcessResult<int> Start(const std::string &name) override;
  ProcessResult<std::shared_ptr<Message>> sendSync(char* buffer,int size, uint64_t priority = 0, uint64_t send_time_out = 0) override;
  ProcessResult<int> Start(const std::string &remote_ip, int remote_port) override;
};
}
}
}

#endif /* NET2_CLIENT_ */

