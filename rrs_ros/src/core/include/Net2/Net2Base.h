#ifndef NET2_BASE_
#define NET2_BASE_

#include <string>
#include <unordered_map>
#include <vector>
#include <stdexcept>
#include <memory>
#include <string>
#include <iostream>
#include <string>

#include "bridge_msgs/Scene.pb.h"

#include "Enum.h"

#include "zmq.h"

#include <thread>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/mutex.hpp>

#include "ppconsul/agent.h"

#include "ProcessResult.h"
#include "TransmitedData.h"

#include "Net2Helper.h"


namespace lmt
{
namespace Tools
{
namespace Network
{
class Net2Base
{

protected:
  //Members
  void *zmqcontext;
  void *socket;
  void *monitor;
  std::thread  *socket_thread;
  std::thread  *monitor_thread;

  std::shared_ptr<TransmitedData> bytes_send;
  std::shared_ptr<TransmitedData> bytes_receive;

  //Config
  Net2State internal_state = Net2State::STOPPED;
  Net2NetworkType network_type = static_cast<Net2NetworkType>(0);

  std::string last_desire_name = "";
  std::string name = "";
  std::string section;
  std::string monitor_path = "";

  bool thread_exit = false;
  bool is_auto_connection = true;
  bool req_wait = false;

  uint64_t time_out_read_ms = 10;
  uint64_t time_out_write_ms = 10;
  uint64_t local_port = 0;
  uint64_t remote_port = 0;

  int service_counter = 0;
  int time_out_write_ts;

  std::string local_ip = "";
  std::string remote_ip = "";
  std::string path = "tcp://*";
  std::string bind_address = "";
  std::string service_request_key = "";

  unsigned long long sequence = 0;
  unsigned long long req_sequence = 0;
  uint64_t last_send_receive_time = 0;
  uint64_t connection_count = 0;

  std::shared_ptr<void> update_mutex = nullptr;

  ProcessResult<int> finalSend(Message message);

  void reportState(std::string sender);
  void reportLog(const std::string &log_message, LogType log_type, const std::string &section);
  virtual void reportData(Message packet, std::string sender);

  Message createMessage(char *buffer, int size);

  void setState(Net2State new_state);
  void Bytes_receive_delegateTransmitedChanged();
  void Bytes_send_delegateTransmitedChanged();

public:
  void update();
  void socketThread();
  void monitorThread();
  void msdelay(int ms);
  void setServiceKey(const std::string &key);
  std::string getServiceKey();

  //Events
  std::function<void (std::string sender)> delegateStateChanged;
  std::function<void (const std::string &log_message, LogType log_type, const std::string &section)> delegateNewLog;
  std::function<std::vector<char> (std::vector<char> buffer, uint64_t priority, std::string sender)> delegateNewData;
  std::function<void (std::string sender)> delegateSendChanged;
  std::function<void (std::string sender)> delegateReceiveChanged;
  std::function<void (std::string name)> callbackRemoveService;
  std::function<void (std::string name,int port)> callbackAdvertiseService;
  std::function<ProcessResult<ppconsul::ServiceInfo> (std::string name)> callbackGetServiceInfo;
  std::function<long ()> callbackGetTime;

  virtual ProcessResult<int> send(char *data, int size, uint64_t priority = 0);
  virtual ProcessResult<std::shared_ptr<Message>> sendSync(char *data, int size, uint64_t priority = 0, uint64_t send_time_out = 0);

  bool getIsReqWait() const;
  std::string getLastDesireName() const;

  bool getIsAutoConnection() const;
  void setIsAutoConnection(bool value);
  int get_monitor_event(void *monitor);
  int getConnectionCount() const;

  std::string getBytesSend() const;
  std::string getBytesReceive() const;

  std::string getPath() const;
  int getLocalPort() const;
  std::string getLocalIp() const;
  int getRemotePort() const;
  std::string getRemoteIp() const;
  Net2State getState() const;
  Net2NetworkType getNetworkType() const;
  std::string getName() const;

  Net2Base(const std::string &name, Net2NetworkType type, void* context);
  virtual ProcessResult<int> Start(const std::string &remote_ip, int remote_port);
  virtual ProcessResult<int> Start(const std::string &name);
  virtual ProcessResult<int> Start();
  ProcessResult<int> Stop();

  void Monitor_Disconnected();
  void Monitor_Connected();
};
}
}
}

#endif /* NET2_BASE_ */
