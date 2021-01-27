#include "Net2/Net2Subscirber.h"
#include "Net2/Enum.h"
#include "Net2/Net2.h"

namespace lmt
{
namespace Tools
{
namespace Network
{

Net2Subscriber::Net2Subscriber(void* context) : Net2Base("", Net2NetworkType::SUBSCRIBER,context)
{

}

ProcessResult<int> Net2Subscriber::Start(const std::string &name)
{
  ProcessResult<int> result;

  if (internal_state == Net2State::STOPPED && name.empty() == false)
  {
    last_desire_name = name;
    this->name = name;

    if ( callbackGetServiceInfo )
    {
      ProcessResult<ppconsul::ServiceInfo> result2 = callbackGetServiceInfo(name);

      if (result2.success )
      {
         ppconsul::ServiceInfo info = result2.result;
         Start(info.address, info.port);
      }
    }
  }

  return result;
}

ProcessResult<int> Net2Subscriber::Start(const std::string &remote_ip, int remote_port)
{
  ProcessResult<int> result;
  thread_exit = false;

  if (remote_ip == "" || remote_port < 1000)
  {
    result.message = "Invalid arguments";
    result.success = false;
    return result;
  }

  if (internal_state != Net2State::STOPPED)
  {
    result.success = false;
    result.message = "Invalid state";
    return result;
  }

  try
  {
    reportLog("Starting subscriber " + getName(), LogType::INFO, section);

    this->remote_ip = remote_ip;
    this->remote_port = remote_port;

    path = "tcp://" + remote_ip + ":" + std::to_string(remote_port);

    socket = zmq_socket(zmqcontext,ZMQ_SUB);

    setState(Net2State::STARTED);

    //srand (time(NULL));
    //int v1 = rand() % 1000 + 1;

    //this->monitor_path = "inproc://sub" + name + ".inproc" + std::to_string(remote_port) + std::to_string(v1);

    thread_exit = false;

    //int rc = zmq_socket_monitor(socket,this->monitor_path.c_str(),ZMQ_EVENT_ALL);

    //monitor_thread = new boost::thread(&Net2Publisher::monitorThread,this);

    zmq_connect(socket,path.c_str());

    const char *filter = "";

    zmq_setsockopt(socket,ZMQ_SUBSCRIBE, filter, strlen (filter));

    socket_thread = new std::thread(&Net2Subscriber::socketThread,this);

    result.message = "Done";
    result.success = true;

    reportLog("Subscriber " + getName() + " is setup for  " + path, LogType::INFO, section);
  }
  catch (const std::runtime_error &e)
  {
    result.message = e.what();
    result.result = false;
    reportLog(e.what(), LogType::ERROR, section);
  }

  return result;
}
}
}
}
