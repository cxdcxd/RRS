#include "Net2/Net2AsyncClient.h"
#include "Net2/Enum.h"
#include "Net2/Net2.h"


namespace lmt
{
namespace Tools
{
namespace Network
{

Net2AsyncClient::Net2AsyncClient(void *context) : Net2Base("",Net2NetworkType::ASYNCCLIENT,context)
{

}

ProcessResult<int> Net2AsyncClient::Start(const std::string &name)
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

ProcessResult<int> Net2AsyncClient::Start(const std::string &remote_ip, int remote_port)
{
  ProcessResult<int> result;

  if (remote_ip == "" || remote_port < 1024)
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
    reportLog("Starting Dealer for ip " + remote_ip + " and port " + std::to_string(remote_port), LogType::INFO, section);

    std::string path = "tcp://" + remote_ip + ":" + std::to_string(remote_port);

    socket = zmq_socket(zmqcontext, ZMQ_DEALER);

    setState(Net2State::STARTED);

    //srand (time(NULL));
    //int v1 = rand() % 1000 + 1;

    //this->monitor_path = "inproc://dealer" + name + ".inproc" + std::to_string(remote_port) + std::to_string(v1);

    //int rc = zmq_socket_monitor (socket, this->monitor_path.c_str(), ZMQ_EVENT_ALL);

    //monitor_thread = new std::thread(&Net2AsyncClient::monitorThread,this);

    zmq_connect(socket,path.c_str());

    thread_exit = false;
    socket_thread = new std::thread(&Net2AsyncClient::socketThread,this);

    reportLog("Dealer is connected to " + path, LogType::INFO, section);

    result.success = true;
    result.message = "Done";
  }
  catch (const std::runtime_error &e)
  {
    result.success = false;
    result.message = e.what();
  }

  return result;
}
}
}
}
