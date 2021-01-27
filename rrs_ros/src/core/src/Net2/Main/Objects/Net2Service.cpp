#include "Net2/Net2Service.h"
#include "Net2/Enum.h"

namespace lmt
{
namespace Tools
{
namespace Network
{

Net2Service::Net2Service(const std::string &name,void *context) : Net2Base(name, Net2NetworkType::SERVICE,context)
{

}

ProcessResult<int> Net2Service::Start()
{
  ProcessResult<int> result;

  if (internal_state != Net2State::STOPPED)
  {
    result.success = false;
    result.message = "Invalid state";
    return result;
  }

  try
  {
    reportLog("Starting Router name " + getName(), LogType::INFO, section);

    socket = zmq_socket(zmqcontext,ZMQ_ROUTER);

    setState(Net2State::STARTED);

    char port[1024];
    size_t size = sizeof(port);

    zmq_bind(socket,"tcp://*:*");

    zmq_getsockopt(socket,ZMQ_LAST_ENDPOINT, &port, &size);

    std::string str_port = port;

    std::vector<std::string> items;

    boost::split(items,str_port,boost::is_any_of(":"));

    ROS_INFO_STREAM("Port: " << items[2]);

    local_port = std::stoi(items[2]);

    //srand (time(NULL));
    //int v1 = rand() % 1000 + 1;
    //this->monitor_path = "inproc://router" + name + ".inproc" + std::to_string(local_port) + std::to_string(v1);

    thread_exit = false;

    //int rc = zmq_socket_monitor (socket, this->monitor_path.c_str(), ZMQ_EVENT_ALL);

    //monitor_thread = new std::thread(&Net2Service::monitorThread,this);

    socket_thread = new std::thread(&Net2Service::socketThread,this);

    reportLog("Router is ready on " + path + ":" + std::to_string(local_port), LogType::INFO, section);

    result.success = true;
    result.result = local_port;
    result.message = "Done";
  }
  catch (const std::runtime_error &e)
  {
    result.success = false;
    result.message = e.what();
    reportLog(e.what(), LogType::ERROR, section);
  }

  return result;
}
}
}
}
