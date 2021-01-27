#include "Net2/Net2Publisher.h"
#include "Net2/Enum.h"

namespace lmt
{
namespace Tools
{
namespace Network
{

Net2Publisher::Net2Publisher(const std::string &name, void* context) : Net2Base(name, Net2NetworkType::PUBLISHER,context)
{

}

ProcessResult<int> Net2Publisher::Start()
{
  ProcessResult<int> result;

  if (internal_state != Net2State::STOPPED)
  {
    result.success = false;
    result.message = "Invalid state";
    return result;
  }

  reportLog("Starting publisher " + getName(), LogType::INFO, section);

  std::string path = "tcp://*";

  try
  {
    socket = zmq_socket(zmqcontext,ZMQ_PUB);

    bind_address = path;

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

    //this->monitor_path = "inproc://pub" + name + ".inproc" + std::to_string(remote_port) + std::to_string(v1);

    //int rc = zmq_socket_monitor (socket, this->monitor_path.c_str(), ZMQ_EVENT_ALL);

    //monitor_thread = new boost::thread(&Net2Publisher::monitorThread,this);

    reportLog("Publisher " + getName() + " is ready on " + path + ":" + std::to_string(local_port), LogType::INFO, section);

    result.message = "Done";
    result.success = true;
    result.result = local_port;

    setState(Net2State::STARTED);
  }
  catch (const std::runtime_error &e)
  {
    result.message = e.what();
    result.success = false;
    result.result = -1;
    reportLog(e.what(), LogType::ERROR, section);
  }

  return result;
}
}
}
}
