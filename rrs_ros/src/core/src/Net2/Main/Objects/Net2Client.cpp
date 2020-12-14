#include "Net2/Net2Client.h"
#include "Net2/Enum.h"
#include "Net2/Net2.h"

namespace Roboland
{
namespace Tools
{
namespace Network
{

Net2Client::Net2Client(void *context) : Net2Base("", Net2NetworkType::CLIENT,context)
{

}

void Net2Client::reportData(Message packet, std::string sender)
{
  if (wait_for_reply && packet.header().sequence() == req_sequence)
  {
    {
      boost::mutex::scoped_lock lock(mutex);
      result_message = std::make_shared<Message>(packet);
      wait_for_reply = false;
    }
  }
}

ProcessResult<int> Net2Client::Start(const std::string &name)
{
  ProcessResult<int> res;
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

  return res;
}

ProcessResult<std::shared_ptr<Message>> Net2Client::sendSync(char* buffer,int size, unsigned int priority, unsigned int send_time_out)
{
  ProcessResult<std::shared_ptr<Message>> res;

  if (getState() == Net2State::CONNECTED)
  {
    Message msg = createMessage(buffer,size);
    msg.mutable_header()->set_priority(priority);

    result_message.reset();

    ProcessResult<int> send_res = finalSend(msg);

    if (send_res.success)
    {
      if (send_time_out != 0)
      {
        time_out_counter = send_time_out;
      }
      else
      {
        time_out_counter = time_out;
      }

      wait_for_reply = true;

      while (wait_for_reply && time_out_counter > 0)
      {
        time_out_counter--;
        msdelay(1);
      }

      if (time_out_counter <= 0)
      {
        res.success = false;
        res.message = "TimeOut";

        setState(Net2State::STOPPED);
      }
      else if ((getState() == Net2State::CONNECTED) && time_out_counter > 0 && result_message != nullptr)
      {
        res.success = true;
        res.message = "Done";

        {
          boost::mutex::scoped_lock lock(mutex);
          res.result = result_message;
        }

      }
      else if (getState() != Net2State::CONNECTED)
      {
        res.success = false;
        res.message = "Canceled";
      }
    }
    else
    {
      ROS_ERROR_STREAM("Final send failed " << res.message);
    }

  }
  else
  {
    res.success = false;
    res.message = "Channel is not connected yet";
  }

  return res;
}

ProcessResult<int> Net2Client::Start(const std::string &remote_ip, int remote_port)
{
  ProcessResult<int> result;

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
    reportLog("Starting requester name " + getName(), LogType::INFO, section);

    std::string path = "tcp://" + remote_ip + ":" + std::to_string(remote_port);

    socket = zmq_socket(zmqcontext,ZMQ_REQ);

    setState(Net2State::STARTED);

    //srand (time(NULL));
    //int v1 = rand() % 1000 + 1;

    //this->monitor_path = "inproc://requester" + name + ".inproc" + std::to_string(remote_port) + std::to_string(v1);

    //int rc = zmq_socket_monitor (socket, this->monitor_path.c_str(), ZMQ_EVENT_ALL);

    //monitor_thread = new boost::thread(&Net2Client::monitorThread,this);

    zmq_connect(socket,path.c_str());

    thread_exit = false;
    socket_thread = new std::thread(&Net2Client::socketThread,this);

    reportLog("Requester is ready on " + path, LogType::INFO, section);

    result.success = true;
    result.message = "Done";
  }
  catch (const std::runtime_error &e)
  {
    reportLog("Cant start Requester on " + path + ", " + e.what(), LogType::ERROR, section);
    result.success = false;
    result.result = -1;
    result.message = e.what();
  }

  return result;
}

}
}
}
