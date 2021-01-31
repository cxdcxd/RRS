#include "Net2/Net2Base.h"

namespace lmt
{
namespace Tools
{
namespace Network
{

Net2Base::Net2Base(const std::string &name, Net2NetworkType type, void *context)
{
  this->zmqcontext = context;
  this->section += "-" + name;
  this->name = name;
  this->network_type = type;

  section = this->name;
  setState(Net2State::STOPPED);

  //Send
  bytes_send = std::make_shared<TransmitedData>();
  bytes_send->delegateTransmitedChanged = std::bind(&Net2Base::Bytes_send_delegateTransmitedChanged,this);

  //Receive
  bytes_receive = std::make_shared<TransmitedData>();
  bytes_receive->delegateTransmitedChanged = std::bind(&Net2Base::Bytes_receive_delegateTransmitedChanged,this);
}

ProcessResult<int> Net2Base::Stop()
{
  ProcessResult<int> result;

  if (internal_state == Net2State::STOPPED || internal_state == Net2State::STOPPING)
  {
    result.success = false;
    result.message  = "Invalid state";
    return result;
  }

  try
  {
    internal_state = Net2State::STOPPING;

    thread_exit = true;

    connection_count = 0;
    sequence = 0;

    if (network_type == Net2NetworkType::PUBLISHER || network_type == Net2NetworkType::SERVICE)
    {
      if (network_type == Net2NetworkType::PUBLISHER)
      {
        if (socket != nullptr)
        {
          zmq_close (socket);
        }
      }

      if ( callbackRemoveService )
        callbackRemoveService(getName());
    }

    result.success = true;
    result.message = "Done";

    //TODO : correct monitor issues
    //if (monitor != null && monitor.IsRunning)
    //monitor.Stop();
  }
  catch (std::runtime_error e)
  {
    result.success = false;
    result.message = e.what();
  }

  setState(Net2State::STOPPED);

  return result;
}

ProcessResult<int> Net2Base::Start(const std::string &remote_ip, int remote_port)
{
  ProcessResult<int> r;
  return r;
}

ProcessResult<int> Net2Base::Start(const std::string &name)
{
  ProcessResult<int> r;
  return r;
}

ProcessResult<int> Net2Base::Start()
{
  ProcessResult<int> r;

  if (network_type == Net2NetworkType::CLIENT || network_type == Net2NetworkType::ASYNCCLIENT || network_type == Net2NetworkType::SUBSCRIBER)
  {
    throw std::runtime_error("Use the start(name) instead");
  }

  return r;
}

ProcessResult<int> Net2Base::finalSend(Message message)
{
  ProcessResult<int> result;

  if (network_type == Net2NetworkType::SERVICE || network_type == Net2NetworkType::PUBLISHER)
  {
    if (internal_state != Net2State::STARTED)
    {
      result.success = false;
      result.message = "Socket is not started yet";
      return result;
    }
  }
  else
  {
    if (internal_state != Net2State::CONNECTED)
    {
      result.success = false;
      result.message = "Socket is not connected yet";
      return result;
    }
  }

  try
  {
    int size = message.ByteSize();
    char buffer[size];

    //ROS_INFO_STREAM("Message size " << size);

    message.SerializeToArray(buffer,size);

    if ( message.header().zmq_router_address() != "")
    {
      //Address
      std::string address1 = message.header().zmq_router_address();
      int size1 = message.header().zmq_router_address().size();
      char buffer1[size1];

      for (int i = 0 ; i < size1 ; i++)
        buffer1[i] = address1[i];

      //Empty
      std::string address2 = "";
      int size2 = 0;
      char buffer2[size2];

      ROS_WARN_STREAM("address size " << size1);
      zmq_send(socket,buffer1,size1,ZMQ_SNDMORE);

      ROS_WARN_STREAM("blank size " << size2);
      zmq_send(socket,buffer2,size2,ZMQ_SNDMORE);
    }

    //ROS_WARN_STREAM("main size " << size);
    zmq_send(socket,buffer,size,0);

    bytes_send->setBytes(bytes_send->getBytes() + static_cast<unsigned long long>(size));

    result.success = true;
    result.message = "Done";

    if (network_type == Net2NetworkType::CLIENT)
    {
      req_wait = true;
    }
    else
    {
      result.success = false;
      result.message = "Timout after " + std::to_string(time_out_write_ms) + " ms";
    }
  }
  catch (std::runtime_error e)
  {
    result.success = false;
    result.message = e.what();
    reportLog(e.what(), LogType::ERROR, section);
  }

  return result;
}

ProcessResult<int> Net2Base::send(char* data,int size, uint64_t priority)
{
  last_send_receive_time = callbackGetTime();
  Message msg = createMessage(data,size);
  msg.mutable_header()->set_priority(priority);
  return finalSend(msg);
}

ProcessResult<std::shared_ptr<Message>> Net2Base::sendSync(char* data,int size, uint64_t priority, uint64_t send_time_out)
{
  ProcessResult<std::shared_ptr<Message>> result;
  result.success = false;
  result.message = "Invalid use of sendSync";
  return result;
}

bool Net2Base::getIsReqWait() const
{
  return req_wait;
}

std::string Net2Base::getLastDesireName() const
{
  return last_desire_name;
}

bool Net2Base::getIsAutoConnection() const
{
  return is_auto_connection;
}

void Net2Base::setIsAutoConnection(bool value)
{
  is_auto_connection = value;
}

int Net2Base::getConnectionCount() const
{
  return connection_count;
}

std::string Net2Base::getBytesSend() const
{
  return bytes_send->toString();
}

std::string Net2Base::getBytesReceive() const
{
  return bytes_receive->toString();
}

std::string Net2Base::getPath() const
{
  return path;
}

int Net2Base::getLocalPort() const
{
  return local_port;
}

std::string Net2Base::getLocalIp() const
{
  return local_ip;
}

int Net2Base::getRemotePort() const
{
  return remote_port;
}

std::string Net2Base::getRemoteIp() const
{
  return remote_ip;
}

Net2State Net2Base::getState() const
{
  return internal_state;
}

Net2NetworkType Net2Base::getNetworkType() const
{
  return network_type;
}

std::string Net2Base::getName() const
{
  return name;
}

void Net2Base::reportState(std::string sender)
{
  if ( delegateStateChanged )
       delegateStateChanged(sender);
}

void Net2Base::reportData(Message packet, std::string sender)
{
  //ROS_INFO("Sub get data");
  last_send_receive_time = callbackGetTime();

  if ( internal_state == Net2State::STARTED)
  {    
     //ROS_INFO("Connected !");
     Monitor_Connected();     
  }
  
  if ( delegateNewData )
  {
      uint64_t priority = packet.header().time_span();

      std::string buffer = packet.payload();
      std::vector<char> vector_buffer;

      for ( int i = 0 ; i < buffer.size() ; i++)
      {
       vector_buffer.push_back(buffer[i]);
      }

      delegateNewData(vector_buffer,priority,sender);
  }

  //   if (network_type == Net2NetworkType::SERVICE)
  //   {
  //     if (result.size() > 0)
  //     {
  //       Message ack_message;
  //       Header *ack_header = new Header();

  //       std::string ack_buffer = "";

  //       for ( int i = 0 ; i < result.size() ; i++)
  //       {
  //         ack_buffer += result[i];
  //       }

  //       ack_message.set_payload(ack_buffer);

  //       int s = packet.header().sequence();

  //       ack_header->set_mode(Mode::ACK);
  //       ack_header->set_message_type(Type::ROUTER);
  //       ack_header->set_sequence(packet.header().sequence());
  //       ack_header->set_time_span(callbackGetTime());
  //       ack_header->set_zmq_router_address(packet.header().zmq_router_address());

  //       ack_message.set_allocated_header(ack_header);

  //       finalSend(ack_message);
  //     }
  //   }
    //}
}

void Net2Base::reportLog(const std::string &log_message, LogType log_type, const std::string &section)
{
  //if ( delegateNewLog )
  //  delegateNewLog(log_message, log_type, section);

  ROS_INFO_STREAM("Log " << log_message << " " << (int)log_type << " " << section);
}

void Net2Base::msdelay(int ms)
{
  boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

int Net2Base::get_monitor_event (void *monitor)
{
  // // First frame in message contains event number and value
  // zmq_msg_t msg;
  // zmq_msg_init (&msg);
  // if (zmq_msg_recv (&msg, monitor, 0) == -1)
  //   return -1; // Interrupted, presumably
  // assert (zmq_msg_more (&msg));

  // uint8_t *data = (uint8_t *) zmq_msg_data (&msg);
  // uint16_t event = *(uint16_t *) (data);

  // // Second frame in message contains event address
  // zmq_msg_init (&msg);
  // if (zmq_msg_recv (&msg, monitor, 0) == -1)
  //   return -1; // Interrupted, presumably
  // assert (!zmq_msg_more (&msg));

  return 0;
}

void Net2Base::monitorThread ()
{
  // monitor = zmq_socket (zmqcontext, ZMQ_PAIR);
  // int rc = zmq_connect (monitor, this->monitor_path.c_str());

  // //ROS_INFO_STREAM("Monitor RC " << rc);
  // //ROS_INFO_STREAM("Monitor path " << this->monitor_path);

  // int event = 0;

  // while (event != -1 && thread_exit == false)
  // {
  //   event = get_monitor_event (monitor);

  //   switch (event)
  //   {
  //   case ZMQ_EVENT_LISTENING:
  //     ROS_WARN_STREAM("Listening socket descriptor for " << getName());
  //     break;
  //   case ZMQ_EVENT_ACCEPTED:
  //     ROS_WARN_STREAM("Accepted socket descriptor for " << getName());
  //     Monitor_Connected();
  //     break;
  //   case ZMQ_EVENT_CLOSE_FAILED:
  //     ROS_WARN_STREAM("Socket close failure error code for " << getName());
  //     break;
  //   case ZMQ_EVENT_CLOSED:
  //     ROS_WARN_STREAM("Closed socket descriptor for " << getName());
  //     break;
  //   case ZMQ_EVENT_DISCONNECTED:
  //     ROS_WARN_STREAM("Disconnected socket descriptor for " << getName());
  //     Monitor_Disconnected();
  //     break;
  //   case ZMQ_EVENT_CONNECTED:
  //     ROS_WARN_STREAM("Connected socket descriptor for " << getName());
  //     Monitor_Connected();
  //     break;
  //   case ZMQ_EVENT_ACCEPT_FAILED:
  //     ROS_WARN_STREAM("Socket accept failure descriptor for " << getName());
  //     break;
  //   }

  // }

  // ROS_ERROR_STREAM("Monitor exited " << getName());

  // zmq_close(monitor);
}

void Net2Base::socketThread()
{
  char* address_buffer;
  int address_buffer_size = 0;

  try
  {
    while (thread_exit == false && socket != nullptr)
    {
      if ((network_type == Net2NetworkType::CLIENT && req_wait) || network_type != Net2NetworkType::CLIENT)
      {
        try
        {
          //Message
          zmq_msg_t msg;
          zmq_msg_init (&msg);

          //Receive
          int rc = zmq_recvmsg (socket, &msg, 0);
          req_wait = false;

          //More
          int more;
          size_t more_size = sizeof (more);
          zmq_getsockopt(socket,ZMQ_RCVMORE,&more, &more_size);

          //Get Size/Data
          int size = (int)zmq_msg_size(&msg);                    //Size
          char *buffer = static_cast<char*>(zmq_msg_data(&msg)); //Data

          //ROS_INFO_STREAM("new recvmsg size " << size << " more ? " << more);

          if (more == 0 && size > 0)
          {
            Message packet;
            //ROS_WARN_STREAM("Try parse normal message " << size);

            packet.ParseFromArray(buffer,size);

            if (packet.ByteSize() > 0)
            {
              bytes_receive->setBytes(bytes_receive->getBytes() + static_cast<unsigned long long>(size));

              if (address_buffer_size > 0)
              {
                packet.mutable_header()->set_zmq_router_address(address_buffer,address_buffer_size);
              }

              reportData(packet, getName());
            }
            else
            {
              ROS_ERROR("Invalid packet");
              reportLog("Ivalid packet", LogType::ERROR, section);
            }
          }
          else if (more == 1 && size > 0)
          {
            address_buffer = (char*)malloc( size * sizeof(char) );

            for ( int i = 0 ; i < size ; i++)
            {
              address_buffer[i] = buffer[i];
            }

            address_buffer_size = size;
          }
        }
        catch (std::runtime_error e)
        {
          reportLog("Invalid message ", LogType::ERROR, section);
        }
      }
      else
      {
        msdelay(1);
      }
    }
  }
  catch (const std::runtime_error &e)
  {
    reportLog("error in thread", LogType::ERROR, section);
  }

  if (socket != nullptr)
  {
    zmq_close(socket);
  }

  if (internal_state != Net2State::STOPPING && internal_state != Net2State::STOPPED)
  {
    Stop();
  }

  reportLog("Thread exited clearly for " + getName(), LogType::INFO, section);
}

void Net2Base::Monitor_Disconnected()
{
  if (connection_count > 0)
  {
    connection_count--;
  }

  if (network_type == Net2NetworkType::SUBSCRIBER || network_type == Net2NetworkType::CLIENT || network_type == Net2NetworkType::ASYNCCLIENT)
  {
    Stop();
  }

  if ( delegateReceiveChanged)
    delegateReceiveChanged(getName());
}

void Net2Base::Monitor_Connected()
{
  connection_count++;

  if (network_type != Net2NetworkType::PUBLISHER && network_type != Net2NetworkType::SERVICE)
  {
    setState(Net2State::CONNECTED);
  }

  if ( delegateReceiveChanged)
    delegateReceiveChanged(getName());
}

Message Net2Base::createMessage(char* buffer,int size)
{
  req_sequence++;

  Message msg;
  Header *header = new Header;

  header->set_sequence(req_sequence);
  header->set_time_span(callbackGetTime());
  header->set_source_channel_name(getName());

  msg.set_payload(buffer,size);
  msg.set_allocated_header(header);

  return msg;
}

void Net2Base::setState(Net2State new_state)
{
  if (this->internal_state != new_state)
  {
    this->internal_state = new_state;

    //reportState(getName());
  }
}

void Net2Base::update()
{
  if (network_type == Net2NetworkType::SERVICE || network_type == Net2NetworkType::PUBLISHER)
  {
     if (internal_state == Net2State::STARTED )
     {
       if ( callbackAdvertiseService ) callbackAdvertiseService(name, local_port);

       if ( network_type == Net2NetworkType::PUBLISHER )
       {
         long diff = callbackGetTime() - last_send_receive_time;
         if (diff > 2000)
         { 
            //std::cout << "Publisher heart beat 2 sec" << std::endl;
            char data[1];
            send(data,1, 10);
         }
       }
     }
  }
  else if (network_type == Net2NetworkType::ASYNCCLIENT || network_type == Net2NetworkType::SUBSCRIBER || network_type == Net2NetworkType::CLIENT)
  {
    if (internal_state == Net2State::STOPPED)
    {
        if (is_auto_connection && last_desire_name != "")
        {
          //std::cout << "Autoconnection for subsctriber" << std::endl;
          Start(last_desire_name);
        }
    }
    else if ( internal_state == Net2State::STARTED || internal_state == Net2State::CONNECTED)
    {
       if ( callbackGetTime() - last_send_receive_time > 5000)
       {
          service_counter = 0;
          last_send_receive_time = callbackGetTime();
          //std::cout << "Detect subscriber timeout after 5 sec" << std::endl;
          Stop();
        }
    }
  }

}

void Net2Base::Bytes_receive_delegateTransmitedChanged()
{
  if (delegateReceiveChanged)
    delegateReceiveChanged(getName());
}

void Net2Base::Bytes_send_delegateTransmitedChanged()
{
  if (delegateSendChanged)
    delegateSendChanged(getName());
}

void Net2Base::setServiceKey(const std::string &key)
{
  service_request_key = key;
}

std::string Net2Base::getServiceKey()
{
  return service_request_key;
}
}
}
}
