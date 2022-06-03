//Roboland International Inc. 2016
#include "Net1/network.hh"

using namespace std;

namespace roboland
{

Network::Network(int port,
                 int subsribe_port,
                 int responser_port,
                 std::string net_name,
                 std::string subscribe_ip,
                 bool is_req,
                 int net_timeout) :
  tcp_port(port),
  name(net_name),
  subscribe_tcp_ip(subscribe_ip),
  responser_tcp_port(responser_port),
  subscribe_tcp_port(subsribe_port),
  is_req_mode(is_req),
  app_exit(false),
  thread_main(&Network::thrMain,this)
{
  is_restart = false;
  is_inited = false;
  is_connected = false;
  time_out_val = net_timeout;
  time_out = time_out_val;

  zmqcontext = new zmq::context_t(1);

  if ( is_req )
    start();
  else
    setup();
}

void Network::setup()
{
  is_inited = true;

  connectionp = "tcp://*:" + to_string(tcp_port);
  printf("ZMQ Publisher Started :  %s for %s \n",connectionp.c_str(),name.c_str());

  publisher  = new zmq::socket_t((*zmqcontext), ZMQ_PUB);
  publisher->bind(connectionp);

  connections = "tcp://" + subscribe_tcp_ip + ":" + to_string(subscribe_tcp_port);
  printf("ZMQ Subscriber Started :  %s for %s \n",connections.c_str(),name.c_str());

  subscriber = new zmq::socket_t((*zmqcontext), ZMQ_SUB);
  subscriber->connect(connections);

  const char *filter = "";
  subscriber->setsockopt(ZMQ_SUBSCRIBE, filter, strlen (filter));

  thread_ZMQ = new boost::thread(&Network::thrZmqSubscriber,this);

  last_read_time = 0;
}

void Network::start()
{
  thread_Req = new boost::thread(&Network::thrZmqReq,this);
}

void Network::thrMain()
{
  app_exit = false;
  wait(1000);
  while ( app_exit == false )
  {
    if ( is_connected )
    {
      time_out--;
      if ( time_out <= 0)
      {
        printf("Time out %d for conenction in %s\n",time_out_val,name.c_str());
        time_out = time_out_val;
        is_connected = false;

        if ( statusCallBackFunction )
          statusCallBackFunction(is_connected);

        if ( is_req_mode )
        {
          killSetup();
        }

      }
    }
    else
      if ( is_inited )
      {
        if ( is_req_mode )
        {
          time_out--;
          if ( time_out <= 0)
          {
            time_out = time_out_val;
            is_connected = false;
            printf("Time out %d for init in %s\n",time_out_val,name.c_str());
            killSetup();
          }
        }
      }

    wait(1000);
  }
}

Network::~Network()
{
  kill();
}

void Network::receiveCallback(char * data,int size,int index)
{
  //nothing yet
}

void Network::kill()
{
  killConnection();
  killSetup();
}

void Network::killConnection()
{
  if ( is_req_mode )
  {
    app_exit_req = true;

    responser->close();

    this->thread_ZMQ->interrupt();
    delete this->thread_ZMQ;
  }
}

void Network::killSetup()
{
  is_inited = false;
  app_exit_sub = true;

  publisher->close();
  subscriber->disconnect(connections);

  this->thread_ZMQ->interrupt();
  delete this->thread_ZMQ;
}

void Network::wait(int ms)
{
  //wait
  boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

void Network::thrZmqSubscriber()
{
  app_exit_sub = false;
  wait(1000);
  while ( !app_exit_sub )
  {
    try
    {
      zmq::message_t update;
      subscriber->recv(&update);
      char* get = static_cast<char*>(update.data());

      if ( update.size() > 0 && is_inited)
      {
        //Send to parent
        if ( dataCallBackFunction )
          dataCallBackFunction(get,(int)update.size(),index);

        time_out = time_out_val;

        is_connected = true;
      }
    }
    catch ( std::exception e)
    {

    }
  }
}

void Network::thrZmqReq()
{

  bool re_init = true;
  app_exit_req = false;
  wait(1000);

  while ( !app_exit_req )
  {
    try
    {
      if ( re_init )
      {
        connectionp = "tcp://*:" + to_string(responser_tcp_port);

        responser  = new zmq::socket_t((*zmqcontext), ZMQ_REP);

        int timeout = 2000;
        int keepalive = 1;

        responser->setsockopt(ZMQ_RCVTIMEO,&timeout, sizeof (int));
        responser->setsockopt(ZMQ_TCP_KEEPALIVE,&keepalive, sizeof (int));
        //responser->setsockopt(ZMQ_TCP_KEEPALIVE_IDLE,&timeout, sizeof (int));
        responser->bind(connectionp);

        re_init = false;

        printf("ZMQ Responser Started : %s for %s \n",connectionp.c_str(),name.c_str());
        wait(1000);

      }

      zmq::message_t update;
      bool result = responser->recv(&update);

      //      if ( result == false)
      //      {
      //        printf("ZMQ Error No : %s for %s \n",zmq_strerror(zmq_errno()),name.c_str());
      //      }
      //      else
      //      {
      //        printf("Get result for %s\n",name.c_str());
      //      }

      if ( update.size() > 0)
      {
        char* get = static_cast<char*>(update.data());

        RobolandConnection msg;
        msg.ParseFromArray(get,(int)update.size());

        if ( msg.status() == "alive" && msg.ip() == subscribe_tcp_ip && is_inited )
        {
          time_out = time_out_val;
          is_connected = true;

          msg.set_time(last_read_time);
          msg.set_status("ack");

          connectionWrite(msg);
        }
        else if ( msg.status() == "connect" )
        {
          printf("Get connect for %s\n",name.c_str());

          if ( is_connected )
          {
            printf("REQ reject for %s\n",name.c_str());
            std::cout<<"Get Req Reject for"<<name.c_str();
          }
          else
          {
            if ( is_inited == false )
            {
              is_connected = true;
              msg.set_status("connected");
              subscribe_tcp_ip = msg.ip();
              connectionWrite(msg);
              time_out = time_out_val;

              if ( statusCallBackFunction )
                statusCallBackFunction(is_connected);

              setup();
            }
            else
            {
              printf("REQ busy for %s\n",name.c_str());
              std::cout<<"Get Req Busy for"<<name.c_str();
            }
          }
        }
        else
        {
          printf("REQ error for %s\n",name.c_str());
          std::cout<<"Get Req Error for"<<name.c_str();
        }
      }
    }
    catch ( zmq::error_t t)
    {
      printf("REQ error zmq::error_t for %s\n",name.c_str());
      std::cout<<"Get Req Exp for"<<name.c_str();
      is_connected = false;
      time_out = time_out_val;

      delete responser;
      re_init = true;
    }

    wait(10);
  }
}

void Network::tcpWrite(char * buffer,int size)
{
  if ( is_connected && is_inited )
  {
    publisher->send(buffer, size);
  }
  else
  {
    if ( is_req_mode == false )
    {
      publisher->send(buffer, size);
    }
  }
}

void Network::connectionWrite(RobolandConnection msg)
{
  if ( responser )
  {
    int size = msg.ByteSize();
    char buffer[size];
    msg.SerializeToArray(buffer,size);
    responser->send(buffer,size);
  }
}

}





