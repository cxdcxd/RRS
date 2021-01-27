#include <Net2/Net2.h>

namespace lmt
{
namespace Tools
{
namespace Network
{

Net2::Net2()
{
  channels_list = std::vector<std::shared_ptr<Net2Base>>();
}

std::string Net2::getNameSpace() const
{
  return name_space;
}

std::string Net2::getHostName() const
{
  return host_name;
}

std::shared_ptr<Net2Publisher> Net2::publisher(const std::string &name)
{
  if (state == Net2State::STARTED)
  {
    std::string new_name =  getNameSpace() + "-" + name;

    std::shared_ptr<Net2Publisher> publisher = std::make_shared<Net2Publisher>(new_name,context);

    publisher->callbackAdvertiseService = std::bind(&Net2::advertiseService,this,std::placeholders::_1,std::placeholders::_2);
    publisher->callbackRemoveService = std::bind(&Net2::removeService,this,std::placeholders::_1);
    publisher->callbackGetServiceInfo = std::bind(&Net2::getService ,this,std::placeholders::_1);
    publisher->callbackGetTime = std::bind(&Net2::getTime,this);

    addChannel(publisher);
    return publisher;
  }
  else
  {
    throw std::runtime_error("Init first");
  }
}

std::shared_ptr<Net2Subscriber> Net2::subscriber()
{
  if (state == Net2State::STARTED)
  {
    std::shared_ptr<Net2Subscriber> subscriber = std::make_shared<Net2Subscriber>(context);

    subscriber->callbackAdvertiseService = std::bind(&Net2::advertiseService,this,std::placeholders::_1,std::placeholders::_2);
    subscriber->callbackRemoveService = std::bind(&Net2::removeService,this,std::placeholders::_1);
    subscriber->callbackGetServiceInfo = std::bind(&Net2::getService ,this,std::placeholders::_1);
    subscriber->callbackGetTime = std::bind(&Net2::getTime,this);

    addChannel(subscriber);
    return subscriber;
  }
  else
  {
    throw std::runtime_error("Init first");
  }
}

std::shared_ptr<Net2Service> Net2::service(const std::string &name)
{
  if (state == Net2State::STARTED)
  {
    std::string new_name =  getNameSpace() + "-" + name;
    std::shared_ptr<Net2Service> service = std::make_shared<Net2Service>(new_name,context);

    service->callbackAdvertiseService = std::bind(&Net2::advertiseService,this,std::placeholders::_1,std::placeholders::_2);
    service->callbackRemoveService = std::bind(&Net2::removeService,this,std::placeholders::_1);
    service->callbackGetServiceInfo = std::bind(&Net2::getService ,this,std::placeholders::_1);
    service->callbackGetTime = std::bind(&Net2::getTime,this);

    addChannel(service);
    return service;
  }
  else
  {
    throw std::runtime_error("Init first");
  }
}

std::shared_ptr<Net2Client> Net2::clinet()
{
  if (state == Net2State::STARTED)
  {
    std::shared_ptr<Net2Client> client = std::make_shared<Net2Client>(context);

    client->callbackAdvertiseService = std::bind(&Net2::advertiseService,this,std::placeholders::_1,std::placeholders::_2);
    client->callbackRemoveService = std::bind(&Net2::removeService,this,std::placeholders::_1);
    client->callbackGetServiceInfo = std::bind(&Net2::getService ,this,std::placeholders::_1);
    client->callbackGetTime = std::bind(&Net2::getTime,this);

    addChannel(client);
    return client;
  }
  else
  {
    throw std::runtime_error("Init first");
  }
}

std::shared_ptr<Net2AsyncClient> Net2::asynclient()
{
  if (state == Net2State::STARTED)
  {
    std::shared_ptr<Net2AsyncClient> asynclient = std::make_shared<Net2AsyncClient>(context);

    asynclient->callbackAdvertiseService = std::bind(&Net2::advertiseService,this,std::placeholders::_1,std::placeholders::_2);
    asynclient->callbackRemoveService = std::bind(&Net2::removeService,this,std::placeholders::_1);
    asynclient->callbackGetServiceInfo = std::bind(&Net2::getService ,this,std::placeholders::_1);
    asynclient->callbackGetTime = std::bind(&Net2::getTime,this);

    addChannel(asynclient);
    return asynclient;
  }
  else
  {
    throw std::runtime_error("Init first");
  }
}

void Net2::addChannel(std::shared_ptr<Net2Base> item)
{
  {
    boost::mutex::scoped_lock lock(mutex);
    channels_list.push_back(item);
  }
}

void Net2::Init(const Net2Config &config, const std::string &vname_space, const std::string &vhost_name)
{
  if (state == Net2State::STOPPED)
  {
    this->name_space = vname_space;

    if (vhost_name == "")
    {
      //TODO get station name from host
      throw std::runtime_error("host_name is NULL");
    }
    else
    {
      //std::cout << "NET Initing ... " << std::endl;

      this->net2_config = config;
      this->host_name = vhost_name;

      std::string consul_URL = "http://" + this->net2_config.consul_network_address + ":" + this->net2_config.consul_network_port;

      ROS_INFO_STREAM("Init Net2 ");
      ROS_INFO_STREAM("Consul URL " << consul_URL);

      this->context =  zmq_ctx_new ();
      this->net2_consul = std::make_shared<Net2Consul>(consul_URL,config.consul_mode);
      this->net2_helper = new Net2Helper(this->net2_config,config.local_network_address);

      //Timer
      this->timer = new boost::asio::deadline_timer(io_service);
      this->timer->expires_from_now(boost::posix_time::seconds(1));
      this->timer->async_wait(boost::bind(&Net2::Main_timer_Elapsed,this));
      this->thread = new std::thread(&Net2::boostThread,this);

      ROS_INFO_STREAM("Creating Net2 internal service ");
      std::string new_name = "station-" + getHostName() + "-net2status";

      ROS_INFO_STREAM("Starting Net2 internal service ");

      state = Net2State::STARTED;

      ROS_INFO_STREAM("Init done");

      //std::cout << "NET Initing ... done " << std::endl;
    }
  }
  else
  {
    throw std::runtime_error("Already inited");
  }
}

ProcessResult<ppconsul::ServiceInfo> Net2::getService(std::string name)
{
  ProcessResult<ppconsul::ServiceInfo> result;

  ppconsul::ServiceInfo info = net2_consul->getServiceInfo(name);

  if (info.name != "")
  {
    result.result = info;
    result.success = true;
  }
  else
  {
    result.success = false;
  }

  return result;
}

long Net2::getTime()
{
  return net2_helper->getTime();
}

void Net2::boostThread()
{
  this->io_service.run();
}


void Net2::removeService(const std::string &name)
{
  net2_consul->removeService(name);
}

void Net2::advertiseService(const std::string &name, unsigned int local_port)
{
  std::string station_ip = net2_helper->getStationIp();
  //ROS_INFO_STREAM("AD Station IP " << station_ip << " " << name);
  net2_consul->advertiseService(name, local_port, station_ip, std::to_string(net2_helper->getTime()));
}

void Net2::Main_timer_Elapsed()
{
  if (state == Net2State::STARTED)
  {
    std::string station_ip = net2_helper->getStationIp();
    //ROS_INFO_STREAM("TIMER Station IP " << station_ip << " " << "station-" + getHostName());
    if (station_ip != "")
    {
      net2_consul->advertiseService("station-" + getHostName(), 0, station_ip, std::to_string(net2_helper->getTime()));
    }

    {
      boost::mutex::scoped_lock lock(mutex);

      for ( int i = 0 ; i < channels_list.size() ; i++)
      {
        channels_list.at(i)->update();
      }
    }

  }

  this->timer->expires_from_now(boost::posix_time::seconds(1));
  this->timer->async_wait(boost::bind(&Net2::Main_timer_Elapsed,this));
}

void Net2::Shutdown()
{

  if (state == Net2State::STARTED)
  {
    state = Net2State::STOPPING;

    net2_consul->removeService(getHostName());

    {
      boost::mutex::scoped_lock lock(mutex);

      for (auto item : channels_list)
      {
        item->Stop();
      }

      channels_list.clear();
    }

    if ( net2_consul )
      net2_consul->shutdown();

    state = Net2State::STOPPED;

    host_name = "";
    name_space = "";
  }
  else
  {
    if (state != Net2State::STOPPED)
    {
      throw std::runtime_error("Init first");
    }
  }

}

}
}
}
