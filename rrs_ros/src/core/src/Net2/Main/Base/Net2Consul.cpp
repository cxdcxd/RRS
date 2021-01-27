#include "Net2/Net2Consul.h"
#include "Net2/AsyncRunner.h"
#include "Net2/Enum.h"
#include "Net2/ProcessResult.h"

using ppconsul::Consul;

namespace lmt
{
namespace Tools
{
namespace Network
{

Net2Consul::Net2Consul(const std::string &uri, Net2ConsulMode mode)
{
  ROS_INFO("Net2Consul started");

  consul_mode = mode;

  consul = new Consul(uri);
  agent = new Agent(*consul);

  is_thread_exit = false;
  thread = new std::thread(&Net2Consul::mThread,this);
}

void Net2Consul::msdelay(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

ppconsul::ServiceInfo Net2Consul::getServiceInfo(const std::string &name)
{
  ppconsul::ServiceInfo info;
  info.id = "";

  {
    boost::mutex::scoped_lock (mutex_get);

    std::vector<std::string> names;
    boost::split(names,name,boost::is_any_of("-"));

    for (int i = 0; i < updated_list.size(); i++)
    {
      if ( updated_list.at(i).name == name)
      {
        info = updated_list.at(i);
        break;
      }
    }

    if ( info.id != "")
    {
      for (int i = 0; i < updated_list.size(); i++)
      {
        std::vector<std::string> target_names;
        boost::split(target_names,updated_list.at(i).id ,boost::is_any_of("-"));

        if ( target_names.size() == 2)
        {
          if (target_names[0] == "station" && target_names[1] == names[0])
          {
             //ROS_INFO_STREAM("Target " << target_names[0] << " " << target_names[1] << " " << names[0]);
             //ROS_INFO_STREAM("Address " << updated_list.at(i).address);
             info.address = updated_list.at(i).address;
             break;
          }
        }
      }
    }

  }

  return info;
}

void Net2Consul::shutdown()
{
  this->is_thread_exit = true;
  thread->join();
  //ROS_WARN("Net2Consul Shutdowned");
}

void Net2Consul::mThread()
{
  msdelay(1000);

  //ROS_INFO_STREAM( "Consul Thread " << is_thread_exit);

  std::map<std::string, ppconsul::ServiceInfo> list;

  while(is_thread_exit == false)
  {
    //ROS_WARN_STREAM( "Loop mode " << this->consul_mode);

    if ( this->consul_mode == CLIENT)
    {
      list = agent->services();

      {
        boost::mutex::scoped_lock (mutex_get);

        updated_list.clear();

        for (auto& it: list)
        {
          updated_list.push_back(it.second);
        }
      }

      //ROS_WARN_STREAM("done");
      msdelay(1000);

       {
         boost::mutex::scoped_lock (mutex_send);

        //ROS_WARN_STREAM("Sending Queue " << sending_queue.size() );

        for (int i = 0; i < sending_queue.size(); i++)
        {
          
          ppconsul::ServiceInfo info = sending_queue.at(i);
          
          //ROS_WARN_STREAM("advertise for index " << i << " " << info.name << " " << info.address);

          agent->registerService
              (
                kw::name =  info.name,
                kw::id =  info.name,
                kw::address = info.address,
                kw::port = info.port,
                kw::tags = info.tags
              );
        }
      }
      
    }
    else if ( this->consul_mode == MANAGER )
    {

      list = agent->services();

      {
        boost::mutex::scoped_lock (mutex_get);

        updated_list.clear();

        for (auto& it: list)
        {
          updated_list.push_back(it.second);
        }
      }

      msdelay(4000);

      list = agent->services();

      {
        boost::mutex::scoped_lock (mutex_get);

        temp_updated_list.clear();

        for (auto& it: list)
        {
          temp_updated_list.push_back(it.second);
        }

        for ( int i = 0; i < updated_list.size() ; i++)
        {
          std::string target_name = updated_list.at(i).name;

          for (int j = 0; j < temp_updated_list.size() ; j++)
          {
            if (temp_updated_list.at(j).name == target_name && target_name != "station-manager-net2status" && target_name != "station-manager")
            {

              std::string a = *std::next(updated_list.at(i).tags.begin(), 0);
              std::string b = *std::next(temp_updated_list.at(j).tags.begin(), 0);

              //ROS_INFO_STREAM( "compare " << a);
              //ROS_INFO_STREAM( "compare " << b);

              //ROS_INFO_STREAM("A " << a << "B " << b);

              long old_time = std::stol(a);
              long new_time = std::stol(b);

              long diff = new_time - old_time;

              if (diff == 0)
              {
                ROS_WARN_STREAM("Service is deleted : " << target_name << " " << diff );
                agent->deregisterService(target_name);
                
              }
              else
              {
                //ROS_INFO_STREAM("Service is ok : " << target_name << " " << diff );
              }


            }
          }
        }
      }

      msdelay(1000);

      {
        boost::mutex::scoped_lock (mutex_send);
        for (int i = 0; i < sending_queue.size(); i++)
        {
          ppconsul::ServiceInfo info = sending_queue.at(i);

          agent->registerService
              (
                kw::name =  info.name,
                kw::id =  info.name,
                kw::address = info.address,
                kw::port = info.port,
                kw::tags = info.tags
              );
        }
      }
    }
  }
}

void Net2Consul::advertiseService(const std::string &name, unsigned int port,const std::string &address,std::string time)
{
  {
    boost::mutex::scoped_lock (mutex_send);
    int index = -1;
    for (int i = 0; i < sending_queue.size(); i++)
    {
      if ( sending_queue.at(i).name == name)
      {
        index = i;
        break;
      }
    }

    if ( index == -1)
    {

      ppconsul::ServiceInfo info;
      info.name = name;
      info.id = name;
      info.port = port;
      info.address = address;
      info.tags = {"time",time};

      sending_queue.push_back(info);
    }
    else
    {
       sending_queue.at(index).tags = {"time",time};
    }
  }
}

void Net2Consul::removeService(const std::string &name)
{
  {
    boost::mutex::scoped_lock (mutex_send);

    int index = -1;
    for (int i = 0; i < sending_queue.size(); i++)
    {
      if ( sending_queue.at(i).name == name)
      {
        index = i;
        break;
      }
    }

    if ( index != -1)
      sending_queue.erase(sending_queue.begin() + index);
  }
}

}
}
}
