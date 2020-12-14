#include "net2_test_ros.hh"
#define Lab

namespace roboland
{

Net2TestROS::Net2TestROS(ros::NodeHandle &nh,
                         ros::NodeHandle &pnh,
                         int argc,
                         char *argv[])
{
  ROS_INFO("NET2 CPP ROS TEST HAS STARTED");

  //Config
  Net2Config config;

#ifdef Lab
  //Home
  config.consul_network_address = "192.168.1.102";
  config.consul_network_mask = "255.255.255.0";
#else
  //Arsam
  config.consul_network_address = "172.25.129.84";
  config.consul_network_mask = "255.255.248.0";
#endif

  config.consul_network_port = "8500";
  config.ntp_server_host_name = "test";
  config.consul_mode = CLIENT;

  net2 = new Net2();

  //Init Net2 with config, namespace and station names
  this->net2->Init(config,"net2cpp","test");

  ROS_WARN("RUN TEST 1 [THE NTP TIME TEST]");

  //long time1 = this->net2->net2_helper->getTime();
  //long time2 = this->net2->net2_helper->syncTime();

  //ROS_INFO_STREAM("Test Clock  RAW-TIME " << time1);
  //ROS_INFO_STREAM("Test Clock SYNC-TIME " << time2);

  //if ( std::abs( time1 - time2 ) < 3 )
  //    ROS_WARN("Test 1 passed");
  //else
   //   ROS_ERROR("Test 1 failed");
//
  //ROS_WARN("==================================");


  this->test_step = 1;

}

std::vector<char> Net2TestROS::callbackData(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  std::vector<char> result;
  ROS_INFO_STREAM("Get callback data " << sender << " " << buffer.size() << " " << priority );
  return result;
}

std::vector<char> Net2TestROS::callbackService(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  std::vector<char> result;
  ROS_INFO_STREAM("Get callback service " << sender << " " << buffer.size() << " " << priority );

  result.push_back('o');
  result.push_back('k');

  return result;
}

void Net2TestROS::update()
{
   if ( this->test_step == 1)
   {
     ROS_WARN("RUN TEST 2 [NET2 Objects]");

     //1
     publisher = net2->publisher("sensor");
     publisher->Start();

     ROS_WARN("Publisher Started");

     //2
     service = net2->service("joint");
     //service->delegateNewData = std::bind(&Net2TestROS::callbackService, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
     service->Start();

     ROS_WARN("Service Started");

     //3
     subscriber = net2->subscriber();
     //subscriber->delegateNewData = std::bind(&Net2TestROS::callbackData, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);

     ROS_WARN("Subscriber Started");

     //4
     async_client = net2->asynclient();
     //async_client->delegateNewData = std::bind(&Net2TestROS::callbackData, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);

     ROS_WARN("Async Client Started");

     //5
     //client = net2->clinet();
     //client->delegateNewData = std::bind(&Net2TestROS::callbackData, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);

     //ROS_WARN("Client Started");

     ROS_WARN("Objects created");
   }
   else if ( this->test_step == 50)
   {
     //ProcessResult<int> result = subscriber->Start("ubuntu-net2cpp-appsensor");
     ProcessResult<int> result = subscriber->Start("Net2Test-sensor");
     ROS_WARN("Subscriber started");
   }
   else if ( this->test_step > 50 && this->test_step < 100)
   {
       if ( this->test_step % 5 == 0 )
       {
         char data[10] = {0};
         publisher->send(data,10,1);
         ROS_WARN("Publisher sent 10 bytes");
       }
   }
   else if ( this->test_step == 100)
   {
     ProcessResult<int> result = async_client->Start("Net2Test-joint");
     ROS_WARN("Async client started");
   }
   else if ( this->test_step > 100 && this->test_step < 150)
   {
     if ( this->test_step % 5 == 0 )
     {
       char data[10] = {0};
       async_client->send(data,10,1);
       ROS_WARN("Async client sent 10 bytes");
     }
   }
   else if ( this->test_step == 150)
   {
     test_step = 0;
     ROS_WARN("TEST 2 finished");
     ROS_WARN("==================================");
   }

   if ( this->test_step != 0 && this->test_step < 150)
   test_step++;

   //if ( test_step % 5 == 0 && test_step != 0)
   //ROS_INFO_STREAM("Step " << test_step);
}

void Net2TestROS::kill()
{
  //Sutdown Net2
  this->net2->Shutdown();

  ROS_INFO("NET2 CPP ROS TEST HAS TERMINATED");
}

Net2TestROS::~Net2TestROS()
{

}

} // namespace roboland
