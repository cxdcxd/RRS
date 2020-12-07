#include "rrs_net_test.hh"

namespace roboland
{


  RRSNetTest::RRSNetTest(ros::NodeHandle &nh,
   ros::NodeHandle &pnh,
   int argc,
   char *argv[])
  {
  	// if (__cplusplus == 201703L) std::cout << "C++17\n";
    //  else if (__cplusplus == 201402L) std::cout << "C++14\n";
    //  else if (__cplusplus == 201103L) std::cout << "C++11\n";
    //  else if (__cplusplus == 199711L) std::cout << "C++98\n";
    //  else std::cout << "pre-standard C++\n";

    ROS_INFO("RRS Network Test V 1.0");

    std::string path = ros::package::getPath("rrs_ros");
    path = path + "/config/config.yaml";

    config_path = path;

    ROS_INFO_STREAM("Config path is : " << config_path);

    //Creating the config file for first use or load it 
    loadYaml();
    saveYaml();

    //Config Print
    ROS_INFO_STREAM("consul network address :" << m_settings.consul_network_address );
    ROS_INFO_STREAM("Local network address :" << m_settings.local_network_address );
    ROS_INFO_STREAM("consul network mask :" << m_settings.consul_network_mask );
    ROS_INFO_STREAM("consul network port :" << m_settings.consul_network_port);
    ROS_INFO_STREAM("ntp server host name : " << m_settings.ntp_server_host_name);

    //Config
    Net2Config config;

    config.consul_network_address = m_settings.consul_network_address;
    config.consul_network_mask = m_settings.consul_network_mask;
    config.consul_network_port = m_settings.consul_network_port;
    config.ntp_server_host_name = m_settings.ntp_server_host_name;
    config.local_network_address = m_settings.local_network_address;
    config.consul_mode = CLIENT;

    net2 = new Net2();

    //Init Net2 with config, namespace and station names
    this->net2->Init(config,"rrs_ros","test");

    // //RRS High Performance Bridge
    for ( int i = 0 ; i < sub_count ; i++)
    {
       subscribers[i] = net2->subscriber();
       string name = "rrs-pub" + std::to_string(i+1); 
       ProcessResult<int> result = subscribers[i]->Start(name.c_str());
    }

    for ( int i = 0 ; i < pub_count ; i++)
    {
      string name = "pub" + std::to_string(i+1);
      publishers[i] = net2->publisher(name.c_str());
      publishers[i]->Start(); 
    }

    subscribers[0]->delegateNewData = std::bind(&RRSNetTest::callbackDataSub1,this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    subscribers[1]->delegateNewData = std::bind(&RRSNetTest::callbackDataSub2,this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    subscribers[2]->delegateNewData = std::bind(&RRSNetTest::callbackDataSub3,this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    subscribers[3]->delegateNewData = std::bind(&RRSNetTest::callbackDataSub4,this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    subscribers[4]->delegateNewData = std::bind(&RRSNetTest::callbackDataSub5,this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    //subscribers[5]->delegateNewData = std::bind(&RRSNetTest::callbackDataSub6,this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    //subscribers[6]->delegateNewData = std::bind(&RRSNetTest::callbackDataSub7,this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    //subscribers[7]->delegateNewData = std::bind(&RRSNetTest::callbackDataSub8,this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    //subscribers[8]->delegateNewData = std::bind(&RRSNetTest::callbackDataSub9,this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    //subscribers[9]->delegateNewData = std::bind(&RRSNetTest::callbackDataSub10,this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    
  }

  bool RRSNetTest::loadYaml()
  {
    try
    {
      if ( is_file_exist(config_path.c_str()) )
      {
       m_config = YAML::LoadFile(config_path.c_str());

       m_settings.consul_network_address = m_config["consul_network_address"].as<std::string>();
       m_settings.local_network_address = m_config["local_network_address"].as<std::string>();
       m_settings.consul_network_mask = m_config["consul_network_mask"].as<std::string>();
       m_settings.consul_network_port = m_config["consul_network_port"].as<std::string>();
       m_settings.ntp_server_host_name = m_config["ntp_server_host_name"].as<std::string>();

       return true;
     }
     else
     {
      ROS_WARN("Config file is not exist");
    }
  }
  catch (...)
  {
   ROS_ERROR("Could not parse YAML config, or not exist");
 }

 return false;
}

bool RRSNetTest::is_file_exist(const char *fileName)
{
  std::ifstream infile(fileName);
  return infile.good();
}

bool RRSNetTest::saveYaml()
{
  try
  {

    std::ofstream fout(config_path.c_str()); 

    m_config["consul_network_address"] = m_settings.consul_network_address;
    m_config["local_network_address"] = m_settings.local_network_address;
    m_config["consul_network_mask"] = m_settings.consul_network_mask;
    m_config["consul_network_port"] = m_settings.consul_network_port;
    m_config["ntp_server_host_name"] = m_settings.ntp_server_host_name;


    fout << m_config; 

    fout.flush();
    fout.close();

    return true;

  }
  catch (...)
  {
    ROS_ERROR("Could not save YAML config");
  }

  return false;
}

void RRSNetTest::send(int index)
{
 RVector3 proto_msg;
 proto_msg.set_x(100);
 proto_msg.set_y(200);
 proto_msg.set_theta(300);

 int size = proto_msg.ByteSize();
 char buffer[size];
 proto_msg.SerializeToArray(buffer,size);

 publishers[index]->send(buffer,size,1);
}

std::vector<char> RRSNetTest::callbackDataSub1(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  std::vector<char> result;
  //ROS_WARN("GET SUB IN FUNCTION CALLBACK");

  if ( priority == 10 ) return result;

  SVector3 msg;
  msg.ParseFromArray(&buffer[0],buffer.size());

  ROS_INFO_STREAM("Get data " << sender << " " << msg.x() << " " << msg.y() << " " << msg.z() );

  return result;
}

std::vector<char> RRSNetTest::callbackDataSub2(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  std::vector<char> result;
  //ROS_WARN("GET SUB IN FUNCTION CALLBACK");

  if ( priority == 10 ) return result;

  SVector3 msg;
  msg.ParseFromArray(&buffer[0],buffer.size());

  ROS_INFO_STREAM("Get data " << sender << " " << msg.x() << " " << msg.y() << " " << msg.z() );

  return result;
}

std::vector<char> RRSNetTest::callbackDataSub3(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  std::vector<char> result;
  //ROS_WARN("GET SUB IN FUNCTION CALLBACK");

  if ( priority == 10 ) return result;

  SVector3 msg;
  msg.ParseFromArray(&buffer[0],buffer.size());

  ROS_INFO_STREAM("Get data " << sender << " " << msg.x() << " " << msg.y() << " " << msg.z() );

  return result;
}

std::vector<char> RRSNetTest::callbackDataSub4(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  std::vector<char> result;
  //ROS_WARN("GET SUB IN FUNCTION CALLBACK");

  if ( priority == 10 ) return result;

  SVector3 msg;
  msg.ParseFromArray(&buffer[0],buffer.size());

  ROS_INFO_STREAM("Get data " << sender << " " << msg.x() << " " << msg.y() << " " << msg.z() );

  return result;
}

std::vector<char> RRSNetTest::callbackDataSub5(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  std::vector<char> result;
  //ROS_WARN("GET SUB IN FUNCTION CALLBACK");

  if ( priority == 10 ) return result;

  SVector3 msg;
  msg.ParseFromArray(&buffer[0],buffer.size());

  ROS_INFO_STREAM("Get data " << sender << " " << msg.x() << " " << msg.y() << " " << msg.z() );

  return result;
}

std::vector<char> RRSNetTest::callbackDataSub6(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  std::vector<char> result;
  //ROS_WARN("GET SUB IN FUNCTION CALLBACK");

  if ( priority == 10 ) return result;

  SVector3 msg;
  msg.ParseFromArray(&buffer[0],buffer.size());

  ROS_INFO_STREAM("Get data " << sender << " " << msg.x() << " " << msg.y() << " " << msg.z() );

  return result;
}

std::vector<char> RRSNetTest::callbackDataSub7(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  std::vector<char> result;
  //ROS_WARN("GET SUB IN FUNCTION CALLBACK");

  if ( priority == 10 ) return result;

  SVector3 msg;
  msg.ParseFromArray(&buffer[0],buffer.size());

  ROS_INFO_STREAM("Get data " << sender << " " << msg.x() << " " << msg.y() << " " << msg.z() );

  return result;
}

std::vector<char> RRSNetTest::callbackDataSub8(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  std::vector<char> result;
  //ROS_WARN("GET SUB IN FUNCTION CALLBACK");

  if ( priority == 10 ) return result;

  SVector3 msg;
  msg.ParseFromArray(&buffer[0],buffer.size());

  ROS_INFO_STREAM("Get data " << sender << " " << msg.x() << " " << msg.y() << " " << msg.z() );

  return result;
}

std::vector<char> RRSNetTest::callbackDataSub9(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  std::vector<char> result;
  //ROS_WARN("GET SUB IN FUNCTION CALLBACK");

  if ( priority == 10 ) return result;

  SVector3 msg;
  msg.ParseFromArray(&buffer[0],buffer.size());

  ROS_INFO_STREAM("Get data " << sender << " " << msg.x() << " " << msg.y() << " " << msg.z() );

  return result;
}

std::vector<char> RRSNetTest::callbackDataSub10(std::vector<char> buffer, unsigned int priority, std::string sender)
{
  std::vector<char> result;
  //ROS_WARN("GET SUB IN FUNCTION CALLBACK");

  if ( priority == 10 ) return result;

  SVector3 msg;
  msg.ParseFromArray(&buffer[0],buffer.size());

  ROS_INFO_STREAM("Get data " << sender << " " << msg.x() << " " << msg.y() << " " << msg.z() );

  return result;
}


void RRSNetTest::update()
{
   for ( int i = 0 ; i < pub_count ; i++)
   {
    send(i);
   }
}

void RRSNetTest::kill()
{
  for ( int i = 0 ; i < pub_count ; i++)
  this->publishers[i]->Stop();

  for ( int i = 0 ; i < sub_count ; i++)
  this->subscribers[i]->Stop();

  this->net2->Shutdown();
  ROS_INFO("RRS NET TEST TERMINATED");
}

RRSNetTest::~RRSNetTest()
{
}

} // namespace roboland
