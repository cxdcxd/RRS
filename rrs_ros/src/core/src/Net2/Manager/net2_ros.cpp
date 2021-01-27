#include "net2_ros.hh"
#define Lab

namespace lmt
{

	Net2ROS::Net2ROS(ros::NodeHandle &nh,
		ros::NodeHandle &pnh,
		int argc,
		char *argv[])
	{
		ROS_INFO("NET2 manager 1.0 started");

		std::string path = ros::package::getPath("rrs_ros");
		path = path + "/cfg/config.yaml";

		config_path = path;

		ROS_INFO_STREAM("Config path is : " << config_path);

        //Creating the config file for first use or load it 
		loadYaml();
		saveYaml();

        //Config Print
		ROS_INFO_STREAM("consul network address :" << m_settings.consul_network_address );
		ROS_INFO_STREAM("consul network mask :" << m_settings.consul_network_mask );
		ROS_INFO_STREAM("consul network port :" << m_settings.consul_network_port);
		ROS_INFO_STREAM("ntp server host name : " << m_settings.ntp_server_host_name);

        //Config
		Net2Config config;

		config.consul_network_address = m_settings.consul_network_address;
		config.consul_network_mask = m_settings.consul_network_mask;
		config.consul_network_port = m_settings.consul_network_port;
		config.ntp_server_host_name =  m_settings.ntp_server_host_name;
		config.consul_mode = MANAGER;

		net2 = new Net2();

         //Init Net2 with config, namespace and station names
		this->net2->Init(config,"net2manager","manager");

		ROS_INFO("NET2 init done");
	}

	void Net2ROS::kill()
	{
        //Sutdown Net2
		ROS_INFO("NET2 manager shutting down");

		this->net2->Shutdown();

		ROS_INFO("NET2 manager has terminated");
	}

	bool Net2ROS::loadYaml()
	{
		try
		{
			if ( is_file_exist(config_path.c_str()) )
			{
				m_config = YAML::LoadFile(config_path.c_str());

				m_settings.consul_network_address = m_config["consul_network_address"].as<std::string>();
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

	bool Net2ROS::is_file_exist(const char *fileName)
	{
		std::ifstream infile(fileName);
		return infile.good();
	}

	bool Net2ROS::saveYaml()
	{
		try
		{

			std::ofstream fout(config_path.c_str()); 

			m_config["consul_network_address"] = m_settings.consul_network_address;
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

	void Net2ROS::update()
	{

	}

	Net2ROS::~Net2ROS()
	{

	}

} // namespace lmt
