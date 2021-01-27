#pragma once

#include <string>

namespace lmt
{
	namespace Tools
	{
		namespace Network
		{
			enum class Net2DataRateUnit
			{
				B,
				KB,
				MB,
				GB,
				TB,
				PB
			};

			enum class Net2State
			{
				STOPPED,
				STOPPING,
				STARTED,
				STARTING,
				CONNECTED,
				BUSY
			};

			enum class Net2Direction
			{
				INPUT,
				OUTPUT
			};

			enum class Net2NetworkType
			{
				CLIENT,
				PUBLISHER,
				SUBSCRIBER,
				SERVICE,
				ASYNCCLIENT,
			};

			enum LogType
            {
                INFO,
                DEBUG,
                WARN,
                ERROR
            };

            enum Net2ConsulMode
            {
              CLIENT,
              MANAGER
            };

            struct Net2Config
            {
              std::string consul_network_address;
	      std::string local_network_address;
              std::string consul_network_mask;
              std::string consul_network_port;
              std::string ntp_server_host_name;
              int ntp_server_port = 123;
              Net2ConsulMode consul_mode;
              double price;
            };
		}
	}
}
