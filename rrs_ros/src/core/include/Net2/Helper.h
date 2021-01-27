#pragma once

#include <string>
#include <vector>
#include <memory>


namespace lmt
{
	namespace Tools
	{
		namespace Network
		{
			class NetStatus : public std::enable_shared_from_this<NetStatus>
			{
			public:
				unsigned long long ping_value = 0;
				unsigned long long read_time = 0;
				unsigned long long write_time = 0;
				unsigned long long index = 0;
			};



			class Helper : public std::enable_shared_from_this<Helper>
			{
			public:
				static DateTime ref_time;

				static unsigned long long getCurrentTimeSpan();

				static std::wstring getNetworkLocalIp(NetworkInterfaceType type, const std::wstring &name);
			};
		}
	}
}
