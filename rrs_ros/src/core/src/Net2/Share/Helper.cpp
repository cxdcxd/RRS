#include "Helper.h"

namespace lmt
{
	namespace Tools
	{
		namespace Network
		{

            DateTime Helper::ref_time = DateTime(2017, 1, 1);

			unsigned long long Helper::getCurrentTimeSpan()
			{
				return static_cast<unsigned long long>((DateTime::Now - ref_time).TotalMilliseconds);
			}

			std::wstring Helper::getNetworkLocalIp(NetworkInterfaceType type, const std::wstring &name)
			{
				std::vector<std::wstring> lst;
				for (auto ni : NetworkInterface::GetAllNetworkInterfaces())
				{
					if (ni->NetworkInterfaceType == type && ni->Name == name)
					{
						for (auto ip : ni->GetIPProperties()->UnicastAddresses)
						{
							if (ip->Address->AddressFamily == System::Net::Sockets::AddressFamily::InterNetwork)
							{
//C# TO C++ CONVERTER TODO TASK: There is no native C++ equivalent to 'ToString':
								return ip->ToString();
							}
						}
					}
				}

				return L"";
			}
		}
	}
}
