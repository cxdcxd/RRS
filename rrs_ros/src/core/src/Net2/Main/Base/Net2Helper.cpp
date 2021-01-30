#include "Net2/Net2Helper.h"

namespace lmt
{
namespace Tools
{
namespace Network
{
Net2Helper::Net2Helper(const Net2Config &config, std::string settings_local_ip)
{
  this->net2_config = config;
  this->local_ip = settings_local_ip;
  
  //NTP Sync
  ntp_client = new NTPClient();
  ntp_client->sync(this->net2_config.ntp_server_host_name,this->net2_config.ntp_server_port);

  std::cout << "NTP sync successful" << std::endl;
  std::cout << "Time : "  << getTime() << std::endl;
}

Net2Helper::~Net2Helper()
{

}

std::vector<std::string> Net2Helper::getAllInterfaceIps()
{
  std::vector<std::string> result;

  setenv("LANG","C",1);
  FILE * fp = popen("ifconfig", "r");

  //Updated for Ubuntu 18.0

  if (fp)
  {
    char *p=NULL, *e; size_t n;
    while ((getline(&p, &n, fp) > 0) && p)
    {
      if (p = strstr(p, "inet "))
      {
          p += 5;
          if (e = strchr(p, ' '))
          {
            *e='\0';

            //ROS_INFO_STREAM("Line " << p);
            //printf("%s\n", p);
            result.push_back(p);
          }
        
      }
    }
  }
  else
  {
    ROS_ERROR("ifconfig station ip failed");
  }

  pclose(fp);

  return result;
}

std::string Net2Helper::toBinary(std::string value)
{
  int n = std::stoi(value);
  std::string r;

  while(n!=0) {r=(n%2==0 ?"0":"1")+r; n/=2;}

  if      (r.size()==0) r = "00000000" + r;
  else if (r.size()==1) r = "0000000" + r;
  else if (r.size()==2) r = "000000" + r;
  else if (r.size()==3) r = "00000" + r;
  else if (r.size()==4) r = "0000" + r;
  else if (r.size()==5) r = "000" + r;
  else if (r.size()==6) r = "00" + r;
  else if (r.size()==7) r = "0" + r;

  return r;
}

std::string Net2Helper::toAND(std::string a,std::string b)
{
  std::string result = "";

  if ( a.size() == b.size())
  {
    for( int i = 0 ; i < a.size() ; i++)
    {
      if ( a[a.size() - i - 1] == '0' || b[a.size() - i - 1] == '0') result += '0';
      if ( a[a.size() - i - 1] == '1' && b[a.size() - i - 1] == '1') result += '1';
    }
  }
  else
  {

  }

  return result;
}

std::string Net2Helper::getStationIp()
{
  return local_ip;
  
  std::vector<std::string> result = getAllInterfaceIps();

  std::string mask = this->net2_config.consul_network_mask;
  std::string consul = this->net2_config.consul_network_address;

  //Example
  // 1111 1000 0000 0000 (0.0.248.0)   [mask]

  // 1000 0000 1111 1101 (0.0.128.123) [cosul]
  // 1000 0000 1011 1010 (0.0.128.186) [desire valid]
  // 1000 1100 1100 1010 (0.0.140.202) [desire invalid]

  // [consul] AND [mask] =         1000 0000
  // [desire valid] AND [mask] =   1000 0000
  // [desire invalid] AND [mask] = 1000 1000

  //ROS_INFO_STREAM(result.size() << " " << "Interface Size");

  for (auto item : result)
  {
    //Compare Mask with Network start to get the desire station ip
    //Binary AND compare
    //Return the first valid ip

    std::string desire = item;

    std::vector<std::string> strs_desire;
    boost::split(strs_desire,desire,boost::is_any_of("."));

    std::vector<std::string> strs_consul;
    boost::split(strs_consul,consul,boost::is_any_of("."));

    std::vector<std::string> strs_mask;
    boost::split(strs_mask,mask,boost::is_any_of("."));

    std::string bin_desire = toBinary(strs_desire[0]) + toBinary(strs_desire[1]) + toBinary(strs_desire[2]) + toBinary(strs_desire[3]);
    std::string bin_consul = toBinary(strs_consul[0]) + toBinary(strs_consul[1]) + toBinary(strs_consul[2]) + toBinary(strs_consul[3]);
    std::string bin_mask   = toBinary(strs_mask[0]  ) + toBinary(strs_mask[1]  ) + toBinary(strs_mask[2]  ) + toBinary(strs_mask[3]);

    std::string and_desire = toAND(bin_desire,bin_mask);
    std::string and_consul = toAND(bin_consul,bin_mask);

        //ROS_INFO_STREAM("Consul : " << bin_consul);
        //ROS_INFO_STREAM("Desire : " << bin_desire);
        //ROS_INFO_STREAM("mask   : " << bin_mask);
        //ROS_INFO_STREAM("mask d : " << and_desire);
        //ROS_INFO_STREAM("mask c : " << and_consul);

    if ( and_consul == and_desire )
    {
      return desire;
    }
  }

  return "";
}

long Net2Helper::getTime()
{
  return ntp_client->get();
}

long Net2Helper::syncTime()
{
  return ntp_client->sync(this->net2_config.ntp_server_host_name,this->net2_config.ntp_server_port);
}
}
}
}
