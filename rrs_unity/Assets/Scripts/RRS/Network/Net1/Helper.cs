using System;
using System.Collections.Generic;
using System.Net;
using System.Net.NetworkInformation;
using System.Text;
using System.Linq;

namespace RRS.Tools.Network
{
    public class NetStatus
    {
        public ulong ping_value;
        public ulong read_time;
        public ulong write_time;
        public ulong index;
    }

    

    static class Helper
    {
        public static DateTime ref_time = new DateTime(2017, 1, 1);

        public static ulong getCurrentTimeSpan()
        {
            return (ulong)(DateTime.Now - ref_time).TotalMilliseconds;
        }

        public static string getNetworkLocalIp(NetworkInterfaceType type,string name)
        {
            List<string> lst = new List<string>();
            foreach (NetworkInterface ni in NetworkInterface.GetAllNetworkInterfaces())
            {
                if (ni.NetworkInterfaceType == type && ni.Name == name)
                {
                    foreach (UnicastIPAddressInformation ip in ni.GetIPProperties().UnicastAddresses)
                    {
                        if (ip.Address.AddressFamily == System.Net.Sockets.AddressFamily.InterNetwork)
                        {
                            return ip.ToString();
                        }                    
                    }
                }
            }

            return "";
        }
    }
}
