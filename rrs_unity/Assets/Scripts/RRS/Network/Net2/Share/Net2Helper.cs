using System;
using System.Collections.Generic;
using System.Net.NetworkInformation;
using System.Text;
using System.Threading.Tasks;

namespace RRS.Tools.Network
{
    public static class Net2Helper
    {
        public static List<string> getAllInterfaceIps()
        {
            List<string> result = new List<string>();

            foreach (NetworkInterface ni in NetworkInterface.GetAllNetworkInterfaces())
            {
                if (ni.NetworkInterfaceType == NetworkInterfaceType.Wireless80211 || ni.NetworkInterfaceType == NetworkInterfaceType.Ethernet)
                {
                    foreach (UnicastIPAddressInformation ip in ni.GetIPProperties().UnicastAddresses)
                    {
                        if (ip.Address.AddressFamily == System.Net.Sockets.AddressFamily.InterNetwork)
                        {
                            string address = ip.Address.ToString();
                            result.Add(address);
                        }
                    }
                }
            }

            return result;
        }

        public static string toBinary(string value)
        {

            int n = int.Parse(value);
            string r = "";

            while (n != 0) { r = (n % 2 == 0 ? "0" : "1") + r; n /= 2; }

            if (r.Length == 0) r = "00000000" + r;
            else if (r.Length == 1) r = "0000000" + r;
            else if (r.Length == 2) r = "000000" + r;
            else if (r.Length == 3) r = "00000" + r;
            else if (r.Length == 4) r = "0000" + r;
            else if (r.Length == 5) r = "000" + r;
            else if (r.Length == 6) r = "00" + r;
            else if (r.Length == 7) r = "0" + r;

            return r;
        }

        public static string toAND(string a,string b)
        {
           string result = "";

            if (a.Length == b.Length)
            {
                for (int i = 0; i < a.Length; i++)
                {
                    if (a[a.Length - i - 1] == '0' || b[a.Length - i - 1] == '0') result += '0';
                    if (a[a.Length - i - 1] == '1' && b[a.Length - i - 1] == '1') result += '1';
                }
            }
            else
            {

            }

            return result;
        }

        public static string getStationIp(string mask, string consul)
        {
            return "192.168.1.101";

            List<string> result = getAllInterfaceIps();

            //Example
            // 1111 1000 0000 0000 (0.0.248.0)   [mask]

            // 1000 0000 1111 1101 (0.0.128.123) [cosul]
            // 1000 0000 1011 1010 (0.0.128.186) [desire valid]
            // 1000 1100 1100 1010 (0.0.140.202) [desire invalid]

            // [consul] AND [mask] =         1000 0000
            // [desire valid] AND [mask] =   1000 0000
            // [desire invalid] AND [mask] = 1000 1000

            foreach (var item in result)
            {
                //Compare Mask with Network start to get the desire station ip
                //Binary AND compare
                //Return the first valid ip

                string desire = item;

                string[] strs_desire;
                strs_desire = desire.Split('.');

                string[] strs_consul;
                strs_consul = consul.Split('.');

                string[] strs_mask;
                strs_mask = mask.Split('.');

                string bin_desire = toBinary(strs_desire[0]) + toBinary(strs_desire[1]) + toBinary(strs_desire[2]) + toBinary(strs_desire[3]);
                string bin_consul = toBinary(strs_consul[0]) + toBinary(strs_consul[1]) + toBinary(strs_consul[2]) + toBinary(strs_consul[3]);
                string bin_mask = toBinary(strs_mask[0]) + toBinary(strs_mask[1]) + toBinary(strs_mask[2]) + toBinary(strs_mask[3]);

                string and_desire = toAND(bin_desire, bin_mask);
                string and_consul = toAND(bin_consul, bin_mask);

                //    ROS_INFO_STREAM("Consul : " << bin_consul);
                //    ROS_INFO_STREAM("Desire : " << bin_desire);
                //    ROS_INFO_STREAM("mask   : " << bin_mask);
                //    ROS_INFO_STREAM("mask d : " << and_desire);
                //    ROS_INFO_STREAM("mask c : " << and_consul);

                if (and_consul == and_desire)
                {
                    return desire;
                }
            }

            return "";
        }
    }
}
