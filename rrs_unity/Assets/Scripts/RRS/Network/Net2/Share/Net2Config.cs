using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RRS.Tools.Network
{
    public class Net2Config
    {
        public string name_space;
        public string consul_network_address;
        public string consul_network_mask;
        public uint consul_network_port;
        public string ntp_server_host_name;
        public uint ntp_server_port = 123;
        public string local_network_address;
        public Net2ConsulMode consul_mode = Net2ConsulMode.CLIENT;
    };
}
