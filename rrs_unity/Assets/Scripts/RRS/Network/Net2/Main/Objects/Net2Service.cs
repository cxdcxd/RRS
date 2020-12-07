﻿/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    Autor: Edwin Babaians
    Organization: www.Arsamrobotics.com
    
    Release Note:

    Version 1.0.0
    - first initial release

 */

using NetMQ;
using NetMQ.Monitoring;
using NetMQ.Sockets;
using ProtoBuf;
using RRS.Tools.Log;
using RRS.Tools.Protobuf;
using System;
using System.Collections.Generic;
using System.Net.NetworkInformation;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Timers;
using Consul;

namespace RRS.Tools.Network
{
    public partial class Net2
    {
        private class Net2Service : Net2Base
        {
            DelegateGetResponse GetResponse;

            public Net2Service(string service_name, DelegateGetResponse response_callback) : base()
            {
                GetResponse = response_callback;
                this.name = service_name;
            }

            ~Net2Service()
            {

            }

            protected override void _CheckState()
            {

            }

            protected override void threadExitProcess()
            {
                setState(Net2State.STOPPED);
            }

            protected override void _SocketBuilder()
            {
                reportLog("Starting Router name " + Name, LogType.INFO, section);

                socket = new RouterSocket();

                socket.Options.TcpKeepalive = true;
                socket.Options.TcpKeepaliveIdle = TimeSpan.FromMilliseconds(100);
                socket.Options.TcpKeepaliveInterval = TimeSpan.FromMilliseconds(100);

                local_port = socket.BindRandomPort("tcp://*");

                monitor = new NetMQMonitor(socket, "inproc://router" + Net2.GetAddressIndex(), SocketEvents.All);
                monitor.Disconnected += Monitor_Disconnected;
                monitor.Accepted += Monitor_Connected;
                monitor.Timeout = TimeSpan.FromMilliseconds(100);
                task = monitor.StartAsync();

                setState(Net2State.STARTED);
                start_time = GetTime();

                reportLog("Router is ready on " + local_port, LogType.INFO, section);
            }

            protected override void reportData(Message packet)
            {
                byte[] result = GetResponse(packet.header.sequence, packet.payload, packet.header.priority, Id);

                if (result != null)
                {
                    Message ack_message = new Message();
                    ack_message.header = new Header();

                    ack_message.payload = result;
                    ack_message.header.sequence = packet.header.sequence;
                    ack_message.header.mode = Header.Mode.ACK;
                    ack_message.header.message_type = Header.Type.ROUTER;
                    ack_message.header.time_span = GetTime();
                    ack_message.header.zmq_router_address = packet.header.zmq_router_address;

                    addToSendQueue(ack_message);
                }
            }

            protected override void StopMonitor()
            {
                if (monitor != null)
                {
                    monitor.Disconnected -= Monitor_Disconnected;
                    monitor.Accepted -= Monitor_Connected;
                    monitor.Stop();
                }
            }

            public DelegateGetResponse ResponseCallback { get { return GetResponse; } }
        }
    }
}
