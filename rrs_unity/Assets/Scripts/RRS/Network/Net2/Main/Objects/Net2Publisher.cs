using NetMQ.Monitoring;
using NetMQ.Sockets;
using NetMQ;

using RRS.Tools.Log;
using RRS.Tools.Protobuf;
using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;
using ProtoBuf;
using System.Threading;

namespace RRS.Tools.Network
{
    public partial class Net2
    {
        private class Net2Publisher : Net2Base
        {
            public Net2Publisher(string topic_name) : base()
            {
                this.name = topic_name;
            }

            public void Send(byte[] data, uint priority = 0)
            {
                if (internal_state == Net2State.STARTED)
                {
                    var msg = createMessage(data, priority);
                    addToSendQueue(msg);
                }

                last_send_receive_time = GetTime();
            }

            protected override void _SocketBuilder()
            {
                reportLog("Starting publisher " + Name, LogType.INFO, section);

                socket = new PublisherSocket();
              

                socket.Options.TcpKeepalive = true;
                socket.Options.TcpKeepaliveIdle = TimeSpan.FromMilliseconds(100);
                socket.Options.TcpKeepaliveInterval = TimeSpan.FromMilliseconds(100);

                local_port = socket.BindRandomPort("tcp://*");

                //monitor = new NetMQMonitor(socket, "inproc://pub" + Net2.GetAddressIndex(), SocketEvents.All);

                //monitor.Disconnected += Monitor_Disconnected;
                //monitor.Accepted += Monitor_Connected;
                //monitor.Timeout = TimeSpan.FromMilliseconds(100);
                //task = monitor.StartAsync();

                setState(Net2State.STARTED);

                start_time = GetTime();

                reportLog("Publisher " + Name + " is ready on " + local_port, LogType.INFO, section);
            }

            protected override void _Receive()
            {

            }

            protected override void _CheckState()
            {
                if (internal_state == Net2State.STARTED)
                {
                    long diff = GetTime() - last_send_receive_time;
                    if (diff > 2000)
                    {
                        Send(new byte[1], 10);
                    }
                }
            }

            protected override void threadExitProcess()
            {
                setState(Net2State.STOPPED);
            }

            protected override void reportData(Message packet)
            {
                
            }

            protected override void StopMonitor()
            {
                //if (monitor != null)
                //{
                //    monitor.Disconnected -= Monitor_Disconnected;
                //    monitor.Accepted -= Monitor_Connected;
                //    monitor.Stop();
                //}
            }
        }
    }
}
