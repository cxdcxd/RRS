using RRS.Tools.Log;
using RRS.Tools.Protobuf;
using System;
using System.Collections.Generic;
using System.Text;
using NetMQ;
using NetMQ.Sockets;
using NetMQ.Monitoring;
using ProtoBuf;
using System.Threading;
using System.Threading.Tasks;
using Consul;

namespace RRS.Tools.Network
{
    public partial class Net2
    {
        private class Net2Subscriber : Net2Base
        {
            public delegate ProcessResult DelegateGetServiceInfo(string name);
            public event DelegateGetServiceInfo delegateGetServiceInfo;

            public delegate void DelegateNewData(long sequence, byte[] buffer, uint priority, ulong id);
            public event DelegateNewData delegateNewData;

            AgentService info;

            public Net2Subscriber(string topic_name) : base()
            {
                this.name = topic_name;
            }

            protected ProcessResult getServiceInfo(string name)
            {
                ProcessResult result = new ProcessResult();

                if (delegateGetServiceInfo != null)
                {
                    return delegateGetServiceInfo(name);
                }
                else
                {
                    result.Success = false;
                    result.Result = null;
                    result.ResultType = null;
                }

                return result;
            }

            public AgentService GetAddress(string name)
            {
                ProcessResult result = getServiceInfo(name);

                if (result.Success && result.Result != null && result.Result is AgentService)
                {
                    return result.Result as AgentService;
                }
                else
                {
                    return null;
                }
            }

            protected override void _SocketBuilder()
            {
                if (info == null)
                {
                    info = GetAddress(name);
                    if (info == null)
                    {
                        Thread.Sleep(900);
                        return;
                    }
                }
                reportLog("Starting subscriber " + Name, LogType.INFO, section);

                socket = new SubscriberSocket();

                socket.Options.TcpKeepalive = true;
                socket.Options.TcpKeepaliveIdle = TimeSpan.FromMilliseconds(100);
                socket.Options.TcpKeepaliveInterval = TimeSpan.FromMilliseconds(100);

                string path = "tcp://" + info.Address + ":" + info.Port;

                ((SubscriberSocket)socket).Subscribe("");

                //monitor = new NetMQMonitor(socket, "inproc://sub" + Net2.GetAddressIndex(), SocketEvents.All);

                //monitor.Disconnected += Monitor_Disconnected;
                //monitor.Connected += Monitor_Connected;

                //monitor.Timeout = TimeSpan.FromMilliseconds(100);
                //task = Task.Factory.StartNew(monitor.Start);

                setState(Net2State.STARTED);
                start_time = GetTime();

                socket.Connect(path);

                reportLog("Subscriber " + Name + " is connecting to " + path, LogType.INFO, section);
            }

            protected override void _CheckState()
            {
                if (internal_state == Net2State.STARTED)
                {
                    long diff = GetTime() - start_time;
                    if (diff > 2000)
                    {
                        reportLog("Stopping cause monitor failure", LogType.WARN, section);
                        throw new Exception("monitor failure");
                    }
                }
                else
                if (internal_state == Net2State.CONNECTED)
                {
                    long diff = GetTime() - last_send_receive_time;
                    if (diff > 5000)
                    {
                        Monitor_Disconnected(null,null);
                    }
                }
            }

            protected override void _Send()
            {

            }

            protected override void threadExitProcess()
            {
               
            }

            protected override void Monitor_Disconnected(object sender, NetMQMonitorSocketEventArgs e)
            {
                reportLog("Monitor disconnected is called " + name, LogType.INFO, section);
                thread_exit = true;
                Dispose(false);
                reportCrash(); 
            }

            protected override void Monitor_Connected(object sender, NetMQMonitorSocketEventArgs e)
            {
                base.Monitor_Connected(sender, e);
                setState(Net2State.CONNECTED);
            }

            protected override void reportData(Message packet)
            {
                last_send_receive_time = GetTime();

                if (internal_state == Net2State.STARTED)
                {
                    Monitor_Connected(null, null);
                }

                delegateNewData?.Invoke(packet.header.sequence, packet.payload, packet.header.priority, Id);
            }

            protected override void StopMonitor()
            {
                if (monitor != null)
                {
                    monitor.Disconnected -= Monitor_Disconnected;
                    monitor.Connected -= Monitor_Connected;
                    monitor.Stop();
                }
            }
        }
    }
}
