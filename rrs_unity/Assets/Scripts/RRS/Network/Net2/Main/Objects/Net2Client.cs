using Consul;
using NetMQ;
using NetMQ.Monitoring;
using NetMQ.Sockets;
using ProtoBuf;
using RRS.Tools.Log;
using RRS.Tools.Protobuf;
using System;
using System.Collections.Generic;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace RRS.Tools.Network
{
    public partial class Net2
    {
        private class Net2Client : Net2Base
        {
            public delegate ProcessResult DelegateGetServiceInfo(string name);
            public event DelegateGetServiceInfo delegateGetServiceInfo;

            public delegate void DelegateNewData(ulong sequence, byte[] buffer, uint priority, ulong id);
            public event DelegateNewData delegateNewData;

            AgentService info;

            public Net2Client(string name) : base()
            {
                this.name = name;
            }

            public ulong Send(byte[] data, uint priority = 0)
            {
                if (internal_state == Net2State.CONNECTED)
                {
                    var msg = createMessage(data, priority);

                    addToSendQueue(msg);

                    return msg.header.sequence;
                }
                return 0;
            }

            protected override void threadExitProcess()
            {

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
                reportLog("Starting Dealer for ip " + info.Address + " and port " + info.Port, LogType.INFO, section);

                string path = "tcp://" + info.Address + ":" + info.Port;

                socket = new DealerSocket();

                socket.Options.TcpKeepalive = true;
                socket.Options.TcpKeepaliveIdle = TimeSpan.FromMilliseconds(100);
                socket.Options.TcpKeepaliveInterval = TimeSpan.FromMilliseconds(100);

                monitor = new NetMQMonitor(socket, "inproc://dealer" + Net2.GetAddressIndex(), SocketEvents.All);
                monitor.Disconnected += Monitor_Disconnected;
                monitor.Connected += Monitor_Connected;
                monitor.Timeout = TimeSpan.FromMilliseconds(100);
                task = Task.Factory.StartNew(monitor.Start);

                setState(Net2State.STARTED);
                start_time = GetTime();

                socket.Connect(path);

                reportLog("Dealer is connected to " + path, LogType.INFO, section);
            }

            protected override void _CheckState()
            {
                if (internal_state == Net2State.STARTED)
                {
                    ulong diff = GetTime() - start_time;
                    if (diff > 2000)
                    {
                        reportLog("Stopping cause monitor failure", LogType.WARN, section);
                        throw new Exception("monitor failure");
                    }
                }
            }

            protected override void reportData(Message packet)
            {
                delegateNewData?.Invoke(packet.header.sequence, packet.payload, packet.header.priority, Id);
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
