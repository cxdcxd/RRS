using System;
using System.Collections.Generic;
using System.Net.NetworkInformation;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using RRS.Tools.Log;
using RRS.Tools.Protobuf;
using Consul;
using NetMQ;
using NetMQ.Monitoring;
using ProtoBuf;
using UnityEngine;
using LogType = RRS.Tools.Log.LogType;

namespace RRS.Tools.Network
{
    public partial class Net2
    {
        abstract private class Net2Base : IDisposable
        {
            #region Events

            public delegate void DelegateStateChanged(ulong id);
            public event DelegateStateChanged delegateStateChanged;

            public delegate void DelegateNewLog(string log_message, LogType log_type, string section);
            public event DelegateNewLog delegateNewLog;

            public delegate long DelegateGetTime();
            public event DelegateGetTime delegateGetTime;

            public delegate void DelegateDisposing(ulong id);
            public event DelegateDisposing delegateDisposing;

            public delegate void DelegateCrashed(ulong id);
            public event DelegateCrashed delegateCrashed;

            public delegate void DelegateSendChanged(ulong id);
            public event DelegateSendChanged delegateSendChanged;

            public delegate void DelegateReceiveChanged(ulong id);
            public event DelegateReceiveChanged delegateReceiveChanged;

            #endregion

            public long start_time;

            public static ulong last_id = 0;

            private ulong id;

            protected NetMQSocket socket;
            protected NetMQMonitor monitor;
            protected Thread thread;
            protected Task task;
            protected TransmitedData bytes_send;
            protected TransmitedData bytes_receive;
            protected long req_sequence = 1;
            protected System.Threading.ThreadPriority thread_priority = System.Threading.ThreadPriority.Highest;
            protected Net2State internal_state = Net2State.STOPPED;
            protected string section;
            protected bool thread_exit = false;
            protected string name = "";
            protected uint time_out_read_ms = 10;
            protected int local_port = 0;
            protected string local_ip = "";
            protected uint connection_count = 0;
            protected long last_send_receive_time = 0;
            protected object update_lock = null;

            private List<Message> send_queue;
            private object send_queue_lock;
            private object state_lock;
            private bool disposed = false;
            

            public Net2Base()
            {
                this.id = ++last_id;

                this.send_queue = new List<Message>();
                this.send_queue_lock = new object();
                this.state_lock = new object();

                section = this.GetType().Name + id;
                setState(Net2State.STOPPED);

                bytes_send = new TransmitedData();
                bytes_send.delegateTransmitedChanged += Bytes_send_delegateTransmitedChanged;

                bytes_receive = new TransmitedData();
                bytes_receive.delegateTransmitedChanged += Bytes_receive_delegateTransmitedChanged;

                thread_exit = false;
                thread = new Thread(new ThreadStart(socketThread));
                thread.Priority = thread_priority;
                thread.Start();
            }

            ~Net2Base()
            {
                Dispose(false);
                thread_exit = true;
            }

            public ulong Id { get { return id; } }

            protected void addToSendQueue(Message message)
            {
                lock (send_queue_lock)
                {
                    send_queue.Add(message);
                }
            }

            protected Message getFromSendQueue()
            {
                Message message = null;

                lock (send_queue_lock)
                {
                    if (send_queue.Count > 0)
                    {
                        message = send_queue[0];
                        send_queue.RemoveAt(0);
                    }
                }

                return message;
            }

            protected virtual void _Send()
            {
                Message message = getFromSendQueue();

                if (message != null)
                {
                    using (System.IO.MemoryStream ms = new System.IO.MemoryStream())
                    {
                        Serializer.Serialize<Message>(ms, message);

                        Msg msg = new Msg();

                        if (message.header.zmq_router_address != null)
                        {
                            byte[] address = message.header.zmq_router_address;

                            msg.InitGC(address, address.Length);
                            socket.Send(ref msg, true);
                            bytes_send.Bytes += (ulong)address.Length;

                            msg.InitGC(new byte[0], 0);
                            socket.Send(ref msg, true);
                            bytes_send.Bytes += 0;
                        }

                        msg.InitGC(ms.ToArray(), (int)ms.Length);
                        socket.Send(ref msg, false);
                        bytes_send.Bytes += (ulong)ms.Length;
                    }
                }
            }

            protected virtual void _CheckState()
            {
                throw new TypeAccessException();
            }

            protected virtual void _SocketBuilder()
            {
                throw new TypeAccessException();
            }

            
            protected virtual void _Receive()
            {
                Msg msg = new Msg();
                msg.InitEmpty();

                bool result = false;
                List<Msg> address_buffer = new List<Msg>();

                do
                {
                    result = socket.TryReceive(ref msg, TimeSpan.FromMilliseconds((int)time_out_read_ms)); //non-block-100hz

                    if (result && msg.HasMore == false && msg.Data.Length > 0)
                    {
                        Message packet = null;

                        using (System.IO.MemoryStream ms = new System.IO.MemoryStream(msg.Data))
                        {
                            packet = Serializer.Deserialize<Message>(ms);
                        }

                        if (packet != null)
                        {
                            bytes_receive.Bytes += (ulong)msg.Data.Length;

                            if (address_buffer.Count == 1)
                            {
                                //address buffer stored the router address; Router address has two parts, first is empty, second is the address
                                //So we select the second Msg of it
                                Msg m = address_buffer[0];

                                if (m.Data.Length > 0)
                                {
                                    packet.header.zmq_router_address = m.Data;
                                }
                            }

                            reportData(packet);
                        }
                        else
                        {
                            reportLog("Ivalid packet", LogType.ERROR, section);
                        }
                    }
                    else if (result && msg.HasMore)
                    {
                        address_buffer.Add(msg);
                    }

                } while (result && msg.HasMore);
            }

            public uint ConnectionCount
            {
                get { return connection_count; }
            }

            public TransmitedData BytesSend
            {
                get
                {
                    return (TransmitedData)bytes_send.Clone();
                }
            }

            public TransmitedData BytesReceive
            {
                get
                {
                    return (TransmitedData)bytes_receive.Clone();
                }
            }

            public int LocalPort
            {
                get { return local_port; }
            }

            public string LocalIp
            {
                get { return local_ip; }
            }

            public Net2State State
            {
                get { return internal_state; }
            }

            public string Name
            {
                get { return name; }
            }

            protected void reportState()
            {
                delegateStateChanged?.Invoke(id);
            }

            protected virtual void reportData(Message packet)
            {
                throw new TypeAccessException();
            }

            protected void reportLog(string log_message, LogType log_type, string section)
            {
                delegateNewLog?.Invoke(log_message, log_type, section);
            }

            protected void socketThread()
            {
                bool is_crashed = false;
                reportLog("Socket thread started", LogType.INFO, section);
                try
                {
                    while (thread_exit == false)
                    {
                        if (socket != null)
                        {
                            _CheckState();
                            _Receive();
                            _Send();
                            Thread.Sleep(1);
                        }
                        else
                        {
                            _SocketBuilder();
                            Thread.Sleep(100);
                        }
                    }
                }
                catch (Exception e)
                {
                    is_crashed = true;
                    reportLog(e.Message, LogType.ERROR, section);
                }
                finally
                {
                    reportLog("Thread exited clearly for " + Name, LogType.INFO, section);
                    reportLog("Exit params " + thread_exit + " " + (socket == null) + " " + is_crashed, LogType.INFO, section);

                    StopMonitor();
                    reportLog("Monitor stopped.", LogType.INFO, section);

                    threadExitProcess();

                    if (socket != null)
                        socket.Close();

                    if (is_crashed)
                    {
                        Dispose(false);
                        reportLog("Reporting crash", LogType.INFO, section);
                        delegateCrashed?.Invoke(Id);
                        reportLog("Crash reported", LogType.INFO, section);
                    }
                }
            }

            protected void reportCrash()
            {
                delegateCrashed?.Invoke(Id);
            }

            protected virtual void threadExitProcess()
            {
                throw new TypeAccessException();
            }

            protected virtual void Monitor_Disconnected(object sender, NetMQMonitorSocketEventArgs e)
            {
                if (connection_count > 0)
                    connection_count--;
            }

            protected virtual void Monitor_Connected(object sender, NetMQMonitorSocketEventArgs e)
            {
                connection_count++;
            }

            protected Message createMessage(byte[] buffer, uint priority = 0)
            {
                Message msg = new Message();
                msg.header = new Header();
             
                msg.header.sequence = req_sequence++;

                if (delegateGetTime != null)
                    msg.header.time_span = delegateGetTime();
                    
                msg.header.source_channel_name = Name;
                msg.header.priority = priority;
                msg.payload = buffer;

                return msg;
            }

            protected void setState(Net2State new_state)
            {
                if (this.internal_state != new_state)
                {
                    this.internal_state = new_state;
                    reportState();
                }
            }

            private void Bytes_receive_delegateTransmitedChanged()
            {
                delegateReceiveChanged?.Invoke(id);
            }

            private void Bytes_send_delegateTransmitedChanged()
            {
                delegateSendChanged?.Invoke(id);
            }

            protected long GetTime()
            {
                return delegateGetTime?.Invoke() ?? 0;
            }

            protected virtual void StopMonitor()
            {
                throw new TypeAccessException("Net2Base");
            }

            public bool IsDisposed { get { return disposed; } }

            public void Dispose()
            {
                Dispose(true);
                GC.SuppressFinalize(this);
            }

            protected virtual void Dispose(bool disposing)
            {
                if (disposed)
                    return;

                if (disposing)
                {
                    // Free any other managed objects here.
                    thread_exit = true;
                    delegateDisposing?.Invoke(id);
                }

                // Free any unmanaged objects here.

                disposed = true;
            }
        }
    }
}
