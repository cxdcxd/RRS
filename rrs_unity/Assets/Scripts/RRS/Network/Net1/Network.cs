/// <summary>
/// Arsam Robotics 2017
/// </summary>
/// author  : edwin.babaians@gmail.com

using NetMQ;
using NetMQ.Monitoring;
using NetMQ.Sockets;

using ProtoBuf;
using RRS.Tools.Log;
using RRS.Tools.Protobuf;
using System;
using System.Collections.Generic;
using System.IO;
using System.Net.NetworkInformation;
using System.Threading.Tasks;
using RRS.Tools;

namespace RRS
{
    namespace Tools
    {
        namespace Network
        {

            public class Network<PUBTYPE, SUBTYPE>
            {
                #region vars

                Ping ping;
                string section = "Network";
                public int max_history = 100000;
                public int ping_timeout_sec = 2;

                ulong status_index = 0;

                public string ping_last_message = "";
                public ulong ping_time = 0;
                public NetworkType type = NetworkType.PUBSUB;
                public int max_connection_timeout = 5; //sec
                public int connection_timeout = 0;
                public ulong stream_timeout = 5000; //ms
                public bool connected = false;
                public string name = "net";
                public List<string> subscribers = new List<string>();

                public bool read_timeout_error = false;
                public bool write_timeout_error = false;
                public bool is_ping_enabled = false;
                public bool is_subscriber_monitor_enabled = false;

                public string remote_ip = "";
                public string local_ip = "";

                public ulong last_read_time = 0;
                public ulong last_write_time1 = 0;
                public ulong last_write_time2 = 0;

                public ulong last_read_dt = 0;
                public ulong last_write_dt = 0;

                public NetworkState state = NetworkState.IDLE;
                public NetworkResponse result_response = NetworkResponse.NONE;

                public System.Timers.Timer main_timer = new System.Timers.Timer();

                public delegate void dstatusupdated();

                public event dstatusupdated eventSetupStatus;
                public event dstatusupdated eventSubscriberChanged;
                public event dstatusupdated eventConnectionStatus;
                public event dstatusupdated eventDataUpdated;

                public SUBTYPE get; //get for serialized datas
                public byte[] get_raw_bytes; //Raw bytes

                PublisherSocket publisher;
                SubscriberSocket subscriber;
                RequestSocket request;

                bool app_exit_sub = false;
                bool app_exit_res = false;

                System.Threading.Thread sub_thread;
                System.Threading.Thread res_thread;

                bool busy = false;

                string local_port = "";
                string remote_port = "";
                string request_port = "";

                #endregion

                #region  methods

                #region  Events
                /// <summary>
                /// Notify parents for data recieved
                /// </summary>
                public void dataTrigger()
                {
                    if (eventDataUpdated != null) eventDataUpdated();
                }

                public void trigerSetupStatus()
                {
                    if (eventSetupStatus != null) eventSetupStatus();
                    delegateNewLog?.Invoke("Network setup status : " + state, LogType.INFO, section);
                }

                public void triggerSubscriberChange(bool add)
                {
                    if (eventSubscriberChanged != null) eventSubscriberChanged();
                    if (add)
                        delegateNewLog?.Invoke("Network subscriber changed : " + subscribers.Count + " " + subscribers[subscribers.Count - 1].ToString(), LogType.INFO, section);
                }

                public void triggerConnectionStatus()
                {
                    if (eventConnectionStatus != null) eventConnectionStatus();
                    if (state != NetworkState.CONNECTED)
                        delegateNewLog?.Invoke("Network connection status : " + state, LogType.INFO, section);
                }

                void updateConnectionStatus()
                {
                    triggerConnectionStatus();
                }

                void updateSetupStatus(bool value)
                {
                    connection_timeout = max_connection_timeout;

                    if (value != connected)
                    {
                        connected = value;
                        trigerSetupStatus();
                    }
                }

                public delegate void DelegateNewLog(string log_message, LogType log_type, string section);
                public event DelegateNewLog delegateNewLog;

                #endregion


                /// <summary>
                /// Setup subscriber and publisher for this network
                /// </summary>
                /// <param name="ip"></param>
                /// <param name="only_pub"></param>
                void setup(string ip = "")
                {
                    if (state == NetworkState.SETUP && busy == false)
                    {
                        app_exit_sub = false;
                        busy = true;

                        if (type != NetworkType.SUB)
                        {
                            publisher = new PublisherSocket();

                            if (is_subscriber_monitor_enabled)
                            {
                                //monitor = new NetMQMonitor(publisher, "inproc://pub.inproc" + local_port, SocketEvents.All);
                                //monitor.Disconnected += Monitor_Disconnected;
                                //monitor.EventReceived += Monitor_EventReceived;
                                //monitor.Accepted += Monitor_Accepted;
                                //monitor.Timeout = new TimeSpan(0, 0, 0, 0, 1000);
                                //Task task = monitor.StartAsync();
                            }

                            publisher.Bind("tcp://*:" + local_port);

                            if (ip == "") ip = remote_ip;

                            if (type != NetworkType.PUB)
                            {
                                //We shoudl have a subscriber too !
                                subscriber = new SubscriberSocket();
                                subscriber.Subscribe("");
                                subscriber.Connect("tcp://" + ip + ":" + remote_port);
                                sub_thread = new System.Threading.Thread(new System.Threading.ThreadStart(zmqSubThread));
                                sub_thread.Start();
                            }

                            delegateNewLog?.Invoke("Creating new network for remote: " + remote_ip + " and port: " + remote_port + " " + type.ToString(), LogType.INFO, section);
                            state = NetworkState.CONNECTED;
                            busy = false;
                        }
                    }
                }

                private void Monitor_Accepted(object sender, NetMQMonitorSocketEventArgs e)
                {
                    subscribers.Add(e.Address);
                    triggerSubscriberChange(true);
                }

                private void Monitor_EventReceived(object sender, NetMQMonitorEventArgs e)
                {

                }


                private void Monitor_Disconnected(object sender, NetMQMonitorSocketEventArgs e)
                {
                    subscribers.Remove(e.Address);
                    triggerSubscriberChange(false);
                }

                /// <summary>
                /// Check for data receive timeout
                /// </summary>
                public void timeoutChecker()
                {
                    if (connection_timeout > 0)
                        connection_timeout--;
                    else
                        if (connection_timeout == 0)
                    {
                        result_response = NetworkResponse.TIMEOUT;

                        if (type == NetworkType.PUBSUBREQRES)
                        {
                            killSetup();
                            killConnect();
                        }

                        state = NetworkState.DISCONNECTED;

                        //Notify
                        updateSetupStatus(false);
                        updateConnectionStatus();
                    }
                }

                /// <summary>
                /// Setup req/rev for this network
                /// </summary>
                public void connect()
                {
                    if (type != NetworkType.PUBSUBREQRES) return;

                    if (state == NetworkState.IDLE)
                    {
                        request = new RequestSocket();
                        request.Connect("tcp://" + remote_ip + ":" + request_port);

                        app_exit_res = false;
                        res_thread = new System.Threading.Thread(new System.Threading.ThreadStart(zmqResThread));
                        res_thread.Start();

                        state = NetworkState.CONNECTING;
                    }
                }

                /// <summary>
                /// Main networking timer
                /// </summary>
                /// <param name="sender"></param>
                /// <param name="e"></param>
                private void Main_timer_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
                {
                    #region  TimeCalculations

                    ulong time = Helper.getCurrentTimeSpan();
                    last_read_dt = (ulong)Math.Abs((long)(time - last_read_time));

                    if (last_read_dt > stream_timeout)
                    {
                        read_timeout_error = true;
                    }
                    else
                    {
                        read_timeout_error = false;
                    }

                    last_write_dt = (ulong)Math.Abs((long)(last_write_time2 - last_write_time1));

                    if (last_write_dt > stream_timeout)
                    {
                        write_timeout_error = true;
                    }
                    else
                    {
                        write_timeout_error = false;
                    }

                    if (last_write_dt > stream_timeout) last_write_dt = stream_timeout;
                    if (last_read_dt > stream_timeout) last_read_dt = stream_timeout;

                    status_index++;
                    if (status_index % 2 == 0)
                    {
                        if (!string.IsNullOrEmpty(remote_ip) && is_ping_enabled)
                            ping.SendAsync(remote_ip, ping_timeout_sec, null);
                    }

                    #endregion

                    if (state == NetworkState.CONNECTED || state == NetworkState.CONNECTING)
                    {
                        timeoutChecker();
                    }
                    else if (state == NetworkState.SETUP)
                    {
                        setup();
                    }
                    else if (state == NetworkState.IDLE)
                    {

                    }
                    else if (state == NetworkState.DISCONNECTED)
                    {
                        state = NetworkState.IDLE;
                    }
                }

                /// <summary>
                /// Set ref time from parent if it is needed
                /// </summary>
                /// <param name="dt"></param>
                public void setRefTime(DateTime dt)
                {
                    Helper.ref_time = dt;
                }

                /// <summary>
                /// Main Constructor
                /// </summary>
                /// <param name="local_ip"></param>
                /// <param name="remote_ip"></param>
                /// <param name="local_port"></param>
                /// <param name="remote_port"></param>
                /// <param name="req_port"></param>
                /// <param name="type"></param>
                public Network(string local_ip, string remote_ip, string local_port, string remote_port, string req_port, NetworkType type = NetworkType.PUBSUB, string name = "", bool ping_enable = false, bool subscirber_enable = false)
                {
                    this.is_ping_enabled = ping_enable;
                    this.is_subscriber_monitor_enabled = subscirber_enable;
                    this.name = name;
                    this.type = type;
                    this.connection_timeout = max_connection_timeout;
                    this.local_ip = local_ip;
                    this.remote_ip = remote_ip;
                    this.local_port = local_port;
                    this.remote_port = remote_port;
                    this.request_port = req_port;

                    main_timer = new System.Timers.Timer();
                    main_timer.Interval = 1000;
                    main_timer.Elapsed += Main_timer_Elapsed;
                    main_timer.Start();
                    main_timer.Enabled = true;

                    if (type != NetworkType.PUBSUBREQRES)
                    {
                        state = NetworkState.SETUP;
                        setup(remote_ip);
                    }

                    if (this.is_ping_enabled)
                    {
                        ping = new Ping();
                        ping.PingCompleted += Ping_PingCompleted;
                    }
                }

                private void Ping_PingCompleted(object sender, PingCompletedEventArgs e)
                {
                    try
                    {
                        ping_last_message = e.Reply.Status.ToString();

                        if (e.Reply.Status == IPStatus.Success)
                            ping_time = (ulong)e.Reply.RoundtripTime;
                        else
                        {
                            ping_time = (ulong)ping_timeout_sec * 1000;
                        }

                    }
                    catch (Exception ee)
                    {
                        ping_last_message = ee.Message;
                        ping_time = 0;
                    }
                }

                #region Kills

                //Force Kill request from parent
                public void killAll()
                {
                    killSetup();
                    killConnect();

                    state = NetworkState.DISCONNECTED;
                    updateSetupStatus(false);

                    delegateNewLog?.Invoke("Killall Network done", LogType.INFO, section);
                }
                public void killSetup()
                {
                    app_exit_sub = true;

                    if (subscriber != null)
                    {
                        subscriber.Close();
                    }

                    if (publisher != null)
                    {
                        publisher.Close();
                    }

                    if (sub_thread != null)
                    {
                        sub_thread.Abort();
                    }

                    GC.Collect();
                    GC.WaitForPendingFinalizers();

                }
                public void killConnect()
                {
                    app_exit_res = true;

                    if (res_thread != null)
                    {
                        res_thread.Abort();
                    }

                    if (request != null)
                    {
                        request.Close();
                    }
                }

                #endregion

                #region Receive Threads
                void zmqResThread()
                {
                    while (app_exit_res == false)
                    {
                        if (request != null)
                        {
                            try
                            {
                                System.Threading.Thread.Sleep(1000);

                                if (state == NetworkState.CONNECTING)
                                    sendRequest("connect");
                                else
                                if (state == NetworkState.CONNECTED)
                                    sendRequest("alive");
                                else
                                {
                                    continue;
                                }

                                Msg msg = new Msg();
                                msg.InitEmpty();
                                bool dresult = request.TryReceive(ref msg, new TimeSpan(0, 0, 0, 0, 100));

                                if (dresult)
                                {
                                    System.IO.MemoryStream ms = new System.IO.MemoryStream(msg.Data);
                                    RRSConnection result = Serializer.Deserialize<RRSConnection>(ms);

                                    if (result != null)
                                    {
                                        if (result.status == "connected")
                                        {
                                            state = NetworkState.SETUP;
                                            result_response = NetworkResponse.CONNECTED;
                                            delegateNewLog?.Invoke("Get connection result : " + result_response.ToString(), LogType.INFO, section);
                                        }
                                        else if (result.status == "rejected")
                                        {
                                            state = NetworkState.DISCONNECTED;
                                            result_response = NetworkResponse.REJECTED;
                                            delegateNewLog?.Invoke("Get connection result : " + result_response.ToString(), LogType.WARN, section);
                                        }
                                        else if (result.status == "busy")
                                        {
                                            result_response = NetworkResponse.BUSY;
                                            delegateNewLog?.Invoke("Get connection result : " + result_response.ToString(), LogType.WARN, section);
                                        }
                                        else if (result.status == "ack")
                                        {
                                            //Network ack message 
                                            last_write_time2 = Helper.getCurrentTimeSpan();
                                            result_response = NetworkResponse.ACK;
                                        }
                                        else if (result.status == "error")
                                        {
                                            //nothing
                                            result_response = NetworkResponse.ERROR;
                                            delegateNewLog?.Invoke("Get connection result : " + result_response.ToString(), LogType.ERROR, section);
                                        }

                                        updateConnectionStatus();
                                    }
                                }
                            }
                            catch
                            {

                            }
                        }

                    }
                }
                void zmqSubThread()
                {
                    while (app_exit_sub == false && subscriber != null)
                    {
                        try
                        {
                            Msg msg = new Msg();
                            msg.InitEmpty();
                            bool dresult = subscriber.TryReceive(ref msg, new TimeSpan(0, 0, 0, 0,10));

                            if (dresult)
                            {
                                last_read_time = Helper.getCurrentTimeSpan();

                                if (typeof(SUBTYPE) == typeof(RawData))
                                {
                                    get_raw_bytes = msg.Data;
                                }
                                else
                                {
                                    System.IO.MemoryStream ms = new System.IO.MemoryStream(msg.Data);
                                    SUBTYPE result = Serializer.Deserialize<SUBTYPE>(ms);
                                    get = result;
                                }

                                if (app_exit_sub == false)
                                {
                                    state = NetworkState.CONNECTED;
                                    updateSetupStatus(true);
                                    dataTrigger();
                                }

                            }
                        }
                        catch
                        {

                        }

                    }
                }


                #endregion

                #region Send
                public void sendRequest(string req_msg)
                {
                    if (type != NetworkType.PUBSUBREQRES) return;

                    if (state == NetworkState.CONNECTING || state == NetworkState.CONNECTED)
                    {
                        try
                        {
                            RRSConnection cmd = new RRSConnection();
                            cmd.ip = local_ip;
                            cmd.status = req_msg;

                            System.IO.MemoryStream ms = new System.IO.MemoryStream();
                            Serializer.Serialize<RRSConnection>(ms, cmd);
                            Msg msg = new Msg();
                            msg.InitGC(ms.ToArray(), (int)ms.Length);

                            last_write_time1 = Helper.getCurrentTimeSpan();
                            request.Send(ref msg, false);
                        }
                        catch (Exception e)
                        {
                            string msg = e.Message;
                        }
                    }
                }
                public bool sendMessage(PUBTYPE cmd)
                {
                    if (type == NetworkType.SUB) return false;

                    try
                    {
                        if (typeof(PUBTYPE) == typeof(RawData))
                        {
                            Msg msg = new Msg();
                            IRawData d = (IRawData)(cmd);
                            msg.InitGC(d.Data, d.Data.Length);
                            publisher.Send(ref msg, false);
                        }
                        else
                        {
                            //cmd is a command and we serialize it
                            System.IO.MemoryStream ms = new System.IO.MemoryStream();
                            Serializer.Serialize<PUBTYPE>(ms, cmd);
                            Msg msg = new Msg();
                            msg.InitGC(ms.ToArray(), (int)ms.Length);
                            publisher.Send(ref msg, false);
                        }
                    }
                    catch (Exception e)
                    {
                        string msg = e.Message;
                        return false;
                    }

                    return true;
                }


                #endregion

                #endregion

            }
        }
    }
}