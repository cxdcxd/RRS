using Consul;
using RRS.Tools.Log;
using RRS.Tools.Protobuf;
using System;
using System.Collections.Generic;
using System.Text;
using System.Timers;
using System.Linq;
using RRS.Tools;
using UnityEngine;
using LogType = RRS.Tools.Log.LogType;

namespace RRS.Tools.Network
{
    public partial class Net2
    {
        #region Private

        public delegate void DelegateStateChanged(Net2HandlerBase sender);
        public event DelegateStateChanged delegateInfoServiceStateChanged;

        public delegate void DelegateNewLog(string log_message, LogType log_type, string section);
        public static event DelegateNewLog delegateInfoServiceNewLog;

        public delegate byte[] DelegateGetResponse(ulong sequense, byte[] buffer, uint priority, ulong service_id);
        DelegateGetResponse GetResponse;
        public delegate void DelegateNewData(ulong sequense, byte[] buffer, uint priority, Net2HandlerBase sender);
        public event DelegateNewData delegateInfoServiceRequested;

        private bool is_handle_intternal_service_calls = true;
        private Net2State state = Net2State.STOPPED;
        private Net2Consul net2_consul;
        private Dictionary<ulong, Net2HandlerBase> channels_list;
        private System.Timers.Timer main_timer;
        private static Net2 instance = null;
        private static readonly object padlock = new object();

        private object @lock;
        //private Net2Service net2_service;
        private Net2Config net2_config;
        private Net2NTPClient net2_ntp;
        private string local_network_address = "127.0.0.1";

        private static int address_index = 1;

        #endregion

        private static Net2 Instance
        {
            get
            {
                lock (padlock)
                {
                    if (instance == null)
                    {
                        instance = new Net2();
                    }
                    return instance;
                }
            }
        }

        private Net2()
        {
            @lock = new object();
        }

        public static string NameSpace
        {
            get { return Instance.net2_config.name_space; }
        }

        public static int GetAddressIndex()
        {
            if (address_index == int.MaxValue)
            {
                address_index = 1;
            }
            return address_index++;
        }

        #region  CreateObjects

        private ulong getTime()
        {
            return net2_ntp.get();
        }

        public static Net2HandlerPublisher Publisher(string name)
        {
            return Instance.CreatePublisher(name);
        }

        private Net2HandlerPublisher CreatePublisher(string name)
        {
            if (state == Net2State.STARTED)
            {
                string new_name = NameSpace + "-" + name;
                if (IsTopicExist(new_name))
                {
                    throw new ArgumentException("Duplicate topic name", new_name);
                }
                Net2HandlerPublisher publisher_handler = new Net2HandlerPublisher();

                (publisher_handler as INet2Handler).SetInstance(publisher(new_name));

                addChannel(publisher_handler);
                return publisher_handler;
            }
            else
            {
                throw new Exception("Init frist");
            }
        }

        private Net2Publisher publisher(string name)
        {
            Net2Publisher publisher = new Net2Publisher(name);

            publisher.delegateDisposing += DisposingChannel;
            publisher.delegateGetTime += getTime;
            publisher.delegateCrashed += ChannelCrashed;
            publisher.delegateNewLog += delegateNewLog;

            return publisher;
        }

        private void delegateNewLog(string log_message, LogType log_type, string section)
        {
            delegateInfoServiceNewLog?.Invoke(log_message, log_type, section);
        }

        public static Net2HandlerSubscriber Subscriber(string name)
        {
            return Instance.CreateSubscriber(name);
        }

        private Net2HandlerSubscriber CreateSubscriber(string name)
        {
            if (state == Net2State.STARTED)
            {
                Net2HandlerSubscriber subscriber_handler = new Net2HandlerSubscriber();
                (subscriber_handler as INet2Handler).SetInstance(subscriber(name));

                addChannel(subscriber_handler);
                return subscriber_handler;
            }
            else
            {
                throw new Exception("Init frist");
            }
        }

        private Net2Subscriber subscriber(string name)
        {
            Net2Subscriber subscriber = new Net2Subscriber(name);

            subscriber.delegateDisposing += DisposingChannel;
            subscriber.delegateGetTime += getTime;
            subscriber.delegateGetServiceInfo += getService;
            subscriber.delegateCrashed += ChannelCrashed;
            subscriber.delegateNewLog += delegateNewLog;

            return subscriber;
        }

        public static Net2HandlerService Service(string name, DelegateGetResponse response_callback)
        {
            return Instance.CreateService(name, response_callback);
        }

        private Net2HandlerService CreateService(string name, DelegateGetResponse response_callback)
        {
            if (state == Net2State.STARTED)
            {
                string new_name = NameSpace + "-" + name;
                if (IsTopicExist(new_name))
                {
                    throw new ArgumentException("Duplicate topic name", "name");
                }
                Net2HandlerService service_handler = new Net2HandlerService();
                (service_handler as INet2Handler).SetInstance(service(new_name, response_callback));

                addChannel(service_handler);
                return service_handler;
            }
            else
            {
                throw new Exception("Init frist");
            }
        }

        private Net2Service service(string name, DelegateGetResponse response_callback)
        {
            Net2Service service = new Net2Service(name, response_callback);

            service.delegateDisposing += DisposingChannel;
            service.delegateGetTime += getTime;
            service.delegateCrashed += ChannelCrashed;
            service.delegateNewLog += delegateNewLog;

            return service;
        }

        public static Net2HandlerClient Client(string name)
        {
            return Instance.CreateClient(name);
        }

        private Net2HandlerClient CreateClient(string name)
        {
            if (state == Net2State.STARTED)
            {
                Net2HandlerClient client_handler = new Net2HandlerClient();
                (client_handler as INet2Handler).SetInstance(client(name));

                addChannel(client_handler);
                return client_handler;
            }
            else
            {
                throw new Exception("Init frist");
            }
        }

        private Net2Client client(string name)
        {
            Net2Client client = new Net2Client(name);

            client.delegateDisposing += DisposingChannel;
            client.delegateGetTime += getTime;
            client.delegateGetServiceInfo += getService;
            client.delegateCrashed += ChannelCrashed;
            client.delegateNewLog += delegateNewLog;

            return client;
        }

        private void RecreateChannel(ulong id)
        {
            lock (@lock)
            {
                if (channels_list.ContainsKey(id))
                {
                    Net2Base instance = (channels_list[id] as INet2Handler).Instance;
                    DeleteChannelsEvents(instance);
                    if (instance is Net2Publisher)
                    {
                        (channels_list[id] as INet2Handler).ReleaseInstance();
                        (channels_list[id] as INet2Handler).SetInstance(publisher(instance.Name));
                    }
                    else if (instance is Net2Subscriber)
                    {
                        (channels_list[id] as INet2Handler).ReleaseInstance();
                        (channels_list[id] as INet2Handler).SetInstance(subscriber(instance.Name));
                    }
                    else if (instance is Net2Service)
                    {
                        (channels_list[id] as INet2Handler).ReleaseInstance();
                        (channels_list[id] as INet2Handler).SetInstance(service(instance.Name, (instance as Net2Service).ResponseCallback));
                    }
                    else if (instance is Net2Client)
                    {
                        (channels_list[id] as INet2Handler).ReleaseInstance();
                        (channels_list[id] as INet2Handler).SetInstance(client(instance.Name));
                    }
                    channels_list.Add(channels_list[id].Id, channels_list[id]);
                    channels_list.Remove(id);
                    
                }
            }
        }

        private void addChannel(Net2HandlerBase item)
        {
            ulong id = (item as INet2Handler).Instance.Id;
            lock (@lock)
            {
                channels_list.Add(id, item);
            }
        }

        private void ReleaseChannel(ulong id)
        {
            Net2Base instance = null;
            lock (@lock)
            {
                if (channels_list.ContainsKey(id))
                {
                    instance = (channels_list[id] as INet2Handler).Instance;
                    net2_consul.removeService(instance.Name);
                    channels_list.Remove(id);
                }
            }
            if (instance != null)
            {
                DeleteChannelsEvents(instance);
            }
        }

        private void DeleteChannelsEvents(Net2Base instance)
        {
            if (instance == null)
            {
                return;
            }
            if (instance is Net2Publisher)
            {
                instance.delegateDisposing -= DisposingChannel;
                instance.delegateGetTime -= getTime;
                instance.delegateCrashed -= ChannelCrashed;
                instance.delegateNewLog -= delegateNewLog;
            }
            else if (instance is Net2Subscriber)
            {
                instance.delegateDisposing -= DisposingChannel;
                instance.delegateGetTime -= getTime;
                (instance as Net2Subscriber).delegateGetServiceInfo -= getService;
                instance.delegateCrashed -= ChannelCrashed;
                instance.delegateNewLog -= delegateNewLog;
            }
            else if (instance is Net2Service)
            {
                instance.delegateDisposing -= DisposingChannel;
                instance.delegateGetTime -= getTime;
                instance.delegateCrashed -= ChannelCrashed;
                instance.delegateNewLog -= delegateNewLog;
            }
            else if (instance is Net2Client)
            {
                instance.delegateDisposing -= DisposingChannel;
                instance.delegateGetTime -= getTime;
                (instance as Net2Client).delegateGetServiceInfo -= getService;
                instance.delegateCrashed -= ChannelCrashed;
                instance.delegateNewLog -= delegateNewLog;
            }
        }

        private bool IsTopicExist(string name)
        {
            bool result = false;
            lock (@lock)
            {
                if (channels_list.Values.Any(x => x.Name == name && 
                ((x as INet2Handler).Instance is Net2Service || (x as INet2Handler).Instance is Net2Publisher)))
                {
                    result = true;
                }
            }
            return result;
        }

        #endregion

        public static void Init(Net2Config net2_config, DelegateGetResponse response_callback = null)
        {
            if (net2_config == null || string.IsNullOrEmpty(net2_config.name_space) || 
                string.IsNullOrEmpty(net2_config.consul_network_address) ||
                string.IsNullOrEmpty(net2_config.consul_network_mask) ||
                net2_config.consul_network_port == 0 ||
                string.IsNullOrEmpty(net2_config.ntp_server_host_name) ||
                net2_config.ntp_server_port == 0)
            {
                throw new ArgumentException();
            }

            Instance.Initialize(net2_config, response_callback);
        }

        private void Initialize(Net2Config net2_config, DelegateGetResponse response_callback)
        {
            lock (padlock)
            {
                if (state == Net2State.STOPPED)
                {
                    channels_list = new Dictionary<ulong, Net2HandlerBase>();

                    if (response_callback == null)
                    {
                        GetResponse = ResponseCallback;
                    }
                    else
                    {
                        GetResponse = response_callback;
                    }

                    this.net2_config = net2_config;
                    local_network_address = net2_config.local_network_address;

                    Net2NTPClient.delegateNewLogx += delegateNewLog;
                    net2_ntp = new Net2NTPClient(net2_config.ntp_server_host_name,true);


                    delegateNewLog("test", LogType.INFO, "test");

                    net2_consul = new Net2Consul("http://" + net2_config.consul_network_address + ":" + net2_config.consul_network_port, net2_config.consul_mode);

                    main_timer = new System.Timers.Timer();
                    main_timer.Interval = 1000;
                    main_timer.Elapsed += Main_timer_Elapsed;
                    main_timer.Enabled = true;
                    main_timer.Start();

                    state = Net2State.STARTED;
                }
                else
                {
                    throw new Exception("Already inited");
                }
            }
        }

        private void CreateInternalService()
        {
            //if (net2_service == null)
            //{
            //    string new_name = NameSpace + "-net2status";
            //    net2_service = new Net2Service(new_name, GetResponse);
            //    net2_service.delegateDisposing += Net2_service_delegateDisposing;
            //    net2_service.delegateNewLog += Net2_service_delegateNewLog;
            //    net2_service.delegateStateChanged += Net2_service_delegateStateChanged;
            //    net2_service.delegateCrashed += Net2_service_delegateCrashed;
            //}
        }

        private void Net2_service_delegateDisposing(ulong id)
        {
            ReleaseInternalService();
        }

        private void ReleaseInternalService()
        {
            //if (net2_service != null)
            //{
            //    net2_service.delegateDisposing -= DisposingChannel;
            //    net2_service.delegateNewLog -= Net2_service_delegateNewLog;
            //    net2_service.delegateStateChanged -= Net2_service_delegateStateChanged;
            //    net2_service.delegateCrashed -= Net2_service_delegateCrashed;
            //    net2_service = null;
            //}
        }

        private void Net2_service_delegateCrashed(ulong id)
        {
            ReleaseInternalService();
            CreateInternalService();
        }

        private void Net2_service_delegateStateChanged(ulong id)
        {
            Net2HandlerBase instance = null;
            lock (@lock)
            {
                if (channels_list.ContainsKey(id))
                {
                    instance = channels_list[id];
                }
            }

            if (instance != null)
            {
                delegateInfoServiceStateChanged?.Invoke(instance);
            }
        }

        private void DisposingChannel(ulong id)
        {
            ReleaseChannel(id);
        }

        private void ChannelCrashed(ulong id)
        {
            RecreateChannel(id);
        }

        private ProcessResult getService(string name)
        {
            ProcessResult result = new ProcessResult();

            AgentService info = net2_consul.getServiceInfo(name);

            if (info != null)
            {
                result.Result = info;
                result.Success = true;
                result.ResultType = info.GetType();
            }
            else
            {
                result.Result = null;
                result.Success = false;
                result.ResultType = null;
            }

            return result;
        }

        string getStationIP()
        {
            //return Net2Helper.getStationIp(net2_config.consul_network_mask, net2_config.consul_network_address);
            return local_network_address;
        }

        private void advertiseService(string name, int local_port)
        {
            net2_consul.advertiseService(name, local_port, getStationIP(), getTime().ToString());
        }

        private void Main_timer_Elapsed(object sender, ElapsedEventArgs e)
        {
            if (state == Net2State.STARTED)
            {
                //net2_consul.advertiseService(NameSpace, 0, getStationIP(), getTime().ToString());

                lock (@lock)
                {
                    Net2Base instance;
                    foreach (var item in channels_list.Values.Where(x => (x as INet2Handler).Instance != null && 
                    ((x as INet2Handler).Instance is Net2Service) || (x as INet2Handler).Instance is Net2Publisher))
                    {
                        instance = (item as INet2Handler).Instance;
                        advertiseService(instance.Name, instance.LocalPort);
                    }


                }
            }
        }

        private byte[] ResponseCallback(ulong sequense, byte[] buffer, uint priority, ulong service_id)
        {
            Net2StationInfo station_info = new Net2StationInfo();
            station_info.host_name = NameSpace;
            station_info.ip = "127.0.0.1";

            List<Net2HandlerBase> channels;
            lock (@lock)
            {
                int size = channels_list.Count;
                station_info.items = new Net2ServiceInfo[size];

                channels = channels_list.Values.ToList();
            }

            Net2Base instance;
            for (int i = 0; i < channels.Count; i++)
            {
                Net2ServiceInfo msg = new Net2ServiceInfo();
                instance = (channels[i] as INet2Handler).Instance;

                msg.name = instance.Name;

                msg.local_ip = instance.LocalIp;
                msg.local_port = instance.LocalPort.ToString();

                msg.network_type = instance.GetType().ToString();
                msg.state = instance.State.ToString();

                msg.connection_count = instance.ConnectionCount.ToString();
                msg.bytes_sent = instance.BytesSend.ToString();
                msg.bytes_received = instance.BytesReceive.ToString();

                station_info.items[i] = msg;
            }

            System.IO.MemoryStream ms = new System.IO.MemoryStream();
            ProtoBuf.Serializer.Serialize<Net2StationInfo>(ms, station_info);
            return ms.ToArray();
        }

        public static void Shutdown()
        {
            Instance.InternalShutdown();
        }

      

        private void InternalShutdown()
        {
            lock (padlock)
            {
                if (state == Net2State.STARTED)
                {
                    List<Net2HandlerBase> channels;

                    lock (@lock)
                    {
                        channels = channels_list.Values.ToList();
                    }

                    foreach (var item in channels)
                    {
                        item.Dispose();
                    }

                    // net2_service.Dispose();

                    state = Net2State.STOPPED;

                    net2_consul.removeService(NameSpace);
                    Net2NTPClient.delegateNewLogx -= delegateNewLog;
                    net2_config.name_space = "";

            
                }
                else
                {
                    throw new Exception("Init first");
                }
            }
        }

        void HandleDelegateNewLog(string log_message, LogType log_type, string section)
        {
        }


        private interface INet2Handler
        {
            void SetInstance(Net2Base instance);

            void ReleaseInstance();

            Net2Base Instance { get; }
        }
    }
}
