using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;
using Consul;
using RRS.Tools;
using System.Threading;
using System.Linq;
using System.Timers;

namespace RRS.Tools.Network
{
    public class Net2Consul
    {
        private System.Timers.Timer main_timer;

        private List<AgentService> updated_list;
        private List<AgentService> temp_updated_list;
        private List<AgentService> sending_queue;

        private ConsulClient client;
        private object mutex_send;
        private object mutex_get;
        private Net2ConsulMode consul_mode;

        private int logic_counter = 0;

        public Net2Consul(string uri, Net2ConsulMode mode)
        {
            mutex_get = new object();
            mutex_send = new object();
            consul_mode = mode;

            updated_list = new List<AgentService>();
            temp_updated_list = new List<AgentService>();
            sending_queue = new List<AgentService>();

            ConsulClientConfiguration config = new ConsulClientConfiguration();
            config.Address = new Uri(uri);

            client = new ConsulClient(config);
            
            main_timer = new System.Timers.Timer();
            main_timer.Interval = 1000;
            main_timer.Elapsed += Main_timer_Elapsed;
            main_timer.Start();
            main_timer.Enabled = true;
        }

        private void sendServices()
        {
            lock (mutex_send)
            {
                for (int i = 0; i < sending_queue.Count; i++)
                {
                    AgentService info = sending_queue[i];
                    AgentServiceRegistration service = new AgentServiceRegistration();

                    service.Address = info.Address;
                    service.ID = info.ID;
                    service.Name = info.ID;
                    service.Port = info.Port;
                    service.Tags = info.Tags;

                    client.Agent.ServiceRegister(service).ContinueWith(a => {

                        if ( a.IsCompleted && a.IsFaulted == false)
                        {

                        }
                        else
                        {

                        }

                    });
                }
            }
        }

        private List<AgentService> updateList(Dictionary<string, AgentService> target)
        {
            List<AgentService> desire = new List<AgentService>();

            lock (mutex_get)
            {
                desire.Clear();

                foreach (var item in target.Values)
                {
                    desire.Add(item);
                }

            }

            return desire;
        }

        private void Main_timer_Elapsed(object sender, ElapsedEventArgs e)
        {
            if (consul_mode == Net2ConsulMode.CLIENT)
            {
                sendServices();
                client.Agent.Services().ContinueWith(ax =>
                {

                    if (ax.IsCompleted && ax.IsFaulted == false)
                    {
                        var list = ax.Result.Response;
                        lock (mutex_get)
                        {
                            updated_list = updateList(list);
                        }
                    }
                });


            }
            else if (consul_mode == Net2ConsulMode.MANAGER)
            {
                logic_counter++;

                if (logic_counter == 1)
                {
                    client.Agent.Services().ContinueWith(ax =>
                    {
                        if (ax.IsCompleted && ax.IsFaulted == false)
                        {
                            var list = ax.Result.Response;
                            lock (mutex_get)
                            {
                                updated_list = updateList(list);
                            }
                        }
                    });
                }
                else if (logic_counter == 5)
                {
                    IReadOnlyList<AgentService> uodate_list_readonly;
                    lock (mutex_get)
                    {
                        uodate_list_readonly = updated_list.AsReadOnly();
                    }
                    logic_counter = 0;

                    sendServices();

                    client.Agent.Services().ContinueWith(ax =>
                    {
                        if (ax.IsCompleted && ax.IsFaulted == false)
                        {
                            var list = ax.Result.Response;
                            temp_updated_list = updateList(list);

                            for (int i = 0; i < uodate_list_readonly.Count; i++)
                            {
                                string target_name = uodate_list_readonly[i].ID;

                                for (int j = 0; j < temp_updated_list.Count; j++)
                                {
                                    if (temp_updated_list[j].ID == target_name && target_name != "net2manager-net2status" && target_name != "net2manager")
                                    {
                                        string a = uodate_list_readonly[i].Tags[0];
                                        string b = temp_updated_list[j].Tags[0];

                                        long old_time = long.Parse(a);
                                        long new_time = long.Parse(b);

                                        long diff = new_time - old_time;

                                        if (diff == 0)
                                            client.Agent.ServiceDeregister(target_name);

                                    }
                                }
                            }
                        }
                    });
                }
            }
        }

        #region Service

        public AgentService getServiceInfo(string name)
        {
            AgentService info = null;

            lock (mutex_get)
            {
              
                foreach (var item in updated_list)
                {
                    if (item.ID == name)
                    {
                        info = item;
                        break;
                    }
                }
            }

            return info;
        }

        public void advertiseService(string name, int port, string address, string time)
        {
            lock (mutex_send)
            {

                int index = -1;
                for (int i = 0; i < sending_queue.Count; i++)
                {
                    if (sending_queue[i].ID == name)
                    {
                        index = i;
                        break;
                    }
                }

                if (index == -1)
                {
                    AgentService info = new AgentService();
                    info.ID = name;
                    info.Port = port;
                    info.Address = address;
                    info.Tags = new string[] { time, "time"};

                    sending_queue.Add(info);
                }
                else
                {
                    sending_queue[index].Tags = new string[] {time , "time"};
                    sending_queue[index].Port = port;
                    sending_queue[index].Address = address;
                }
            }
        }

        public void removeService(string name)
        {
            lock (mutex_send)
            {
                int index = -1;
                for (int i = 0; i < sending_queue.Count; i++)
                {
                    if (sending_queue[i].ID == name)
                    {
                        index = i;
                        break;
                    }
                }

                if (index != -1)
                    sending_queue.RemoveAt(index);
            }
        }
        #endregion

    }
}
