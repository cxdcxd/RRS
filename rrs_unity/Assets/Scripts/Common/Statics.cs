using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Roboland.Tools.Network;
using RRS.Tools.Log;
using RRS.Tools.DataManagerXML;
using RRS.Tools;
using System.IO;
using RRS.Tools.Network;

public class Statics
{
    public static Statics instance = null;
    public static readonly object padlock = new object();
    public Net2Config net2_config;
    public LogManager log_manager;
    public DataManagerXML xml_manager;
    public static Config current_config;
    public static string user_name = "Edwin";

    public class Config
    {
        public string consul_network_address = "192.168.92.139";
        public string local_network_address = "192.168.92.1";
        public uint consul_network_port = 8500;
        public string consul_network_mask = "255.255.255.0";
        public string ntp_server_host_name = "8500";
        public bool use_relative_origin = false;
    }

    private static Statics Instance
    {
        get
        {
            lock (padlock)
            {
                if (instance == null)
                {
                    instance = new Statics();
                }
                return instance;
            }
        }
    }

    private void Initialize()
    {
        string folder_path = "c:\\Users\\" + user_name + "\\Documents\\WorkSpace\\RRS\\Config\\";
        log_manager = new LogManager(folder_path + "log.txt", 1000, 1048576, true);
        xml_manager = new DataManagerXML();

        if (File.Exists(folder_path + "config.xml") == false)
            xml_manager.saveXML<Config>(folder_path + "config.xml", new Config());

        ProcessResult result = xml_manager.loadXML<Config>(folder_path + "config.xml");
        current_config = (Config)result.Result;

        

        net2_config = new Net2Config();
        net2_config.consul_mode = RRS.Tools.Network.Net2ConsulMode.CLIENT;

        net2_config.consul_network_address = current_config.consul_network_address;
        net2_config.consul_network_port = current_config.consul_network_port;
        net2_config.consul_network_mask = current_config.consul_network_mask;
        net2_config.ntp_server_host_name = current_config.ntp_server_host_name;
        net2_config.local_network_address = current_config.local_network_address;
        net2_config.ntp_server_port = 123;

        net2_config.name_space = "rrs";

        log_manager.addLog("rrs started", RRS.Tools.Log.LogType.INFO, "main");

        Net2.Init(net2_config);
        Net2._delegateInfoServiceNewLog += Net2__delegateInfoServiceNewLog;

    }

    private void Net2__delegateInfoServiceNewLog(string log_message, RRS.Tools.Log.LogType log_type, string section)
    {
        log_manager.addLog(log_message, RRS.Tools.Log.LogType.DEBUG, "Net2");
    }

    private void InternalShutdown()
    {
        log_manager.addLog("rrs shutting down...", RRS.Tools.Log.LogType.INFO, "main");
        
        Net2.Shutdown();
      
        log_manager.addLog("rrs shutdown done", RRS.Tools.Log.LogType.INFO, "main");
    }

    public static void Init()
    {
        Instance.Initialize();
    }

    public static void Shutdown()
    {
        Instance.InternalShutdown();
    }
}
