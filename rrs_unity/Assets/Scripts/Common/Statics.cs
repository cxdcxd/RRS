using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RRS.Tools.Log;
using RRS.Tools.DataManagerXML;
using RRS.Tools;
using System.IO;
using RRS.Tools.Network;
using RRS.Tools.Protobuf;

public class Statics
{
    #region Net1

    public static Network<HapticCommand, HapticRender> network_manager_left_arm; //Send NMPC command and receive force
    public static Network<HapticCommand, HapticRender> network_manager_right_arm; //Send NMPC command and receive force
    public static Network<HapticCommand, HapticRender> network_manager_arm; //Send NMPC command and receive force for franka
    public static Network<RRSNull, MovoStatus> network_manager_movo_status; //Receive Movo status

    public static Network<RVector7, RVector7> main_tele_network;
    public static Network<RRSCPDCommand, RRSCPDResult> main_cpd_network;

    #endregion

    public static Statics instance = null;
    public static readonly object padlock = new object();
    public Net2Config net2_config;
    public LogManager log_manager;
    public DataManagerXML xml_manager;
    public static Config current_config;
    public static Environments current_environment = Environments.Sim;
    public static Movo movo_ref;
    public static MovoMini movo_mini_ref;
    public static Franka franka_ref;
    public static CPDManager cpd_manager_ref;
    
    public static float right_container_distance = 0.16f;
    public static float left_container_distance = 0.24f;

    public enum Environments
    {
        Real,
        Sim
    }

    public class Config
    {
        public string consul_network_address = "127.0.0.1";
        public string local_network_address = "127.0.0.1";
        public uint consul_network_port = 8500;
        public string consul_network_mask = "255.255.255.0";
        public string ntp_server_host_name = "8500";
        public bool use_relative_origin = false;
        public bool is_real = false;
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
        string p1 = Application.dataPath;
        string folder_path = p1.Replace("/Assets", "/Config/");

        Directory.CreateDirectory(folder_path);

        log_manager = new LogManager(folder_path + "log.txt", 1000, 1048576, true);
        xml_manager = new DataManagerXML();

        if (File.Exists(folder_path + "config.xml") == false)
            xml_manager.saveXML<Config>(folder_path + "config.xml", new Config());

        ProcessResult result = xml_manager.loadXML<Config>(folder_path + "config.xml");
        current_config = (Config)result.Result;

        if (current_config.is_real) current_environment = Environments.Real;

        Debug.Log("Current mode " + current_environment.ToString());

        if (current_environment == Environments.Sim)
        {
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

            Net2.delegateInfoServiceNewLog += Net2__delegateInfoServiceNewLog;
            Net2.Init(net2_config);

            
        }
        else
        {
          

            network_manager_left_arm = new Network<HapticCommand, HapticRender>("10.66.171.167", "10.66.171.182", "4160", "4161", "", NetworkType.PUBSUB, "LeftArm");
            network_manager_left_arm.eventDataUpdated += Network_manager_left_arm_eventDataUpdated;
            network_manager_right_arm = new Network<HapticCommand, HapticRender>("10.66.171.167", "10.66.171.182", "4150", "4151", "", NetworkType.PUBSUB, "RightArm");
            network_manager_right_arm.eventDataUpdated += Network_manager_right_arm_eventDataUpdated;
            network_manager_movo_status = new Network<RRSNull, MovoStatus>("10.66.171.167", "10.66.171.182", "4170", "4171", "", NetworkType.PUBSUB, "MovoStatus");
            network_manager_movo_status.eventDataUpdated += Network_manager_movo_status_eventDataUpdated;

            

        }

        //CPD Part is common for both Sim and Real
        Statics.main_tele_network = new Network<RVector7, RVector7>("127.0.0.1", "127.0.0.1", "9870", "9871", "0", NetworkType.PUBSUB, "Tele", false);
        Statics.main_tele_network.eventDataUpdated += Main_tele_network_eventDataUpdated;
        Statics.main_cpd_network = new Network<RRSCPDCommand, RRSCPDResult>("127.0.0.1", "127.0.0.1", "9872", "9873", "0", NetworkType.PUBSUB, "CPD", false);
        Statics.main_cpd_network.eventDataUpdated += Main_cpd_network_eventDataUpdated;
       
    }

    private void Main_tele_network_eventDataUpdated()
    {
        cpd_manager_ref.Main_tele_network_eventDataUpdated();
    }

    private void Main_cpd_network_eventDataUpdated()
    {
        cpd_manager_ref.Main_cpd_network_eventDataUpdated();
        
    }

    private void Network_manager_movo_status_eventDataUpdated()
    {
       
        if (movo_ref != null)
            movo_ref.Network_manager_movo_status_eventDataUpdated(network_manager_movo_status.get);
    }

    private void Network_manager_right_arm_eventDataUpdated()
    {
        if (movo_ref != null)
            movo_ref.Network_manager_right_arm_eventDataUpdated(network_manager_right_arm.get);
    }

    private void Network_manager_left_arm_eventDataUpdated()
    {
        if (movo_ref != null)
            movo_ref.Network_manager_left_arm_eventDataUpdated(network_manager_left_arm.get);
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
        if ( Statics.current_environment == Environments.Sim)
        Instance.InternalShutdown();
    }
}
