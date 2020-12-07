using Roboland.Tools;
using RRS.Tools.Log;
using RRS.Tools.Protobuf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.NetworkInformation;
using System.Text;
using System.Timers;

namespace RRS.Tools.Network
{
    public class Net2Ping
    {
        //Ping ping;
        //Timer main_timer;
        //Net2NetworkType type = Net2NetworkType.PING;
        //Net2State ping_state = Net2State.STOPPED;

        //int id;
        //string name;
        //string section;
        //int ping_timeout_sec = 2;
        //int ping_check_interval_sec = 2;
        //ulong current_ping_roundtrip_time = 0;
        //string remote_ip;

        //public int Id
        //{
        //    get
        //    {
        //        return id;
        //    }
        //}
        //public string Name
        //{
        //    get
        //    {
        //        return name;
        //    }
        //}

        //public delegate void DelegateNewLog(string log_message, LogType log_type, string section);
        //public event DelegateNewLog delegateNewLog;

        //public delegate void DelegateStateChanged(Net2State state, Net2EventArgs args);
        //public event DelegateStateChanged delegateStateChanged;

        //void setState(Net2State new_state)
        //{
        //    if (this.ping_state != new_state)
        //    {
        //        this.ping_state = new_state;
        //        reportState(new_state, new Net2EventArgs(name,type));
        //    }
        //}

        //void reportState(Net2State new_state, Net2EventArgs args)
        //{
        //    delegateStateChanged?.Invoke(new_state, args);
        //}

        //public Net2State State
        //{
        //    get { return ping_state; }
        //}

        //public ProcessResult Start(string remote_ip)
        //{
        //    ProcessResult result = new ProcessResult();

        //    if ( ping_state != Net2State.STOPPED)
        //    {
        //        result.Success = false;
        //        result.Message = "Invalid state";
        //        return result;
        //    }

        //    try
        //    {
        //        this.remote_ip = remote_ip;

        //        main_timer.Start();
        //        main_timer.Enabled = true;

        //        result.Success = true;
        //        result.Message = "Done";

        //        setState(Net2State.STARTED);
        //    }
        //    catch ( Exception e)
        //    {
        //        result.Success = false;
        //        result.Message = e.Message;
        //    }

        //    return result;
        //}

        //public ProcessResult Stop()
        //{
        //    ProcessResult result = new ProcessResult();

        //    if (ping_state != Net2State.STARTED)
        //    {
        //        result.Success = false;
        //        result.Message = "Invalid state";
        //        return result;
        //    }

        //    try
        //    {
        //        main_timer.Stop();
        //        main_timer.Enabled = false;

        //        setState(Net2State.STOPPED);
        //    }
        //    catch (Exception e)
        //    {
        //        result.Success = false;
        //        result.Message = e.Message;
        //    }

        //    return result;
        //}

        //public Net2Ping()
        //{
        //    section = this.GetType().Name;

        //    ping = new Ping();
        //    ping.PingCompleted += Ping_PingCompleted;

        //    main_timer = new Timer();
        //    main_timer.Elapsed += Main_timer_Elapsed;
        //    main_timer.Interval = ping_check_interval_sec * 1000;

        //    setState(Net2State.STOPPED);
        //}

        //protected void reportLog(string log_message, LogType log_type, string section)
        //{
        //    delegateNewLog?.Invoke(log_message, log_type, section);
        //}

        //private void Ping_PingCompleted(object sender, PingCompletedEventArgs e)
        //{
        //    try
        //    {
        //        if (e.Reply.Status == IPStatus.Success)
        //        {
        //            current_ping_roundtrip_time = (ulong)e.Reply.RoundtripTime;
        //        }
        //        else
        //        {
        //            current_ping_roundtrip_time = (ulong)ping_timeout_sec * 1000;
        //        }
        //    }
        //    catch (Exception ee)
        //    {
        //        reportLog(ee.Message, LogType.ERROR, section);
        //        current_ping_roundtrip_time = (ulong)ping_timeout_sec * 1000;
        //    }

        //    ping_state = Net2State.STARTED;
        //}

        //private void Main_timer_Elapsed(object sender, ElapsedEventArgs e)
        //{
        //    if (!string.IsNullOrEmpty(remote_ip) && ping_state != Net2State.BUSY)
        //    {
        //        ping_state = Net2State.BUSY;
        //        ping.SendAsync(remote_ip, ping_timeout_sec, null);
        //    }
              
        //}
    }
}
