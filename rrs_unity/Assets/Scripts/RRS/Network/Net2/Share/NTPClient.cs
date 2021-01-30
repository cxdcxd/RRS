using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NtpClient;
using System.Diagnostics;
using RRS.Tools.Log;

namespace RRS.Tools.Network
{
    class Net2NTPClient
    {
        Stopwatch stop_watch;
        ulong time_offset = 0;
        string host_name;

        public delegate void DelegateNewLog(string log_message, LogType log_type, string section);
        public static event DelegateNewLog delegateNewLogx;

        protected void reportLog(string log_message, LogType log_type, string section)
        {
            delegateNewLogx?.Invoke(log_message, log_type, section);
        }

        public Net2NTPClient(string host_name, bool sync_ntp)
        {
            this.host_name = host_name;

            if ( sync_ntp )
                sync(this.host_name);
            else
            {
                stop_watch = new Stopwatch();
                stop_watch.Stop();
                var utcNow = DateTime.Now;
                stop_watch.Start();
                time_offset = (ulong)(utcNow - new DateTime(1970, 1, 1)).TotalMilliseconds;
            }

        }

        public ulong get()
        {
            ulong result = 0;

            double diff = stop_watch.Elapsed.TotalMilliseconds;

            result = (ulong)(diff) + time_offset;

            return result;
        }

        public void sync(string host_name)
        {
            try
            {
                var connection = new NtpConnection(host_name);
                stop_watch = new Stopwatch();
                stop_watch.Stop();
                var utcNow = connection.GetUtc();
                stop_watch.Start();
                time_offset = (ulong)(utcNow - new DateTime(1970, 1, 1)).TotalMilliseconds;
                reportLog("NTP Sync successful", LogType.INFO, "NTPClient");
                reportLog("Current time is : " + get(), LogType.INFO, "NTPClient");

            }
            catch (Exception e)
            {
                string msg = e.Message;
                reportLog("Error in time sync : " + msg, LogType.ERROR, "NTPClient");
            }
        }
}
}
