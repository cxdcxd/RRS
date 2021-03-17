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
    class Net2Time
    {
        public long sec;
        public long nsec;
    }

    class Net2NTPClient
    {
        public static DateTime UNIX_EPOCH = new DateTime(1970, 1, 1, 0, 0, 0, 0, DateTimeKind.Utc);

        Stopwatch stop_watch;
        long time_offset = 0;
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
                time_offset = (long)(utcNow - new DateTime(1970, 1, 1)).TotalMilliseconds;
            }

        }

        public Net2Time get()
        {
            Net2Time time = new Net2Time();
          
            //double diff = stop_watch.Elapsed.TotalMilliseconds;
            //result = (long)(diff) + time_offset;
          
            TimeSpan timeSpan = DateTime.Now.ToUniversalTime() - UNIX_EPOCH;
            double msecs = timeSpan.TotalMilliseconds;
            time.sec = (uint)(msecs / 1000);
            time.nsec = (uint)((msecs / 1000 - time.sec) * 1e+9);
        
            return time;
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
                time_offset = (long)(utcNow - new DateTime(1970, 1, 1)).TotalMilliseconds;
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
