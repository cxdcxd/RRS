using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using NtpClient;
using System.Diagnostics;

namespace RRS.Tools.Network
{
    class Net2NTPClient
    {
        Stopwatch stop_watch;
        ulong time_offset = 0;
        string host_name;

        public Net2NTPClient(string host_name)
        {
            this.host_name = host_name;
            sync(this.host_name);
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
            //try
            //{
            //    var connection = new NtpConnection(host_name);
            //    stop_watch = new Stopwatch();
            //    stop_watch.Stop();
            //    var utcNow = connection.GetUtc();
            //    stop_watch.Start();
            //    time_offset = (ulong)(utcNow - new DateTime(1970, 1, 1)).TotalMilliseconds;
            //}
            //catch
            //{

            stop_watch = new Stopwatch();
            stop_watch.Stop();
            var utcNow = DateTime.Now;
            stop_watch.Start();
            time_offset = (ulong)(utcNow - new DateTime(1970, 1, 1)).TotalMilliseconds;
        //}
    }
}
}
