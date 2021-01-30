using RRS.Tools.Log;
using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using NtpClient;
using RRS.Tools;

namespace RRS.Tools.Network
{
    class Net2NTP
    {
        string path;
        DateTime ntp_time;
        NtpConnection connection;

        public delegate void DelegateNewLog(string log_message, LogType log_type, string section);
        public event DelegateNewLog delegateNewLog;

        public string Path
        {
            get { return path; }
        }

        public DateTime lastNTPTime
        {
            get { return ntp_time; }
        }

        public Net2NTP(string ntpServer)
        {
            var connection = new NtpConnection(ntpServer);
        }

        public ProcessResult Sync()
        {
            ProcessResult result = new ProcessResult();

            try
            {
                ntp_time = connection.GetUtc();
            }
            catch (Exception e)
            {
                result.Success = false;
                result.Message = e.Message;
            }

            return result;
        }

    }
}
