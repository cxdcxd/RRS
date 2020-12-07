/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    Autor: Edwin Babaians
    Organization: www.Arsamrobotics.com
    
    Release Note:

    Version 1.0.0
    - first initial release

 */

using RRS.Tools.Log;
using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using NtpClient;
using RRS.Tools;

namespace Roboland.Tools.Network
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
