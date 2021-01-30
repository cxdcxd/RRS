
using RRS.Tools.Log;
using System;
using System.Collections.Generic;

using System.IO;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Runtime.Serialization.Formatters.Binary;
using System.Text;

namespace RRS.Tools
{
    public interface IRRSData
    {
        string Id { get; set; }
    }

    public class ProcessResult
    {
        public ProcessResult()
        {

        }

        public ProcessResult(bool success, string message = "")
        {
            Success = success;
            Message = message;
        }

        public bool Success { get; set; }
        public object Result { get; set; }
        public Type ResultType { get; set; }
        public string Message { get; set; }
    }

    public static class Tools
    {
        public static bool CheckServiceAvailablity(string remote_ip, int remote_port)
        {
            bool result = true;
            using (TcpClient tcp = new TcpClient())
            {
                IAsyncResult ar = tcp.BeginConnect(remote_ip, remote_port, null, null);
                System.Threading.WaitHandle wh = ar.AsyncWaitHandle;
                try
                {
                    if (!ar.AsyncWaitHandle.WaitOne(TimeSpan.FromSeconds(2), false))
                    {
                        tcp.Close();
                        result = false;
                    }
                    else
                    {
                        tcp.EndConnect(ar);
                    }
                }
                catch
                {
                    result = false;
                }
                finally
                {
                    wh.Close();
                }
            }
            return result;
        }

        public static long getTicks(this DateTime date_time)
        {
            TimeSpan tp = date_time - new DateTime(1, 1, 1, 0, 0, 0);
            return tp.Ticks;
        }

        public static IList<T> Clone<T>(this IList<T> listToClone) where T : ICloneable
        {
            return listToClone.Select(item => (T)item.Clone()).ToList();
        }

        public static DateTime GetBegin(this DateTime date)
        {
            return new DateTime(date.Year, date.Month, date.Day);
        }

        public static DateTime GetEnd(this DateTime date)
        {
            return new DateTime(date.Year, date.Month, date.Day, 23, 59, 59);
        }

        public static string ToDbDate(this DateTime? date)
        {
            if (date != null)
            {
                string year = date.Value.Year.ToString();
                string month = date.Value.Month < 10 ? "0" + date.Value.Month : date.Value.Month.ToString();
                string day = date.Value.Day < 10 ? "0" + date.Value.Day : date.Value.Day.ToString();
                return year + "-" + month + "-" + day;
            }
            return "";
        }

        public static string ToDbDateTime(this DateTime date)
        {
            string year = date.Year.ToString();
            string month = (date.Month < 10) ? "0" + date.Month.ToString() : date.Month.ToString();
            string day = (date.Day < 10) ? "0" + date.Day.ToString() : date.Day.ToString();
            return year + "-" + month + "-" + day + " " + date.Hour + ":" + date.Minute + ":" + date.Second;
        }

        public static string GetTimeFromDouble(double amount)
        {
            int hours = (int)amount;
            double min_double = amount - hours;
            int min = (int)Math.Round(min_double * 60);
            string min_str = min.ToString();
            if (min < 10)
            {
                min_str = "0" + min_str;
            }
            return hours + ":" + min_str;
        }

        public static T DeserializeFromString<T>(string data)
        {
            byte[] b = Convert.FromBase64String(data);
            using (var stream = new MemoryStream(b))
            {
                var formatter = new BinaryFormatter();
                stream.Seek(0, SeekOrigin.Begin);
                return (T)formatter.Deserialize(stream);
            }
        }

        public static string SerializeToString<T>(T data)
        {
            using (var stream = new MemoryStream())
            {
                var formatter = new BinaryFormatter();
                formatter.Serialize(stream, data);
                stream.Flush();
                stream.Position = 0;
                return Convert.ToBase64String(stream.ToArray());
            }
        }




    }
}
