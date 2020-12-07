
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
        //public static Bitmap Resize(this Bitmap source_image, Size box_size)
        //{
        //    Size image_size = source_image.Size;
        //    if (image_size.Height <= box_size.Height && image_size.Width <= box_size.Width)
        //    {
        //        return source_image;
        //    }
        //    else
        //    {
        //        double height_rate = (double)image_size.Height / box_size.Height;
        //        double width_rate = (double)image_size.Width / box_size.Width;
        //        double rate = Math.Max(height_rate, width_rate);
        //        Bitmap bitmap = new Bitmap(source_image, (int)(image_size.Width / rate), (int)(image_size.Height / rate));
        //        return bitmap;
        //    }
        //}

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

        public static float getFountSize(int area_height, int area_width, float resize_factor = 0.35f)
        {
            float height = area_height;
            if ((float)area_height / area_width > resize_factor)
            {
                height = area_width * resize_factor;
            }
            float font_size =  (height * 0.62f) - 8;
            return (font_size > 8.25f) ? font_size : 8.25f;
        }

        public static byte[] GetBitmapDataFromUrl(string url)
        {
            try
            {
                WebClient wc = new WebClient();
                return wc.DownloadData(url);
            }
            catch (Exception e)
            {
                //throw new Exception(e.Message);
                return null;
            }
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

        public static string GetCurrencyFormat(this string value)
        {
            decimal.TryParse(value, out decimal result);
            return result.ToString("N0");
        }

        public static string ReplaceEnNumbersWithFa(string English_number)
        {
            return English_number.Replace('0', '۰').Replace('1', '۱').Replace('2', '۲').Replace('3', '۳')
                .Replace('4', '۴').Replace('5', '۵').Replace('6', '۶').Replace('7', '۷').Replace('8', '۸')
                .Replace('9', '۹');
        }
    }
}
