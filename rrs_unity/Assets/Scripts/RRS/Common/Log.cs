using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using System.IO;
using System.Timers;
using RRS.Tools;

namespace RRS
{
    namespace Tools
    {
        namespace Log
        {
            public enum LogType
            {
                INFO,
                DEBUG,
                WARN,
                ERROR
            }

            public class Log
            {
                public DateTime time = DateTime.Now;
                public string info = "";
                public LogType type = LogType.INFO;
                public string section;

                /// <summary>
                /// Get string representaion of log object
                /// </summary>
                /// <returns></returns>
                public string getString()
                {
                    string result = "";
                    result = time.ToString() + " : " + "[" + section + "] " + "[" + type + "] " + info;
                    return result;
                }
            }

            /// <summary>
            /// Main class for managing log in applications
            /// </summary>
            public class LogManager
            {
                public string path;
                public string base_path;
                public int screen_log_limitation;
                public string last_error_string = "";

                List<Log> system_log;
                private object system_log_lock;

                List<Log> save_log;
                private object save_log_lock;

                private object file_lock;
                
                private Timer save_log_timer;

                public LogManager(string full_path, int buffer_limit = 1000, long max_file_size = 1048576, bool reset = false)
                {
                    this.screen_log_limitation = buffer_limit;
                    this.path = full_path;
                    this.base_path = Path.GetDirectoryName(full_path);

                    system_log = new List<Log>();
                    system_log_lock = new object();

                    save_log = new List<Log>();
                    save_log_lock = new object();

                    file_lock = new object();

                    screen_log_limitation = buffer_limit;

                    long fle_size = GetFileSize();
                    if (reset || fle_size > max_file_size)
                    {
                        Reset();
                    }

                    save_log_timer = new Timer();
                    save_log_timer.Interval = 10;
                    save_log_timer.Elapsed += Save_log_timer_Elapsed;
                    save_log_timer.Enabled = true;
                }

                private void Save_log_timer_Elapsed(object sender, ElapsedEventArgs e)
                {
                    Log log = null;

                    lock (save_log_lock)
                    {
                        if (save_log.Count > 0)
                        {
                            log = save_log[0];
                            save_log.RemoveAt(0);
                        }
                    }

                    if (log != null)
                    {
                        sdLog(log);
                    }

                }

                /// <summary>
                /// Return list of sections
                /// </summary>
                public List<string> GetSections()
                {

                    List<string> sections;
                    lock (system_log_lock)
                    {
                        sections = system_log.Select(x => x.section).GroupBy(x => x).Select(grp => grp.First()).ToList();
                    }
                    return sections;
                }

                /// <summary>
                /// Save log object to local storage
                /// </summary>
                /// <param name="log"></param>
                /// <param name="reset"></param>
                bool sdLog(Log log)
                {
                    try
                    {
                        if (!Directory.Exists(base_path))
                        {
                            Directory.CreateDirectory(base_path);
                        }
                        lock (file_lock)
                        {
                            using (FileStream fs = new FileStream(path, FileMode.Append, FileAccess.Write))
                            {
                                using (StreamWriter sw = new StreamWriter(fs))
                                {
                                    sw.WriteLine(log.time.ToString() + " : " + "[" + log.section + "] " + "[" + log.type + "] " + log.info);
                                }
                            }
                        }                        
                    }
                    catch ( Exception e )
                    {
                        last_error_string = e.Message;
                    }

                    return false;
                }

                /// <summary>
                /// Add new log to log list and save the log to local storage
                /// </summary>
                /// <param name="info"></param>
                /// <param name="type"></param>
                /// <param name="reset"></param>
                public void addLog(string info, LogType type, string section, bool save = true)
                {

                    Log log = new Log();
                    log.info = info;
                    log.time = DateTime.Now;
                    log.type = type;
                    log.section = section;


                    if (system_log.Count >= screen_log_limitation)
                    {
                        lock (system_log_lock)
                        {
                            system_log.RemoveAt(0);
                        }               
                    }

                    lock (system_log_lock)
                    {
                        system_log.Add(log);
                    }

                    if (save)
                    {
                        lock (save_log_lock)
                        {
                            save_log.Add(log);
                        }
                    }
                }

                private long GetFileSize()
                {
                    if (File.Exists(path))
                    {
                        lock (file_lock)
                        {
                            FileInfo fi = new FileInfo(path);
                            return fi.Length;
                        }
                    }
                    return 0;
                }

                public List<Log> GetLogs()
                {
                    lock (system_log_lock)
                    {
                        return system_log.AsReadOnly().ToList();
                    }
                }

                public string getSDLog()
                {
                    string log = "";

                    if (File.Exists(path))
                    {
                        lock (file_lock)
                        {
                            using (FileStream fs = new FileStream(path, FileMode.Open, FileAccess.Read))
                            {
                                using (StreamReader sr = new StreamReader(fs))
                                {
                                    log = sr.ReadToEnd();
                                }
                            }
                        }
                    }

                    return log;
                }

                public void Reset()
                {
                    lock (system_log_lock)
                    {
                        system_log.Clear();
                    }

                    lock (save_log_lock)
                    {
                        system_log.Clear();
                    }
                    
                    if (File.Exists(path))
                    {
                        lock (file_lock)
                        {
                            File.Delete(path);
                        }
                    }
                }
            }
        }
    }
}