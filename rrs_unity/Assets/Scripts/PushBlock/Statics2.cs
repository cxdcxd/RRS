using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Xml.Serialization;
using UnityEngine;

public class Statics2
{
    public static Network<IRawData, SVector3> network_main;
    public static Network<HapticCommand, ObservationRL> network_real;
    public static RVector3 last_status;
    public static ObservationRL current_env;
    public static string remote_ip = "10.66.171.119"; //LINUX
    //public static string robot_ip = "10.66.171.182"; //Robot
    public static string robot_ip = "127.0.0.1"; //Robot
    public static string local_ip = "192.168.1.1";  //WINDOWS or MAC
    public static string path_log =  "c:\\Network\\";
}
