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
    public static string remote_ip = "10.152.13.39"; //LINUX
    public static string robot_ip = "10.152.13.39"; //Robot
    public static string local_ip = "10.152.13.17";  //WINDOWS or MAC
    public static string path_log = "/home/lmt";
}
