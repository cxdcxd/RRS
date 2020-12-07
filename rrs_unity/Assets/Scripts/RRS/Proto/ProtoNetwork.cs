using System;
using System.Collections.Generic;
using System.Text;
using ProtoBuf;

namespace RRS
{
    namespace Tools
    {
        namespace Protobuf
        {

            
            [ProtoContract]
            [ProtoInclude(106, typeof(CODeviceProtocol))]
            public class RRSDeviceProtocol
            {
                [ProtoMember(1)]
                public int id;
                [ProtoMember(2)]
                public float battery_rate;

                public RRSDeviceProtocol()
                {

                }
            }

            
            [ProtoContract]
            [ProtoInclude(101,typeof(ZoneProtocol))]
            [ProtoInclude(102, typeof(GameRobotProtocol))]
            public class CODeviceProtocol : RRSDeviceProtocol
            {
                [ProtoMember(1)]
                public string name;
                [ProtoMember(2)]
                public int connection_state;
                [ProtoMember(3)]
                public int signal_level;
                [ProtoMember(4)]
                public long connection_time;
                [ProtoMember(5)]
                public long time_span;

                public CODeviceProtocol()
                {

                }
            }

            
            [ProtoContract]
            [ProtoInclude(105, typeof(RobotProtocol))]
            public class ZoneProtocol : CODeviceProtocol
            {
                [ProtoMember(1)]
                public int type;
                [ProtoMember(2)]
                public NodeProtocol[] nodes;

                public ZoneProtocol()
                {

                }
            }

            
            [ProtoContract]
            public class NodeProtocol
            {
                [ProtoMember(1)]
                public int id;
                [ProtoMember(2)]
                public string name = "";
                [ProtoMember(3)]
                public int type = 0;
                [ProtoMember(4)]
                public float voltage;
                [ProtoMember(5)]
                public float temperature;
                [ProtoMember(6)]
                public SensorProtocol[] sensors;
                [ProtoMember(7)]
                public ActuatorProtocol[] actuators;
                [ProtoMember(8)]
                public NodeDataProtocol[] data;
                [ProtoMember(9)]
                public int mode = 0;
                [ProtoMember(10)]
                public KeyValue[] auto_parameters;
                [ProtoMember(11)]
                public int error = 0;
                [ProtoMember(12)]
                public int state = 0;
                [ProtoMember(13)]
                public int command = 0;
                [ProtoMember(14)]
                public bool is_ok = false;

                public NodeProtocol()
                {

                }
            }

            
            [ProtoContract]
            public class ActuatorProtocol
            {
                [ProtoMember(1)]
                public string name = "";
                [ProtoMember(2)]
                public int id = 0;
                [ProtoMember(3)]
                public int type = 0;
                [ProtoMember(4)]
                public float fvalue = 0;
                [ProtoMember(5)]
                public int ivalue = 0;
                [ProtoMember(6)]
                public int[] ivalues;

                public ActuatorProtocol()
                {

                }
            }

            
            [ProtoContract]
            public class SensorProtocol
            {
                [ProtoMember(1)]
                public string name = "";
                [ProtoMember(2)]
                public int type = 0;
                [ProtoMember(3)]
                public float fvalue = 0;
                [ProtoMember(4)]
                public int ivalue = 0;
                [ProtoMember(5)]
                public int[] ivalues;

                public SensorProtocol()
                {

                }
            }

            
            [ProtoContract]
            public class NodeDataProtocol
            {
                [ProtoMember(1)]
                public string name = "";
                [ProtoMember(2)]
                public int type = 0;
                [ProtoMember(3)]
                public int error = 0;
                [ProtoMember(4)]
                public int state = 0;
                [ProtoMember(5)]
                public bool bvalue;
                [ProtoMember(6)]
                public int ivalue;
                [ProtoMember(7)]
                public float fvalue;
                [ProtoMember(8)]
                public int[] ivalues;

                public NodeDataProtocol()
                {

                }
            }

            
            [ProtoContract]
            [ProtoInclude(103, typeof(REVCOBasedProtocol))]
            [ProtoInclude(104, typeof(ParrotProtocol))]
            public class GameRobotProtocol : CODeviceProtocol
            {
                [ProtoMember(1)]
                public int product_id;
              
                public GameRobotProtocol()
                {

                }
            }

            
	
            [ProtoContract]
            public class REVCOBasedProtocol : GameRobotProtocol
	
            {	
                [ProtoMember(1)]	
                public int current_led;	
	
                public REVCOBasedProtocol()	
                {	
	
                }
                public REVCOBasedProtocol(REVProtocol rev_porotocol)
                {
                    this.id = rev_porotocol.id;
                    this.name = rev_porotocol.name;
                    this.connection_state = rev_porotocol.connection_state;
                    this.signal_level = rev_porotocol.signal_level;
                    this.connection_time = rev_porotocol.connection_time;
                    this.time_span = rev_porotocol.time_span;
                    this.battery_rate = rev_porotocol.battery_rate;
                    this.product_id = rev_porotocol.product_id;
                    this.current_led = rev_porotocol.current_led;
                }
            }



            [ProtoContract]
            public class REVProtocol 
            {
                [ProtoMember(1)]
                public int product_id;
                [ProtoMember(2)]
                public int current_led;
                [ProtoMember(3)]
                public float battery_rate;
                [ProtoMember(4)]
                public string name;
                [ProtoMember(5)]
                public int connection_state;
                [ProtoMember(6)]
                public int signal_level;
                [ProtoMember(7)]
                public long connection_time;
                [ProtoMember(8)]
                public long time_span;
                [ProtoMember(9)]
                public int id;

                public REVProtocol()
                {

                }

                public REVProtocol(REVCOBasedProtocol rev_co_based_protocol)
                {
                    this.id = rev_co_based_protocol.id;
                    this.name = rev_co_based_protocol.name;
                    this.connection_state = rev_co_based_protocol.connection_state;
                    this.signal_level = rev_co_based_protocol.signal_level;
                    this.connection_time = rev_co_based_protocol.connection_time;
                    this.time_span = rev_co_based_protocol.time_span;
                    this.battery_rate = rev_co_based_protocol.battery_rate;
                    this.product_id = rev_co_based_protocol.product_id;
                    this.current_led = rev_co_based_protocol.current_led;
                }
            }

            
            [ProtoContract]
            public class ParrotProtocol : GameRobotProtocol
            {
                [ProtoMember(1)]
                public string alert;
                [ProtoMember(2)]
                public int rtsp_port;
                [ProtoMember(3)]
                public int outdoor;
                [ProtoMember(4)]
                public string posture;
                [ProtoMember(5)]
                public string mac_address;

                public ParrotProtocol()
                {

                }

            }


            [ProtoContract]
            public class REVGDIProtocol
            {
                //TODO
                //Because the REV sdk cant build with android sdk above 5.1 and protobuf inheritance in v 2.3.3 cant work with Xamarin android sdk lower than 7.0 we have to do this :|

                [ProtoMember(1)]
                public string mmajor = "";
                [ProtoMember(2)]
                public string mminor = "";
                [ProtoMember(3)]
                public string param1 = "";
                [ProtoMember(4)]
                public string param2 = "";
                [ProtoMember(5)]
                public string param3 = "";
                [ProtoMember(6)]
                public string param4 = "";
                [ProtoMember(7)]
                public string param5 = "";
                [ProtoMember(8)]
                public string remote_ip = "";
                [ProtoMember(9)]
                public int version;
                [ProtoMember(10)]
                public REVProtocol[] list_devices;

                public REVGDIProtocol()
                {

                }

                public REVGDIProtocol(string major, string minor, string p1 = "", string p2 = "", string p3 = "", string p4 = "", string p5 = "")
                {
                    this.mmajor = major;
                    this.mminor = minor;
                    this.param1 = p1;
                    this.param2 = p2;
                    this.param3 = p3;
                    this.param4 = p4;
                    this.param5 = p5;
                }

                public REVGDIProtocol(RRSMainProtocol protocol)
                {
                    this.mmajor = protocol.mmajor;
                    this.mminor = protocol.mminor;
                    this.param1 = protocol.param1;
                    this.param2 = protocol.param2;
                    this.param3 = protocol.param3;
                    this.param4 = protocol.param4;
                    this.param5 = protocol.param5;
                    this.remote_ip = protocol.remote_ip;
                    this.version = protocol.version;
                    int devices_num = 0;
                    if (protocol.list_devices != null)
                    {
                        devices_num = protocol.list_devices.Length;
                    }
                    if (devices_num > 0)
                    {
                        this.list_devices = new REVProtocol[devices_num];
                        for (int i = 0; i < this.list_devices.Length; i++)
                        {
                            this.list_devices[i] = new REVProtocol((REVCOBasedProtocol)protocol.list_devices[i]);
                        }
                    }
                }
            }

            [ProtoContract]
            public class RobotProtocol :  ZoneProtocol 
            {
                [ProtoMember(1)]
                public RVector3 pose;
                [ProtoMember(2)]
                public RVector3 vel;
                [ProtoMember(3)]
                public int state;
                [ProtoMember(4)]
                public RRSBaseProtocol face;
                [ProtoMember(5)]
                public string remote_ip;
                [ProtoMember(6)]
                public RVector3 grid;
                [ProtoMember(7)]
                public bool has_food;
                [ProtoMember(8)]
                public RVector3 target_location;
                [ProtoMember(9)]
                public RVector3 temp_target_location;
                [ProtoMember(10)]
                public RVector3[] path;
                [ProtoMember(11)]
                public RVector3[] temp_path;
                [ProtoMember(12)]
                public int order_id = 0;
                [ProtoMember(13)]
                public int priority = 0;
                [ProtoMember(14)]
                public RRSLaser laser;
                [ProtoMember(15)]
                public RRSCollision collision;
                [ProtoMember(16)]
                public float temperature;
                [ProtoMember(17)]
                public bool face_local_network;
                [ProtoMember(18)]
                public bool face_remote_network;
                [ProtoMember(19)]
                public bool face_is_ok;
                [ProtoMember(20)]
                public string wifi_essid;
                [ProtoMember(21)]
                public string wifi_bssid;
                [ProtoMember(22)]
                public string wifi_channel;
                [ProtoMember(23)]
                public string wifi_quality;

                public RobotProtocol()
                {


                }
            }

            [ProtoContract]
            public class RRSRobot 
            {
                [ProtoMember(1)]
                public RVector3 target_location;
                [ProtoMember(2)]
                public RVector3 temp_target_location;
                [ProtoMember(3)]
                public RVector3[] path;
                [ProtoMember(4)]
                public RVector3[] temp_path;
              
                public RRSRobot()
                {


                }
            }
        }
    }
}