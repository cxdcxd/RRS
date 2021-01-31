using ProtoBuf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RRS.Tools.Protobuf
{

    [ProtoContract]
    public class RRSHeader
    {
        [ProtoMember(1)]
        public ulong time = 0;
        [ProtoMember(2)]
        public ulong sequence = 0;

        public RRSHeader()
        {

        }
    }

    [ProtoContract]
    public class Net2StationInfo
    {
        [ProtoMember(1)]
        public string host_name;
        [ProtoMember(2)]
        public string ip;
        [ProtoMember(3)]
        public Net2ServiceInfo[] items;
      
        public Net2StationInfo()
        {

        }
    }

    [ProtoContract]
    public class Net2ServiceInfo
    {
        [ProtoMember(1)]
        public string name;
        [ProtoMember(2)]
        public string network_type;
        [ProtoMember(3)]
        public string local_ip;
        [ProtoMember(4)]
        public string remote_ip;
        [ProtoMember(5)]
        public string local_port;
        [ProtoMember(6)]
        public string remote_port;
        [ProtoMember(7)]
        public string bytes_sent;
        [ProtoMember(8)]
        public string bytes_received;
        [ProtoMember(9)]
        public string connection_count;
        [ProtoMember(10)]
        public string state;
        [ProtoMember(11)]
        public string remote_path;

        public Net2ServiceInfo()
        {

        }
    }

    [ProtoContract]
    public class Message
    {
        [ProtoMember(1)]
        public Header header;
        [ProtoMember(2)]
        public byte[] payload;
        
        public Message()
        {
         
        }
    }

    [ProtoContract]
    public class Header
    {
        public enum Type
        {
            DEALER = 0,
            REQUESTER = 1,
            PUBLISHER = 2,
            SUBSCRIBER = 3,
            ROUTER = 4,
        }

        public enum Mode
        {
            COMMAND,
            ACK,
            HEARTBEAT
        }

        [ProtoMember(1)]
        public Type message_type;
        [ProtoMember(2)]
        public Mode mode;
        [ProtoMember(3)]
        public string source_channel_name;
        [ProtoMember(4)]
        public string remote_channel_name;
        [ProtoMember(5)]
        public long time_span = 0;
        [ProtoMember(6)]
        public long sequence = 0;
        [ProtoMember(7)]
        public string source_ip = "";
        [ProtoMember(8)]
        public string remote_ip = "";
        [ProtoMember(9)]
        public uint priority = 0;
        [ProtoMember(10)]
        public byte[] zmq_router_address;

        public Header()
        {

        }
    }

    [ProtoContract]
    public class RVector3
    {
        [ProtoMember(1)]
        public float x = 0;
        [ProtoMember(2)]
        public float y = 0;
        [ProtoMember(3)]
        public float theta = 0;

        public RVector3(float a, float b, float c)
        {
            x = a;
            y = b;
            theta = c;
        }

        public RVector3()
        {

        }
    }

    
    [ProtoContract]
    public class SVector3
    {
        [ProtoMember(1)]
        public float x = 0;
        [ProtoMember(2)]
        public float y = 0;
        [ProtoMember(3)]
        public float z = 0;

        public SVector3(float a, float b, float c)
        {
            x = a;
            y = b;
            z = c;
        }

        public SVector3()
        {

        }
    }

    [ProtoContract]
    public class SVector6
    {
        [ProtoMember(1)]
        public float x = 0;
        [ProtoMember(2)]
        public float y = 0;
        [ProtoMember(3)]
        public float z = 0;

        [ProtoMember(4)]
        public float vx = 0;
        [ProtoMember(5)]
        public float vy = 0;
        [ProtoMember(6)]
        public float vz = 0;

        public SVector6()
        {

        }
    }

    [ProtoContract]
    public class SVector4
    {
        [ProtoMember(1)]
        public float x = 0;
        [ProtoMember(2)]
        public float y = 0;
        [ProtoMember(3)]
        public float z = 0;
        [ProtoMember(4)]
        public float w = 0;

        public SVector4(float a, float b, float c,float d)
        {
            x = a;
            y = b;
            z = c;
            w = d;
        }

        public SVector4()
        {

        }
    }


    [ProtoContract]
    public class RRSNull
    {
        public RRSNull()
        { }
    }

    
    [ProtoContract]
    public class RawData : IRawData
    {
        [ProtoMember(1)]
        public byte[] data;
        [ProtoMember(2)]
        public int length;

        public byte[] Data
        {
            get
            {
                return data;
            }

            set
            {
                data = value;
            }
        }

        public int Length
        {
            get
            {
                return length;
            }

            set
            {
                length = value;
            }
        }
    }

    

   

    [ProtoContract]
    [ProtoInclude(100, typeof(RRSMainProtocol))]
    public class RRSBaseProtocol : IData 
    {
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

        public RRSBaseProtocol()
        {
                    
        }

        public RRSBaseProtocol(string major, string minor, string p1 = "", string p2 = "", string p3 = "", string p4 = "", string p5 = "")
        {
            this.mmajor = major;
            this.mminor = minor;
            this.param1 = p1;
            this.param2 = p2;
            this.param3 = p3;
            this.param4 = p4;
            this.param5 = p5;
        }

        public int Version
        {
            get
            {
                return version;
            }

            set
            {
                version = value;
            }
        }
    }

   


    [ProtoContract]
    public class RRSMainProtocol : RRSBaseProtocol
    {
        [ProtoMember(1)]
        public RRSDeviceProtocol[] list_devices;

        public RRSMainProtocol(REVGDIProtocol protocol)
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
                this.list_devices = new REVCOBasedProtocol[devices_num];
                for (int i = 0; i < this.list_devices.Length; i++)
                {
                    this.list_devices[i] = new REVCOBasedProtocol(protocol.list_devices[i]);
                }
            }
        }

        public RRSMainProtocol(string major, string minor, string p1 = "", string p2 = "", string p3 = "", string p4 = "", string p5 = "") : base(major, minor, p1, p2, p3, p4, p5)
        {

        }

        public RRSMainProtocol()
        {

        }
    }

    [ProtoContract]
    public class RRSConnection
    {
        [ProtoMember(1)]
        public string ip = "";
        [ProtoMember(2)]
        public string status = "";
        [ProtoMember(3)]
        public ulong time = 0;

        public RRSConnection(string ip, string status = "")
        {
            this.ip = ip;
            this.status = status;
        }

        public RRSConnection()
        {

        }
    }

    
    [ProtoContract]
    public class LEDProtocol
    {
        [ProtoMember(1)]
        public int red = 0;
        [ProtoMember(2)]
        public int green = 0;
        [ProtoMember(3)]
        public int blue = 0;
        [ProtoMember(4)]
        public int alpha = 0;

        public LEDProtocol()
        {

        }
    }

   

    [ProtoContract]
    public class KeyValue
    {
        [ProtoMember(1)]
        string key;
        [ProtoMember(2)]
        string value;

        public KeyValue()
        {

        }

        public KeyValue(string key, string value)
        {
            this.key = key;
            this.value = value;
        }

        public string Key
        {
            get { return key; }
        }

        public string Value
        {
            get { return value; }
        }

        public string DispalyMember
        {
            get { return key + ": " + value; }
        }
    }

}
