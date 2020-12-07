using ProtoBuf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RRS.Tools.Protobuf
{
    [ProtoContract]
    public class RRSCollision
    {
        [ProtoMember(1)]
        public int collision_time;

        [ProtoMember(2)]
        public SVector3[] points;

        [ProtoMember(3)]
        public int robot_id;

        [ProtoMember(4)]
        public int collision_type;

        public RRSCollision()
        {

        }
    }

    
    [ProtoContract]
    public class RRSTable
    {
        [ProtoMember(1)]
        public int id = 0;
        [ProtoMember(2)]
        public int state = 0;
        [ProtoMember(3)]
        public int wait_time = 0;

        public RRSTable()
        {

        }
    }

    
    [ProtoContract]
    public class RRSScene
    {
        [ProtoMember(1)]
        public int version;
        [ProtoMember(2)]
        public RobotProtocol[] service_robots;
        [ProtoMember(3)]
        public RRSTable[] tables;
        [ProtoMember(4)]
        public RRSOrder[] orders;
        [ProtoMember(5)]
        public int performance_angery_users;
        [ProtoMember(6)]
        public int performance_total_delivered;
        [ProtoMember(7)]
        public int performance_missed_users;
        [ProtoMember(8)]
        public int performance_max_delivery_time;
        [ProtoMember(9)]
        public int performance_min_delivery_time;
        [ProtoMember(10)]
        public int performance_avg_delivery_time;
        [ProtoMember(11)]
        public int performance_collision_count;
        [ProtoMember(12)]
        public int simulation_time;
        [ProtoMember(13)]
        public string remote_ip;

        public RRSScene()
        {

        }
    }

    
    [ProtoContract]
    public class RRSOrder
    {
        [ProtoMember(1)]
        public int id = 0;
        [ProtoMember(2)]
        public int state = 0;
        [ProtoMember(3)]
        public int wait_time = 0;
        [ProtoMember(4)]
        public int table_id = 0;

        public RRSOrder()
        {

        }
    }
  
}
