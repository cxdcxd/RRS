using ProtoBuf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RRS.Tools.Protobuf
{
    [ProtoContract]
    public class GameScoreBoard
    {
        [ProtoMember(1)]
        public string game_type;
        [ProtoMember(2)]
        public string game_mode;
        [ProtoMember(3)]
        public int state = 0;
        [ProtoMember(4)]
        public GameScorePlayer[] list_players;
        [ProtoMember(5)]
        public string left_time;

        public GameScoreBoard()
        {

        }
    }

    [ProtoContract]
    public class GameScorePlayer
    {
        [ProtoMember(1)]
        public string nick_name;
        [ProtoMember(2)]
        public int score = 0;
        [ProtoMember(3)]
        public byte[] image;
        [ProtoMember(4)]
        public int team = 1;
        [ProtoMember(5)]
        public GameScoreHigh high_score;
        [ProtoMember(6)]
        public int health = 0;
        [ProtoMember(7)]
        public int shield = 0;
        [ProtoMember(8)]
        public bool can_move = false;
        [ProtoMember(9)]
        public bool can_shoot = false;
        [ProtoMember(10)]
        public bool can_blast = false;
        [ProtoMember(11)]
        public bool can_high_jump = false;
        [ProtoMember(12)]
        public bool can_long_jump = false;
        [ProtoMember(13)]
        public int bullets = 0;
        [ProtoMember(14)]
        public int bombs = 0;
        [ProtoMember(15)]
        public int high_jumps = 0;
        [ProtoMember(16)]
        public int long_jumps = 0;
        [ProtoMember(17)]
        public int boost = 0;
        [ProtoMember(18)]
        public int symbol = 0;
        [ProtoMember(19)]
        public string gender;
        [ProtoMember(20)]
        public string total_rank;
        [ProtoMember(21)]
        public string total_score;
        [ProtoMember(22)]
        public string rfid;
        [ProtoMember(23)]
        public string full_name;
        [ProtoMember(24)]
        public string first_name;
        [ProtoMember(25)]
        public string last_name;

        public int CompareTo(GameScorePlayer obj)
        {
            if (obj == null) return 0;

            if (score < obj.score)
            {
                return 1;
            }
            if (score > obj.score)
            {
                return -1;
            }

            return 0;
        }

        public GameScorePlayer()
        {
                
        }
    }

    
    [ProtoContract]
    public class GameScoreHigh
    {
        [ProtoMember(1)]
        public GameScoreRank total;
        [ProtoMember(2)]
        public GameScoreRank type;
        [ProtoMember(3)]
        public GameScoreRank mode;

        public GameScoreHigh()
        {

        }
    }

    
    [ProtoContract]
    public class GameScoreRank
    {
        [ProtoMember(1)]
        public string rank;
        [ProtoMember(2)]
        public string score;

        public GameScoreRank()
        {

        }
    }
}
