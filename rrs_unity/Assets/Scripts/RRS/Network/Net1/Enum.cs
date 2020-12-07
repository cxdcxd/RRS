using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RRS.Tools.Network
{
    public enum NetworkResponse
    {
        CONNECTED,
        TIMEOUT,
        REJECTED,
        NONE,
        BUSY,
        ACK,
        ERROR
    }

    public enum NetworkState
    {
        IDLE,
        CONNECTING,
        SETUP,
        CONNECTED,
        DISCONNECTED
    }

    public enum NetworkType
    {
        PUB,
        SUB,
        PUBSUB,
        PUBSUBREQRES,
        PAIRREQRES
    }

    public enum NetworkKillType
    {
        CONNECTION,
        SETUP,
        ALL
    }
}
