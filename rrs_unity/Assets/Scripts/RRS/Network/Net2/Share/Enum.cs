using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace RRS.Tools.Network
{
    #region NET2

    public enum Net2DataRateUnit
    {
        B,
        KB,
        MB,
        GB,
        TB,
        PB
    }

    public enum Net2State
    {
        STOPPED,
        STOPPING,
        STARTED,
        STARTING,
        CONNECTED,
        DISCONNECTED,
        BUSY
    }

    public enum Net2Direction
    {
        INPUT,
        OUTPUT
    }

    public enum Net2ConsulMode
    {
        CLIENT,
        MANAGER
    };
       
    #endregion

    #region NET1
   
    #endregion
}
