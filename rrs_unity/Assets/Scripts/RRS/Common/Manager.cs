using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Roboland.Tools.Network;

public class Manager : MonoBehaviour
{
    public static bool inited = false;

    void Start()
    {
        AsyncIO.ForceDotNet.Force();
        Statics.Init();
        inited = true;

        print("RRS Ready " + Statics.current_config.consul_network_address);
    }

    void OnApplicationQuit()
    {
        inited = false;
        Statics.Shutdown();
        NetMQ.NetMQConfig.Cleanup(false);

        print("RRS Terminated");
    }
}
