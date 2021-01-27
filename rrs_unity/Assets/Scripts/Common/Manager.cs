using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RRS.Tools.Network;

public class Manager : MonoBehaviour
{
    public static bool inited = false;
    public int desire_frame_rate = 30;
    void Start()
    {
        AsyncIO.ForceDotNet.Force();
        Statics.Init();
        inited = true;

        Application.targetFrameRate = desire_frame_rate;

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
