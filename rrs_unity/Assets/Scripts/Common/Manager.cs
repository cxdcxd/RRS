using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RRS.Tools.Network;
using System;

public class Manager : MonoBehaviour
{
    public static bool inited = false;
 
    void Start()
    {
        print("Start 1");

        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = 50;

        

        AsyncIO.ForceDotNet.Force();
        print("Start 2");

        Statics.Init();
        inited = true;

        print("Start 3");

        if ( Statics.current_environment == Statics.Environments.Sim)
        print("RRS Ready " + Statics.current_config.consul_network_address);
        else
        {
            print("Real Network Ready");
        }
    }

    void OnApplicationQuit()
    {
        inited = false;
        Statics.Shutdown();

        if ( Statics.current_environment == Statics.Environments.Real)
        {
            if (Statics.network_manager_left_arm != null)
            {
                Statics.network_manager_left_arm.killAll();
                Statics.network_manager_left_arm = null;
            }

            if (Statics.network_manager_right_arm != null)
            {
                Statics.network_manager_right_arm.killAll();
                Statics.network_manager_right_arm = null;
            }

            if (Statics.network_manager_movo_status != null)
            {
                Statics.network_manager_movo_status.killAll();
                Statics.network_manager_right_arm = null;
            }
        }

        GC.Collect();
        GC.WaitForPendingFinalizers();

        NetMQ.NetMQConfig.Cleanup(false);

        print("RRS Terminated");
    }
}
