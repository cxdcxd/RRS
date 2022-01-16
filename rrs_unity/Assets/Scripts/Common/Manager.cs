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
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = 50;

        AsyncIO.ForceDotNet.Force();

        Statics.Init();
        inited = true;

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
