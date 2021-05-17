using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RRS.Tools;
using RRS.Tools.Protobuf;
using ProtoBuf;
using System.IO;
using Newtonsoft.Json;

public class IMU : MonoBehaviour
{
    public float fps = 20f;
 
    public Vector3 angVel;
    public Vector3 angAccel;
    public Vector3 linVel;
    public Vector3 linAccel;
    public Quaternion rotation;

    private Vector3 lastPos;
    private Vector3 lastAng;
    private Vector3 lastLinVel;
    private Vector3 lastAngVel;

    private float timer = 0.0f;

    public delegate void DelegateDataChanged(byte[] buffer);
    public event DelegateDataChanged delegateIMUDataChanged;

    void Start()
    {
        
    }

    void FixedUpdate()
    {
        timer += Time.deltaTime;
        if (timer > (1 / fps))
        {
            lastLinVel = linVel;
            lastAngVel = angVel;

            var lastPosInv = transform.InverseTransformPoint(lastPos);

            linVel.x = (0 - lastPosInv.x) / timer;
            linVel.y = (0 - lastPosInv.y) / timer;
            linVel.z = (0 - lastPosInv.z) / timer;

            var deltaX = Mathf.Abs((transform.rotation.eulerAngles).x) - lastAng.x;
            if (Mathf.Abs(deltaX) < 180 && deltaX > -180) angVel.x = deltaX / timer;
            else
            {
                if (deltaX > 180) angVel.x = (360 - deltaX) / timer;
                else angVel.x = (360 + deltaX) / timer;
            }

            var deltaY = Mathf.Abs((transform.rotation.eulerAngles).y) - lastAng.y;
            if (Mathf.Abs(deltaY) < 180 && deltaY > -180) angVel.y = deltaY / timer;
            else
            {
                if (deltaY > 180) angVel.y = (360 - deltaY) / timer;
                else angVel.y = (360 - deltaY) / timer;
            }

            var deltaZ = Mathf.Abs((transform.rotation.eulerAngles).z) - lastAng.z;
            if (Mathf.Abs(deltaZ) < 180 && deltaZ > -180) angVel.z = deltaZ / timer;
            else
            {
                if (deltaZ > 180) angVel.z = (360 - deltaZ) / timer;
                else angVel.z = (360 + deltaZ) / timer;
            }

            linAccel.x = (linVel.x - lastLinVel.x) / timer;
            linAccel.y = (linVel.y - lastLinVel.y) / timer;
            linAccel.z = (linVel.z - lastLinVel.z) / timer;
            angAccel.x = ((angVel.x - lastAngVel.x) / timer) / 9.81f;
            angAccel.y = ((angVel.y - lastAngVel.y) / timer) / 9.81f;
            angAccel.z = ((angVel.z - lastAngVel.z) / timer) / 9.81f;

            lastPos = transform.position;

            lastAng.x = Mathf.Abs((transform.rotation.eulerAngles).x);
            lastAng.y = Mathf.Abs((transform.rotation.eulerAngles).y);
            lastAng.z = Mathf.Abs((transform.rotation.eulerAngles).z);

            rotation = transform.rotation;

            RRSIMU imu_msg = new RRS.Tools.Protobuf.RRSIMU();
            imu_msg.orientation = new RRS.Tools.Protobuf.SVector4();
            var convertr = Helper.Unity2Ros(rotation);
            imu_msg.orientation.x = convertr.x;
            imu_msg.orientation.y = convertr.y;
            imu_msg.orientation.z = convertr.z;
            imu_msg.orientation.w = convertr.w;

            imu_msg.orientation_covariance = new float[9];
            imu_msg.orientation_covariance[0] = 2.6f / 1000000000;
            imu_msg.orientation_covariance[4] = 2.6f / 1000000000;

            imu_msg.linear_acceleration = new RRS.Tools.Protobuf.SVector3();
            var converta = Helper.Unity2Ros(linAccel);
            imu_msg.linear_acceleration.x = converta.x;
            imu_msg.linear_acceleration.y = converta.y;
            imu_msg.linear_acceleration.z = converta.z;

            imu_msg.linear_acceleration_covariance = new float[9];
            imu_msg.linear_acceleration_covariance[0] = 2.6f / 10000000;
            imu_msg.linear_acceleration_covariance[4] = 2.6f / 10000000;
            imu_msg.linear_acceleration_covariance[8] = 2.6f / 10000000;

            imu_msg.angular_velocity = new RRS.Tools.Protobuf.SVector3();
            var convertv = Helper.Unity2Ros(angVel);
            imu_msg.angular_velocity.x = convertv.x;
            imu_msg.angular_velocity.y = convertv.y;
            imu_msg.angular_velocity.z = convertv.z;

            imu_msg.angular_velocity_covariance = new float[9];
            imu_msg.angular_velocity_covariance[0] = 2.5f / 1000000000;
            imu_msg.angular_velocity_covariance[4] = 2.5f / 1000000000;
            imu_msg.angular_velocity_covariance[8] = 2.5f / 1000000000;

            MemoryStream ms = new MemoryStream();
            ms = new MemoryStream();
            Serializer.Serialize<RRSIMU>(ms, imu_msg);

            //string json = JsonConvert.SerializeObject(imu_msg);
            //byte[] size = System.Text.Encoding.ASCII.GetBytes(json);
            //UnityEngine.Debug.Log("IMU2 size " + size.Length.ToString());

            byte[] data = ms.ToArray();

            delegateIMUDataChanged?.Invoke(data);

            timer = 0;
        }
    }
}
