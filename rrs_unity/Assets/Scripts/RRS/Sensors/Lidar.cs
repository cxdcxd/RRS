// RRS Lidar
// Author : Edwin Babaians
// LMT - TUM

using UnityEngine;
using System.Collections;
using System.IO;
using ProtoBuf;
using System;
using System.Threading;
using RRS.Tools.Protobuf;

public class Lidar : MonoBehaviour
{
    public float fps = 15f;

    public int segments = 360;
    public int min_degree = -179;
    public int max_degree = 180;
    public int min_distance = 15;
    public int max_distance = 600;
    public float delta_degree = 0;
    public bool draw_debug = false;

    public delegate void DelegateDataChanged(byte[] buffer);
    public event DelegateDataChanged delegateLidarDataChanged;

    float robot_rotation;
    float[] values;
    float[] intensity;
    int counter_calc = 0;
    bool valid_data = false;
    bool inited = false;

    private float timer = 0;

    void Start()
    {
        values = new float[(int)segments];
        intensity = new float[(int)segments];
        inited = true;
    }

    private void calcScan()
    {
        Vector3 target_pos = new Vector3();
        Vector3 start_pos = new Vector3();

        if (segments != 0)
        {
            robot_rotation = Helper.convertRotation(transform.rotation.eulerAngles.y);
            delta_degree = ((float)(max_degree + (-1 * min_degree) + 1) / segments);

            int count = 0;
            RaycastHit hit;

            for (float i = max_degree; i > min_degree; i -= delta_degree, count++)
            {
                target_pos.y = transform.position.y;
                target_pos.x = Mathf.Cos((i + robot_rotation) * Mathf.Deg2Rad) * (max_distance / 100) + transform.position.x;
                target_pos.z = Mathf.Sin((i + robot_rotation) * Mathf.Deg2Rad) * (max_distance / 100) + transform.position.z;

                start_pos.y = transform.position.y;
                start_pos.x = Mathf.Cos((i + robot_rotation) * Mathf.Deg2Rad) * (min_distance / 100) + transform.position.x;
                start_pos.z = Mathf.Sin((i + robot_rotation) * Mathf.Deg2Rad) * (min_distance / 100) + transform.position.z;

                if (Physics.Raycast(start_pos, new Vector3(target_pos.x - start_pos.x, target_pos.y - start_pos.y, target_pos.z - start_pos.z), out hit, max_distance / 100))
                {
                    float distance = calcDistance(start_pos.x, start_pos.z, hit.point.x, hit.point.z);
                    values[count] = distance;

                    if (draw_debug)
                        Debug.DrawLine(start_pos, hit.point, Color.blue);
                }
                else
                {
                    if (draw_debug)
                        Debug.DrawLine(start_pos, target_pos, Color.red);

                    values[count] = 0;
                }
            }
        }
    }

    private byte[] getBuffer()
    {
        RRSLaser r_laser = new RRSLaser();
     
        float[] temp = (float[])values.Clone();

        int valid_count = 0;

        for (int i = 0; i < temp.Length; i++)
        {
            if (temp[i] != 0)
            {
                valid_count++;
            }
        }

       
        for (int i = 0; i < temp.Length; i++)
        {
            if (float.IsNaN(temp[i]) || float.IsInfinity(temp[i]) || float.IsNegativeInfinity(temp[i]) || float.IsPositiveInfinity(values[i]))
            {
                print("Bad value in laser data");
                temp[i] = 0;
            }
        }

        r_laser.angel_max = Mathf.Deg2Rad * max_degree;
        r_laser.angel_min = Mathf.Deg2Rad * min_degree;
        r_laser.range_max = (float)max_distance / 100;
        r_laser.range_min = (float)min_distance / 100;
        r_laser.ranges = temp;
        r_laser.angel_increment = delta_degree * Mathf.Deg2Rad;
        r_laser.scan_time = 8.7e-05f;
        r_laser.time_increment = r_laser.scan_time / 360;

       
        MemoryStream ms = new MemoryStream();
        ms = new MemoryStream();
        Serializer.Serialize<RRSLaser>(ms, r_laser);

        byte[] data = ms.ToArray();
        return data;
    }

    private float calcDistance(float x1, float y1, float x2, float y2)
    {
        float result = Mathf.Sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
        return result;
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer > (1 / fps) && inited)
        {
            calcScan();
            byte[] buffer = getBuffer();
            delegateLidarDataChanged?.Invoke(buffer);
            timer = 0;
       }
    }

}
