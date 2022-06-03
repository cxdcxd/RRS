using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MaviGroundStation;
using System.IO;
using ProtoBuf;
using RRS.Tools.Network;
using System;
using UnityEngine.SceneManagement;
using System.Linq;

public class CPDManager : MonoBehaviour
{
    public string skill_file_name = "1.txt";
    public string scenario_file_name = "1.txt";

    public GameObject user;
    public GameObject robot;
    public GameObject skillpoint_prefab;
    public GameObject cpdpoint_prefab;
    public GameObject robot_prefab;
    public GameObject user_prefab;



    private float originalWidth = 1000;
    private float originalHeight = 900;
    bool is_teleoperation_mode = false;
    public static bool is_network_inited = false;
    public int current_step = 0;
    public float delta_d = 0.6f;
    public Mode operation_mpde = Mode.SharedAutonomy;
    public Movo movo_ref;
    public Franka franka_ref;
    public string mode = "";


    List<RVector7> robot_point_list = new List<RVector7>();
    List<RVector7> skill_point_list = new List<RVector7>();
    List<RVector7> scenario_point_list = new List<RVector7>();
    List<RVector7> cpd_point_list = new List<RVector7>();
    List<RVector7> user_point_list = new List<RVector7>();
    RVector7 tele_robot_location = new RVector7();
    List<GameObject> skill_point_object_list = new List<GameObject>();
    List<GameObject> robot_point_object_list = new List<GameObject>();
    List<GameObject> cpd_point_object_list = new List<GameObject>();
    List<GameObject> user_point_object_list = new List<GameObject>();
    bool start_recording_skill = false;
    bool start_recording_scenario = false;
    bool update_render_cpd_result = false;
    bool is_switch_to_skill = false;
    Vector3 initial_user_point;
    float next_scnario_time = 0;
    List<float> travel_times = new List<float>();
    float teleoperation_two_points_travel_time = 0;
    float average_teleoperation_two_points_travel_time = 0;
    RVector7 old_user_location = new RVector7();
    RVector7 old_robot_location = new RVector7();
    bool is_begin = false;
    float old_valid_time = 0;
    int reset_trail = 0;
    int scenario_index = 0;
    int scenario_step_index = 0;
    int old_scenario_step_index = 0;
    bool is_play_scenario = false;
    public bool is_rigid = true;
    bool cpd_valid_result = true;
    int bench_index = 1;
    int disconnetion_index = 0;
    int bench_state = 0;
    bool is_go_to_skill = false;
    float p_max_distance = 0;
    float p_tracking_error = 0;
    float p_user_traj_len = 0;
    float p_robot_traj_len = 0;
    DateTime time_cpd_request_start = DateTime.Now;
    DateTime time_cpd_request_done = DateTime.Now;
    float total_max_distance = 0;
    float total_tracking_error = 0;
    float total_user_traj_len = 0;
    float total_robot_traj_len = 0;

    void resetScenario()
    {
        removePoints();

        scenario_step_index = 0;
        tele_robot_location = new RVector7();
        robot_point_list = new List<RVector7>();
        skill_point_list = new List<RVector7>();
        scenario_point_list = new List<RVector7>();
        cpd_point_list = new List<RVector7>();
        user_point_list = new List<RVector7>();

        skill_point_object_list = new List<GameObject>();
        robot_point_object_list = new List<GameObject>();
        cpd_point_object_list = new List<GameObject>();
        user_point_object_list = new List<GameObject>();

        is_begin = false;
        old_valid_time = 0;
        reset_trail = 0;
        scenario_index = 0;
        is_play_scenario = false;
        cpd_valid_result = true;
        disconnetion_index = 0;

        foreach (var item in skill_point_object_list)
        {
            Destroy(item);
        }

        foreach (var item in cpd_point_object_list)
        {
            Destroy(item);
        }

        old_user_location = new RVector7();
        old_robot_location = new RVector7();

        skill_point_object_list = new List<GameObject>();
        cpd_point_object_list = new List<GameObject>();
        is_switch_to_skill = false;
        is_teleoperation_mode = false;
        is_go_to_skill = false;

        user.transform.position = initial_user_point;
        //user.transform.rotation = initial_user_point.rotation;

        cpd_point_list = new List<RVector7>();
        skill_point_list = new List<RVector7>();
        robot_point_list = new List<RVector7>();
        user_point_list = new List<RVector7>();

        is_begin = false;
        old_valid_time = 0;
        current_step = 0;
        average_teleoperation_two_points_travel_time = 0;
        teleoperation_two_points_travel_time = 0;
        travel_times = new List<float>();

        old_user_location = new RVector7();
        old_robot_location = new RVector7();

        scenario_point_list = new List<RVector7>();

        is_play_scenario = false;

        p_max_distance = 0;
        p_tracking_error = 0;
        p_user_traj_len = 0;
        p_robot_traj_len = 0;

        next_scnario_time = 0;

        scenario_index = 0;

        reset_trail = 1;

        is_begin = false;
        is_teleoperation_mode = false;

        removePoints();
        resetPoint();
    }

    public enum Mode
    {
        Nothing,
        SharedAutonomy,
        SharedAutonomyCPD,
    }

    void resetPoint()
    {
        tele_robot_location.x = user.transform.position.x;
        tele_robot_location.y = user.transform.position.y;
        tele_robot_location.z = user.transform.position.z;

        tele_robot_location.qx = user.transform.rotation.x;
        tele_robot_location.qy = user.transform.rotation.y;
        tele_robot_location.qz = user.transform.rotation.z;
        tele_robot_location.qw = user.transform.rotation.w;
    }

    void Start()
    {
        Statics.cpd_manager_ref = this;
        initial_user_point = user.transform.position;
        resetPoint();
    }

    public void Main_tele_network_eventDataUpdated()
    {
        tele_robot_location = Statics.main_tele_network.get;
    }
    public void Main_cpd_network_eventDataUpdated()
    {
        print("Get the CPD result back!");

        cpd_point_list = new List<RVector7>();

        RRSCPDResult cmd = Statics.main_cpd_network.get;

        if (cmd.result_iterations != 0)
        {
            print("Len: " + cmd.result_points.Length);
            print("Sigma2 " + cmd.result_2);
            print("Scale " + cmd.result_1);
            print("Iterations" + cmd.result_iterations);


            for (int i = 0; i < cmd.result_points.Length; i++)
            {
                cpd_point_list.Add(cmd.result_points[i]);

                //Reput the qs to registered points
                cpd_point_list[i].qx = skill_point_list[i].qx;
                cpd_point_list[i].qy = skill_point_list[i].qy;
                cpd_point_list[i].qz = skill_point_list[i].qz;
                cpd_point_list[i].qw = skill_point_list[i].qw;
            }

            update_render_cpd_result = true;
        }

        cpd_valid_result = true;
        time_cpd_request_done = DateTime.Now;
        print("Time is ");
        print("CPD Registeration Time : " + (time_cpd_request_done - time_cpd_request_start).TotalMilliseconds.ToString() + " ms");

    }

    float dist(RVector7 a, RVector7 b)
    {
        float d = Mathf.Sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
        return d;
    }

    float findClosestDistinSkill(GameObject point, List<GameObject> list)
    {
        int min_index = 0;
        float min_distance = 100000000;

        int start = 0;
        int finish = list.Count;

        for (int index = start; index < finish; index++)
        {
            var item = list[index];
            float distance = Mathf.Sqrt((item.transform.position.x - point.transform.position.x) * (item.transform.position.x - point.transform.position.x) + (item.transform.position.y - point.transform.position.y) * (item.transform.position.y - point.transform.position.y) + (item.transform.position.z - point.transform.position.z) * (item.transform.position.z - point.transform.position.z));

            if (distance < min_distance)
            {
                min_distance = distance;
                min_index = index;
            }
        }

        return min_distance;
    }

    int findClosestStepinSkill(RVector7 point, List<RVector7> list, int current_predicted_step = -1)
    {
        int min_index = 0;
        float min_distance = 100000000;

        int start = 0;
        int finish = list.Count;

        if (current_predicted_step != -1)
        {
            start = current_predicted_step - 20;
            finish = current_predicted_step + 20;
        }

        if (start < 0) start = 0;
        if (finish > list.Count) finish = list.Count;

        for (int index = start; index < finish; index++)
        {
            var item = list[index];
            float distance = Mathf.Sqrt((item.x - point.x) * (item.x - point.x) + (item.y - point.y) * (item.y - point.y) + (item.z - point.z) * (item.z - point.z));

            if (distance < min_distance)
            {
                min_distance = distance;
                min_index = index;
            }
        }

        return min_index;
    }

    float findClosestDistanceinSkill(RVector7 point, List<RVector7> list)
    {
        int min_index = 0;
        float min_distance = 100000000;

        int start = 0;
        int finish = list.Count;

        if (start < 0) start = 0;
        if (finish > list.Count) finish = list.Count;

        for (int index = start; index < finish; index++)
        {
            var item = list[index];
            float distance = Mathf.Sqrt((item.x - point.x) * (item.x - point.x) + (item.y - point.y) * (item.y - point.y) + (item.z - point.z) * (item.z - point.z));

            if (distance < min_distance)
            {
                min_distance = distance;
                min_index = index;
            }
        }

        return min_index;
    }

    void KeyQ()
    {
        print("Load the scenario");

        scenario_point_list = new List<RVector7>();

        FileStream fs = new FileStream(Application.dataPath + "/Benchmarks/scenarios/" + scenario_file_name, FileMode.Open, FileAccess.Read);
        StreamReader sr = new StreamReader(fs);

        string a = sr.ReadToEnd();
        char[] ccc = new char[1];
        ccc[0] = '\n';

        char[] ccc2 = new char[1];
        ccc2[0] = ',';

        string[] list = a.Split(ccc);

        try
        {
            foreach (var item in list)
            {
                if (item == "") continue;

                string[] items = item.Replace("\r", string.Empty).Split(ccc2);

                RVector7 point = new RVector7();
                point.x = float.Parse(items[0]);
                point.y = float.Parse(items[1]);
                point.z = float.Parse(items[2]);

                point.qx = float.Parse(items[3]);
                point.qy = float.Parse(items[4]);
                point.qz = float.Parse(items[5]);
                point.qw = float.Parse(items[6]);

                scenario_point_list.Add(point);
            }
        }
        catch (Exception ee)
        {
            string me = ee.Message;
        }

        print("load scenario with " + scenario_point_list.Count + " points");

        sr.Close();
        fs.Close();
    }

    void KeyB()
    {
        reset_trail = 1;
        print("Begin Experiments");
        robot_point_list = new List<RVector7>();
        is_begin = true;
        is_teleoperation_mode = true;
    }

    void KeyL()
    {
        print("Load the recorded skill");

        skill_point_list = new List<RVector7>();

        FileStream fs = new FileStream(Application.dataPath + "/Benchmarks/skills/" + skill_file_name, FileMode.Open, FileAccess.Read);
        StreamReader sr = new StreamReader(fs);

        string a = sr.ReadToEnd();
        char[] ccc = new char[1];
        ccc[0] = '\n';

        char[] ccc2 = new char[1];
        ccc2[0] = ',';

        string[] list = a.Split(ccc);

        try
        {
            foreach (var item in list)
            {
                if (item == "") continue;

                string[] items = item.Replace("\r", string.Empty).Split(ccc2);

                RVector7 point = new RVector7();
                point.x = float.Parse(items[0]);
                point.y = float.Parse(items[1]);
                point.z = float.Parse(items[2]);

                point.qx = float.Parse(items[3]);
                point.qy = float.Parse(items[4]);
                point.qz = float.Parse(items[5]);
                point.qw = float.Parse(items[6]);

                skill_point_list.Add(point);
            }
        }
        catch (Exception ee)
        {
            string me = ee.Message;
        }

        print("load skill with " + skill_point_list.Count + " points");

        sr.Close();
        fs.Close();

        foreach (var item in skill_point_object_list)
        {
            Destroy(item);
        }

        skill_point_object_list = new List<GameObject>();

        foreach (var item in skill_point_list)
        {
            GameObject g = Instantiate(skillpoint_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
            skill_point_object_list.Add(g);
        }
    }

    void KeyT()
    {
        is_teleoperation_mode = !is_teleoperation_mode;

        if (is_teleoperation_mode)
        {
            print("Network Connected");

            is_teleoperation_mode = true;
            is_switch_to_skill = false;
        }
        else
        {
            print("Network Disconneted");

            if (operation_mpde == Mode.SharedAutonomyCPD)
            {
                cpd_valid_result = false;
                doSkillCPD();
            }

            is_teleoperation_mode = false;
            is_switch_to_skill = true;
            is_go_to_skill = true;
        }

        resetPoint();
    }

    void KeyP()
    {
        reset_trail = 1;

        robot_point_object_list = new List<GameObject>();

        foreach (var item in robot_point_list)
        {
            GameObject g = Instantiate(robot_prefab, new Vector3(item.x, item.y, item.z), Quaternion.identity);
            robot_point_object_list.Add(g);
        }

        user_point_object_list = new List<GameObject>();

        foreach (var item in user_point_list)
        {
            GameObject g = Instantiate(user_prefab, new Vector3(item.gx, item.gy, item.gz), Quaternion.identity);
            user_point_object_list.Add(g);
        }

        foreach (var item in skill_point_object_list)
        {
            Destroy(item);
        }

        skill_point_object_list = new List<GameObject>();

        foreach (var item in cpd_point_object_list)
        {
            Destroy(item);
        }

        cpd_point_object_list = new List<GameObject>();

        //
        p_max_distance = calculate_max_delta_d();
        p_tracking_error = calculate_user_tracking_error();
        p_user_traj_len = calculate_user_traj_length();
        p_robot_traj_len = calculate_robot_traj_length();

        print("max_distance = " + p_max_distance.ToString());
        print("total_tracking_error = " + p_tracking_error.ToString());
        print("user_traj_len = " + p_user_traj_len.ToString());
        print("robot_traj_len = " + p_robot_traj_len.ToString());

        print("User points " + user_point_list.Count.ToString());
        print("Robot points " + robot_point_list.Count.ToString());
    }

    void FixedUpdate()
    {
        if (Time.time >= next_scnario_time)
        {
            next_scnario_time = Time.time + (1 / 10); //10 hz record

            if (is_play_scenario)
            {
                if (scenario_index < scenario_point_list.Count && cpd_valid_result)
                {
                    RVector7 next_user_location = new RVector7();
                    next_user_location.x = scenario_point_list[scenario_index].x;
                    next_user_location.y = scenario_point_list[scenario_index].y;
                    next_user_location.z = scenario_point_list[scenario_index].z;

                    next_user_location.qx = scenario_point_list[scenario_index].qx;
                    next_user_location.qy = scenario_point_list[scenario_index].qy;
                    next_user_location.qz = scenario_point_list[scenario_index].qz;
                    next_user_location.qw = scenario_point_list[scenario_index].qw;

                    user.transform.localPosition = new Vector3(next_user_location.x, next_user_location.y, next_user_location.z);
                    user.transform.localRotation = new Quaternion(next_user_location.qx, next_user_location.qy, next_user_location.qz, next_user_location.qw);

                    //print("Next user " + next_user_location.x);
                    //print("Next local user " + user.transform.position.x);

                    scenario_index++;

                    float d = dist(next_user_location, old_user_location);

                    if (scenario_index % 7 == 0)
                    {
                        scenario_step_index++;
                        next_user_location.gx = user.transform.position.x;
                        next_user_location.gy = user.transform.position.y;
                        next_user_location.gz = user.transform.position.z;
                        user_point_list.Add(next_user_location);
                        old_user_location = next_user_location;
                    }
                }
            }

            if (start_recording_scenario)
            {
                RVector7 r = new RVector7();

                r.x = user.transform.localPosition.x;
                r.y = user.transform.localPosition.y;
                r.z = user.transform.localPosition.z;

                r.qx = user.transform.localRotation.x;
                r.qy = user.transform.localRotation.y;
                r.qz = user.transform.localRotation.z;
                r.qw = user.transform.localRotation.w;

                scenario_point_list.Add(r);
            }



            if (is_begin)
            {
                float d = dist(tele_robot_location, old_robot_location);



                if (d >= delta_d)
                {

                    if (start_recording_skill)
                    {

                        skill_point_list.Add(tele_robot_location);


                    }

                    if (current_step == 0)
                        old_valid_time = Time.time;
                    else
                    {
                        teleoperation_two_points_travel_time = Time.time - old_valid_time;
                        old_valid_time = Time.time;
                    }

                    if (is_switch_to_skill == false)
                    {

                        travel_times.Add(teleoperation_two_points_travel_time);
                        average_teleoperation_two_points_travel_time = travel_times.Average();

                        robot_point_list.Add(tele_robot_location);
                        old_robot_location = tele_robot_location;
                        current_step++;
                    }
                }
            }

            if (is_switch_to_skill == false)
            {
                //telerobot is valid so nothing to do
            }
            else
            {
                print(scenario_step_index);

                if (operation_mpde == Mode.SharedAutonomy)
                {
                    if (current_step > 0 && current_step < skill_point_list.Count && scenario_step_index != old_scenario_step_index)
                    {
                        if (is_go_to_skill)
                        {
                            //here because it is rigid we can select the nearest point by using the current step window +/- 20
                            current_step = findClosestStepinSkill(tele_robot_location, skill_point_list, current_step);
                        }

                        //print("Shared Autonomy find closest index " + current_step);

                        is_go_to_skill = false;
                        //next_automatic_skill_time = Time.time + average_teleoperation_two_points_travel_time / 1.3f;
                        old_scenario_step_index = scenario_step_index; //Time Fire
                        tele_robot_location = skill_point_list[current_step];
                        robot_point_list.Add(tele_robot_location);
                        old_robot_location = tele_robot_location;
                        current_step++;
                    }
                }
                else if (operation_mpde == Mode.SharedAutonomyCPD && cpd_valid_result)
                {
                    if (current_step > 0 && current_step < cpd_point_list.Count && scenario_step_index != old_scenario_step_index)
                    {
                        if (is_go_to_skill)
                        {
                            //here we can't predict the step so we just select the nearest point
                            current_step = findClosestStepinSkill(tele_robot_location, cpd_point_list);
                        }

                        //print("Shared Autonomy CPD find closest index " + current_step);

                        is_go_to_skill = false;

                        //next_automatic_skill_time = Time.time + average_teleoperation_two_points_travel_time;
                        old_scenario_step_index = scenario_step_index; //Time Fire
                        tele_robot_location = cpd_point_list[current_step];
                        robot_point_list.Add(tele_robot_location);
                        old_robot_location = tele_robot_location;

                        current_step++;

                        //print("Next step " + current_step);
                    }
                }
            }
        }

        if (is_teleoperation_mode)
        {
            if (Statics.main_tele_network != null)
            {
                RVector7 r = new RVector7();

                r.x = user.transform.position.x;
                r.y = user.transform.position.y;
                r.z = user.transform.position.z;

                r.qx = user.transform.rotation.x;
                r.qy = user.transform.rotation.y;
                r.qz = user.transform.rotation.z;
                r.qw = user.transform.rotation.w;

                Statics.main_tele_network.sendMessage(r);
            }

        }

        //float distance = cdist(tele_robot_location, robot.transform.position);
        //print(distance);

        if (operation_mpde == Mode.SharedAutonomyCPD)
        {
            Vector3 target = new Vector3(tele_robot_location.x, tele_robot_location.y, tele_robot_location.z);
            robot.transform.position = (0.9f * robot.transform.position + 0.1f * target);
        }
        else
        {
            Vector3 target = new Vector3(tele_robot_location.x, tele_robot_location.y, tele_robot_location.z);
            robot.transform.position = target;
        }


        //robot.transform.position = new Vector3(tele_robot_location.x, tele_robot_location.y, tele_robot_location.z);
        robot.transform.rotation = new Quaternion(tele_robot_location.qx, tele_robot_location.qy, tele_robot_location.qz, tele_robot_location.qw);
    }

    float cdist(RVector7 a, Vector3 b)
    {
        return Mathf.Sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
    }

    void saveCurrentBenchStep(string mode, string method, string name, bool total = false)
    {
        print("Save the current benchmark");

        FileStream fs = new FileStream(Application.dataPath + "/Benchmarks/" + mode + "_" + method + ".txt", FileMode.Append, FileAccess.Write);
        StreamWriter sw = new StreamWriter(fs);

        if (total == false)
            sw.WriteLine(name + " " + p_max_distance + " " + p_tracking_error + " " + p_robot_traj_len + " " + p_user_traj_len);
        else
        {
            sw.WriteLine(name + " " + p_max_distance + " " + p_tracking_error + " " + p_robot_traj_len + " " + p_user_traj_len);
            sw.WriteLine(name + " " + total_max_distance + " " + total_tracking_error + " " + total_robot_traj_len + " " + total_user_traj_len);
        }

        sw.Close();
        fs.Close();
    }

    public void Update()
    {
        if (reset_trail > 0)
        {
            user.GetComponent<TrailRenderer>().time = 0;
            robot.GetComponent<TrailRenderer>().time = 0;

            reset_trail++;

            if (reset_trail > 30)
            {
                user.GetComponent<TrailRenderer>().time = 100;
                robot.GetComponent<TrailRenderer>().time = 100;
                reset_trail = 0;
            }
        }

        if (Input.GetKeyDown(KeyCode.E))
        {
            print("Start / Stop scenario recording");

            if (start_recording_scenario == false)
            {
                scenario_point_list = new List<RVector7>();
            }
            start_recording_scenario = !start_recording_scenario;
        }

        if (Input.GetKeyDown(KeyCode.P))
        {
            KeyP();
        }

        if (Input.GetKeyDown(KeyCode.R))
        {
            print("Start / Stop recording skill");

            if (start_recording_skill == false)
            {
                skill_point_list = new List<RVector7>();
            }
            start_recording_skill = !start_recording_skill;
        }

        if (Input.GetKeyDown(KeyCode.S))
        {
            print("Save the recorded skill");

            FileStream fs = new FileStream(Application.dataPath + "/Benchmarks/skills/" + skill_file_name, FileMode.Create, FileAccess.Write);
            StreamWriter sw = new StreamWriter(fs);

            int i = 0;
            foreach (var item in skill_point_list)
            {
                i++;
                sw.WriteLine(item.x + "," + item.y + "," + item.z + "," + item.qx + "," + item.qy + "," + item.qz + "," + item.qw);
            }

            print("save skill with " + i.ToString());

            sw.Close();
            fs.Close();
        }

        if (Input.GetKeyDown(KeyCode.W))
        {
            print("Save the scenario");

            FileStream fs = new FileStream(Application.dataPath + "/Benchmarks/scenarios/" + scenario_file_name, FileMode.Create, FileAccess.Write);
            StreamWriter sw = new StreamWriter(fs);

            int i = 0;
            foreach (var item in scenario_point_list)
            {
                i++;
                sw.WriteLine(item.x + "," + item.y + "," + item.z + "," + item.qx + "," + item.qy + "," + item.qz + "," + item.qw);
            }

            print("Save the scenario with " + i.ToString());

            sw.Close();
            fs.Close();
        }

        if (Input.GetKeyDown(KeyCode.L))
        {
            KeyL();
        }

        if (Input.GetKeyDown(KeyCode.Q))
        {
            KeyQ();
        }

        if (Input.GetKeyDown(KeyCode.T))
        {
            KeyT();
        }

        if (Input.GetKeyDown(KeyCode.B))
        {
            KeyB();
        }

        if (update_render_cpd_result)
        {
            update_render_cpd_result = false;

            foreach (var item in cpd_point_object_list)
            {
                Destroy(item);
            }

            cpd_point_object_list = new List<GameObject>();

            foreach (var item in cpd_point_list)
            {
                GameObject g = Instantiate(cpdpoint_prefab, Vector3.zero, Quaternion.identity);

                g.transform.localPosition = new Vector3(item.x, item.y, item.z);

                cpd_point_object_list.Add(g);
            }
        }

        if (bench_state == 1)
        {
            print("Bench mark " + bench_index.ToString());

            skill_file_name = bench_index.ToString() + ".txt";
            scenario_file_name = bench_index.ToString() + ".txt";

            //Reset Everything
            resetScenario();

            //Load Skill
            KeyL();

            //Load Scenario
            KeyQ();

            //Track
            KeyB();

            //Play Scenario
            is_play_scenario = !is_play_scenario;

            bench_state = 2;
        }

        if (bench_state == 2)
        {
            if (scenario_index >= (scenario_point_list.Count / 2) && disconnetion_index == 0) // disconnet on 0.5
            {
                disconnetion_index = 1;
                print("Disconnet 0.5");
                //Disconnet Network
                KeyT();
            }

            if (scenario_index >= (3 * scenario_point_list.Count / 4) && disconnetion_index == 1) //bring back on 0.75
            {
                disconnetion_index = 2;
                print("Connect 0.75");
                //Connect Network
                KeyT();
            }

            if (scenario_index >= scenario_point_list.Count)
            {
                //Done
                print("Test done for " + bench_index.ToString());

                KeyP();

                total_max_distance += p_max_distance;
                total_tracking_error += p_tracking_error;
                total_user_traj_len += p_user_traj_len;
                total_robot_traj_len += p_robot_traj_len;

                if (bench_index < 26)
                {
                    total_max_distance = total_max_distance / 26;
                    total_tracking_error = total_tracking_error / 26;
                    total_user_traj_len = total_user_traj_len / 26;
                    total_robot_traj_len = total_robot_traj_len / 26;

                    print("Avg max_distance = " + total_max_distance.ToString());
                    print("Avg total_tracking_error = " + total_tracking_error.ToString());
                    print("Avg user_traj_len = " + total_user_traj_len.ToString());
                    print("Avg robot_traj_len = " + total_robot_traj_len.ToString());

                    saveCurrentBenchStep(mode, operation_mpde.ToString(), bench_index.ToString(), false);

                    bench_index++;
                    bench_state = 1;

                }
                else
                {
                    print("benchmark finish");

                    saveCurrentBenchStep(mode, operation_mpde.ToString(), bench_index.ToString(), true);
                    bench_state = 3;
                }
            }
        }
    }

    float calculate_max_delta_d()
    {
        float max_d = -1;
        RVector7 old = null;
        bool skip_first = false;

        foreach (var item in robot_point_list)
        {
            if (skip_first == false)
            {
                skip_first = true;
                continue;
            }

            if (old == null)
            {
                old = item;
            }
            else
            {
                float d = dist(item, old);
                if (d > max_d)
                    max_d = d;
                old = item;
            }
        }

        return max_d;
    }

    float calculate_user_tracking_error()
    {
        float e = 0;
        int x = 0;
        foreach (var item in user_point_object_list)
        {
            float d = findClosestDistinSkill(item, robot_point_object_list);

            if (d < delta_d)
            {
                //ok
            }
            else
            {
                //real error
                x++;
                e += d;
            }

        }

        if (x != 0)
            e = e / x;
        else
            e = 0;

        return e;
    }

    float calculate_user_traj_length()
    {
        float len = 0;
        RVector7 old = null;

        foreach (var item in user_point_list)
        {
            if (old == null)
            {
                old = item;
            }
            else
            {
                float d = dist(item, old);
                len += d;
                old = item;
            }
        }
        return len;
    }

    float calculate_robot_traj_length()
    {
        float len = 0;
        RVector7 old = null;

        foreach (var item in robot_point_list)
        {
            if (old == null)
            {
                old = item;
            }
            else
            {
                float d = dist(item, old);
                len += d;
                old = item;
            }
        }
        return len;
    }

    void removePoints()
    {
        foreach (var item in skill_point_object_list)
        {
            Destroy(item);
        }

        foreach (var item in cpd_point_object_list)
        {
            Destroy(item);
        }

        foreach (var item in robot_point_object_list)
        {
            Destroy(item);
        }

        foreach (var item in user_point_object_list)
        {
            Destroy(item);
        }

        skill_point_object_list = new List<GameObject>();
        cpd_point_object_list = new List<GameObject>();
        robot_point_object_list = new List<GameObject>();
        user_point_object_list = new List<GameObject>();
    }

    void doSkillCPD()
    {
        RRSCPDCommand cpd_command = new RRSCPDCommand();
        //Current User Trajectory
        int robot_count = robot_point_list.Count;

        cpd_command.points_a = robot_point_list.ToArray();
        cpd_command.points_skill = skill_point_list.ToArray();
        cpd_command.mode = 0;

        if (is_rigid == false) cpd_command.mode = 1;

        List<RVector7> cut_skill_list = new List<RVector7>();

        int i = 0;
        foreach (var item in skill_point_list)
        {
            cut_skill_list.Add(item);
            i++;

            if (i >= robot_count)
            {
                break;
            }
        }

        cpd_command.points_b = cut_skill_list.ToArray();

        print("DO CPD " + cpd_command.points_a.Length + " " + cpd_command.points_b.Length);

        //MemoryStream ms = new MemoryStream();
        //ms = new MemoryStream();
        //Serializer.Serialize<RRSCPDCommand>(ms, cpd_command);
        //byte[] data = ms.ToArray();

        //movo_ref.publisher_cpd_command.Send(data);
        time_cpd_request_start = DateTime.Now;
        Statics.main_cpd_network.sendMessage(cpd_command);
    }

    private void OnGUI()
    {
        GUI.skin.label.fontSize = 10;
        GUI.contentColor = Color.black;

        Vector3 scale = new Vector3();

        scale.x = Screen.width / originalWidth;
        scale.y = Screen.height / originalHeight;
        scale.z = 1.0f;

        GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, scale);

        if (GUI.Button(new Rect(10, 50, 100, 50), "Reset Scene"))
        {
            resetScenario();
        }

        if (GUI.Button(new Rect(150, 50, 100, 50), "Remove Points"))
        {
            removePoints();
        }

        if (skill_point_list.Count == 0)
        {
            GUI.Label(new Rect(10, 150, 300, 50), "Current Step: " + current_step.ToString());
        }
        else
        {
            GUI.Label(new Rect(10, 150, 300, 50), "Current Step: " + current_step.ToString() + " / " + skill_point_list.Count);
        }

        GUI.Label(new Rect(10, 200, 300, 50), "Average Step Time: " + average_teleoperation_two_points_travel_time.ToString());

        if (is_switch_to_skill)
            GUI.Label(new Rect(10, 250, 300, 50), "Network QoS: Disconneted!");
        else
            GUI.Label(new Rect(10, 250, 300, 50), "Network QoS: Connected!");

        string is_rigidt = "Rigid";

        if (is_rigid) is_rigidt = "Rigid"; else is_rigidt = "NonRigid";

        if (operation_mpde == Mode.SharedAutonomyCPD)
            GUI.Label(new Rect(10, 300, 300, 50), "Scenario Mode : " + operation_mpde.ToString() + " " + is_rigidt);
        else
            GUI.Label(new Rect(10, 300, 300, 50), "Scenario Mode : " + operation_mpde.ToString());

        GUI.Label(new Rect(10, 350, 300, 50), "Skill Name : " + skill_file_name);


        if (GUI.Button(new Rect(10, 400, 100, 40), "Nothing"))
        {
            operation_mpde = Mode.Nothing;
        }

        if (GUI.Button(new Rect(10, 450, 100, 40), "Shared"))
        {
            operation_mpde = Mode.SharedAutonomy;
        }

        if (GUI.Button(new Rect(10, 500, 100, 40), "SkillCPD"))
        {
            operation_mpde = Mode.SharedAutonomyCPD;
        }

        if (GUI.Button(new Rect(10, 550, 100, 40), "Play"))
        {
            is_play_scenario = !is_play_scenario;
        }

        //if (GUI.Button(new Rect(10, 600, 100, 40), "Rigid"))
        //{
        //    is_rigid = true;
        //}

        //if (GUI.Button(new Rect(10, 650, 100, 40), "Non-rigid"))
        //{
        //    is_rigid = false;
        //}

        if (GUI.Button(new Rect(10, 710, 100, 40), "Benchmark"))
        {
            bench_state = 1;
        }

        GUI.Label(new Rect(10, 600, 1000, 1000),
        "E: Stop/Start Recording \nP: Show points for robot_point \nR: Stop/Start Recording Skill \nS: Save the Skill  \nW: Save the scenario \nL: Load the skill \nQ: Load the scenario \nT: Network disconnetion on/off \nB: Start tracking");

    }
}
