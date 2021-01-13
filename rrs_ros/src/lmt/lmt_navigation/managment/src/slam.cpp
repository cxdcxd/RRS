#include "ros/ros.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <actionlib/server/simple_action_server.h>
#include <lmt_msgs/slamactionAction.h>

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <fstream>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/package.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <lmt_msgs/maptools.h>

#include <cstdio>
#include "ros/ros.h"
#include "ros/console.h"
#include "nav_msgs/GetMap.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "geometry_msgs/Quaternion.h"

#include <lmt_msgs/maptools.h>

using namespace std;
using namespace boost;

nav_msgs::OccupancyGrid global_map;

typedef actionlib::SimpleActionServer<lmt_msgs::slamactionAction> slam_Server;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBase_Client;

MoveBase_Client *globalClient_movebase;
slam_Server *globalServer;

float slam_position_yaw[3] = {0};
float goal_position_yaw[3] = {0};
float delta_position_yaw[3] = {0};
bool App_exit = false;
float distacne_to_goal = 0;
ros::Publisher chatter_pub[20];
bool first_time = false;

ros::ServiceClient client;
lmt_msgs::maptools srv;

struct goal_data
{
  public :
    float x;
    float y;
    int yaw; //angle
    string id;
};

std::vector<goal_data> goal_list;

float ConvertQuatToYaw(const geometry_msgs::Quaternion &quat)
{
    float yaw = tf::getYaw(quat);
    return isnan(yaw) ? 0 : yaw;
}

inline float Deg2Rad(float deg) {
    return deg * M_PI / 180;
}

inline float Rad2Deg(float rad) {
    return rad * 180 / M_PI;
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr &map)
    {
        global_map.data = map->data;
        global_map.header = map->header;
        global_map.info = map->info;
        ROS_INFO("get map");

    }

void PreemptThread()
{
  
   while ( App_exit == false )
   {

        if (globalServer->isPreemptRequested() )
        {
            ROS_INFO(" cancle requested\n");
            globalClient_movebase->cancelGoal();
            globalServer->setAborted();
            ROS_INFO(" aborted done\n");
            break;
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
   }
}

void chatterCallback_pose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    //cout<<"get"<<endl;
    geometry_msgs::Pose pose;
    geometry_msgs::Point point;
    geometry_msgs::Quaternion quaternion;
    pose = msg->pose;

    slam_position_yaw[0] = pose.position.x; //meter
    slam_position_yaw[1] = pose.position.y; //meter
    slam_position_yaw[2] = ConvertQuatToYaw(pose.orientation); //radians

    //================ delta
    
    delta_position_yaw[0] = goal_position_yaw[0] - slam_position_yaw[0];
    delta_position_yaw[1] = goal_position_yaw[1] - slam_position_yaw[1];
    delta_position_yaw[2] = goal_position_yaw[2] - slam_position_yaw[2];

    distacne_to_goal = delta_position_yaw[0] * delta_position_yaw[0] + delta_position_yaw[1] * delta_position_yaw[1];
    distacne_to_goal = sqrt(distacne_to_goal);

   //cout<<delta_position_yaw[0]<<" "<<delta_position_yaw[1]<<" "<<delta_position_yaw[2]<<" "<<distacne_to_goal<<endl;
}

void read_file()
{

        std::string path_points =  ros::package::getPath("managment") + "/maps/points.txt";
        cout<<path_points<<endl;
        std::string line;
        std::ifstream text;

        text.open(path_points.c_str(), ios_base::in);

        if (text.is_open())
        {
            getline(text,line);

             while (text.good())
            {
                vector <string> fields;

                boost::split( fields, line, boost::is_any_of( "," ) );

                goal_data gdata;
                gdata.id = fields[0].c_str();
                gdata.x = atof(fields[1].c_str());
                gdata.y = atof(fields[2].c_str());
                gdata.yaw = atof(fields[3].c_str());

                gdata.x = gdata.x / 100;
                gdata.y = gdata.y / 100;

                goal_list.push_back(gdata);
                //std::cout<<"read "<<goal_list.size()<<" ponts"<<endl;

                getline(text,line);

            }
            text.close();



        }
        else
        {
            std::cout << "Unable to open file" << std::endl << std::endl;
        }

        std::cout << "read done : " << goal_list.size()<<std::endl << std::endl;

}

void savepgmmap(string name)
{
  

            ROS_INFO("Received a %d X %d global_map @ %.3f m/pix",
                     global_map.info.width,
                     global_map.info.height,
                     global_map.info.resolution);


            std::string global_mapdatafile =  ros::package::getPath("athomerobot") +"/" + name +".pgm";
            ROS_INFO("Writing global_map occupancy data to %s", global_mapdatafile.c_str());
            FILE *out = fopen(global_mapdatafile.c_str(), "w");
            if (!out)
            {
                ROS_ERROR("Couldn't save global_map file to %s", global_mapdatafile.c_str());
            }

            fprintf(out, "P5\n# CREATOR: global_map_generator.cpp %.3f m/pix\n%d %d\n255\n",
                    global_map.info.resolution, global_map.info.width, global_map.info.height);
            for (unsigned int y = 0; y < global_map.info.height; y++)
            {
                for (unsigned int x = 0; x < global_map.info.width; x++)
                {
                    unsigned int i = x + (global_map.info.height - y - 1) * global_map.info.width;
                    if (global_map.data[i] == 0)   //occ [0,0.1)
                    {
                        fputc(254, out);
                    }
                    else if (global_map.data[i] == +100)     //occ (0.65,1]
                    {
                        fputc(000, out);
                    }
                    else     //occ [0.1,0.65]
                    {
                        fputc(205, out);
                    }
                }
            }

            fclose(out);


            std::string global_mapmetadatafile = ros::package::getPath("athomerobot") + "/" + name + ".yaml";
            ROS_INFO("Writing global_map occupancy data to %s", global_mapmetadatafile.c_str());
            FILE *yaml = fopen(global_mapmetadatafile.c_str(), "w");

            geometry_msgs::Quaternion orientation = global_map.info.origin.orientation;
            tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
            double yaw, pitch, roll;
            mat.getEulerYPR(yaw, pitch, roll);

            fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
                    global_mapdatafile.c_str(), global_map.info.resolution, global_map.info.origin.position.x, global_map.info.origin.position.y, yaw);

            fclose(yaml);

            ROS_INFO("Done\n");
}

void savepngfrommap_emergency()
{
   //savepgmmap("map");
   //ROS_INFO("pgm saved...");
   //send a request to map saver to save the map as pgm image
   //use opencv to open it
   //use opencv to draw a shape and ifo on it
   //use opencv to save it as a png image
    cv::Mat image;

    string pgm_path = ros::package::getPath("athomerobot") + "/map.pgm";

    image = cv::imread( pgm_path,1 );

    int face[] = {cv::FONT_HERSHEY_SIMPLEX, cv::FONT_HERSHEY_PLAIN, cv::FONT_HERSHEY_DUPLEX, cv::FONT_HERSHEY_COMPLEX,
                  cv::FONT_HERSHEY_TRIPLEX, cv::FONT_HERSHEY_COMPLEX_SMALL, cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
                  cv::FONT_HERSHEY_SCRIPT_COMPLEX, cv::FONT_ITALIC};

    cv::Mat temp = image;
    cv::resize(temp,temp,cv::Size(500,500),0,0,cv::INTER_LINEAR);
    string imagePath =ros::package::getPath("emergency") + "/png/map1.png";
    imwrite( imagePath, temp );
    cout<<"write done..."<<endl;


    for ( int i = 0 ; i < goal_list.size() ; i++)
    {
        goal_data gdata = goal_list.at(i) ;
        int x = 512 + gdata.x * 40;
        int y = 512 - gdata.y * 40;
         if ( gdata.id.at(0) == '$')
         {
        cv::circle( image, cv::Point(x,y), 5, cv::Scalar(200,0,0), -1, CV_AA);
        cv::putText(image, gdata.id , cv::Point(x,y), face[0], 1, cv::Scalar(0,0,200), 1, CV_AA);
         }
    }



    temp = image;
    cv::resize(temp,temp,cv::Size(500,500),0,0,cv::INTER_LINEAR);
    imagePath =ros::package::getPath("emergency") + "/png/map2.png";
    imwrite( imagePath, temp );

    cout<<"write done..."<<endl;


}

void savepngfrommap_restaurant()
{
   savepgmmap("map_restaurant");
   ROS_INFO("pgm saved...");

    cv::Mat image;

    string pgm_path = ros::package::getPath("athomerobot") + "/map_restaurant.pgm";

    image = cv::imread( pgm_path,1 );

    int face[] = {cv::FONT_HERSHEY_SIMPLEX, cv::FONT_HERSHEY_PLAIN, cv::FONT_HERSHEY_DUPLEX, cv::FONT_HERSHEY_COMPLEX,
                  cv::FONT_HERSHEY_TRIPLEX, cv::FONT_HERSHEY_COMPLEX_SMALL, cv::FONT_HERSHEY_SCRIPT_SIMPLEX,
                  cv::FONT_HERSHEY_SCRIPT_COMPLEX, cv::FONT_ITALIC};


    for ( int i = 0 ; i < goal_list.size() ; i++)
    {
        goal_data gdata = goal_list.at(i) ;
        int x = 512 + gdata.x * 40;
        int y = 512 - gdata.y * 40;
         if ( gdata.id.at(0) == '$')
         {
           cv::circle( image, cv::Point(x,y), 5, cv::Scalar(200,0,0), -1, CV_AA);
           cv::putText(image, gdata.id , cv::Point(x,y), face[0], 1, cv::Scalar(0,0,200), 1, CV_AA);
         }
    }

    string imagePath =ros::package::getPath("restaurant") + "/png/restaurant.png";
    imwrite( imagePath, image );
    cout<<"png write done..."<<endl;

}

void adduserpointonmap(bool marked , string id)
{
   goal_data gdata;
   gdata.x = slam_position_yaw[0];
   gdata.y = slam_position_yaw[1];
   gdata.yaw = slam_position_yaw[2];
   if ( marked == false )
   gdata.id = id;
   else
       gdata.id = "$" + id;

   goal_list.push_back(gdata);

 cout<<"user point add"<<" "<<marked<<endl;
   //we dont need to save this point on HDD
   //it is valid just to end of app
}

bool checkmaptools(lmt_msgs::maptools::Request  &req,lmt_msgs::maptools::Response &res)
{
    std::string str = req.command;
    std::string id = req.id;

    //command => savepoint ( string id )
    //command => savemarkedpoint ( string id )
    //command => savemap

    if ( str == "savepoint")
    {
       adduserpointonmap(false,id);
    }
    if ( str == "savemarkedpoint")
    {
       adduserpointonmap(true,id);
    }
    if ( str == "savemap")
    {
      savepngfrommap_emergency();
    }
    if ( str == "savepgmmap")
    {
      savepgmmap("map");
    }
    if ( str == "res_savepoint")
    {
      adduserpointonmap(true,id);
    }
    if ( str == "res_savemap")
    {
      savepngfrommap_restaurant();
    }

}

void init_gols()
{
   goal_data mygoal;
   mygoal.x = -0.12;
   mygoal.y = 8.83;
   mygoal.yaw = 85;
   mygoal.id = "PPS";
   goal_list.push_back(mygoal);

   mygoal.x = -0.77;
   mygoal.y = 5.63;
   mygoal.yaw = -92;
   mygoal.id = "PPG";
   goal_list.push_back(mygoal);

   mygoal.x = 2.97;
   mygoal.y = -0.41;
   mygoal.yaw = -90;
   mygoal.id = "trushbin";
   goal_list.push_back(mygoal);

   // mygoal.x = 4;
   // mygoal.y = -2.3;
   // mygoal.yaw = -90;
   // mygoal.id = "WDYS";

    mygoal.x = 2;
    mygoal.y = 0;
    mygoal.yaw = 0;
    mygoal.id = "WDYS";

   goal_list.push_back(mygoal);

   mygoal.x = 0;
   mygoal.y = 0;
   mygoal.yaw = 0;
   mygoal.id = "Origin";
   goal_list.push_back(mygoal);

   mygoal.x = 0.13;
   mygoal.y = 1;
   mygoal.yaw = 90;
   mygoal.id = "ATS";
   goal_list.push_back(mygoal);

   mygoal.x = 0.58;
   mygoal.y = 0.67;
   mygoal.yaw = 90;
   mygoal.id = "ATF";
   goal_list.push_back(mygoal);
}

int find_goal_byname(string name)
{
    for ( int i = 0 ; i < goal_list.size() ; i++ )
    {
        if ( name == goal_list.at(i).id )
        {
           return i;
        }
    }

    return -1;
}

class myactionserver
{
protected:

  ros::NodeHandle nh_;

  actionlib::SimpleActionServer<lmt_msgs::slamactionAction> as_;
  std::string action_name_;

  lmt_msgs::slamactionFeedback feedback_;
  lmt_msgs::slamactionResult result_;

public:

  myactionserver(std::string name) :
    as_(nh_, name, boost::bind(&myactionserver::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    globalServer = &as_;
  }

  ~myactionserver(void)
  {
  }

  void executeCB(const lmt_msgs::slamactionGoalConstPtr &interfacegoal)
  {
    boost::thread Preempt_thread(&PreemptThread);

    bool success = true;

    ROS_INFO("EXECUTE");
    cout<<interfacegoal->ID<<endl;
    //goal_position_yaw[0] = -0.776;
    //goal_position_yaw[1] = -3;
    //goal_position_yaw[2] = 0;

    if ( interfacegoal->ID == "")
    {
       goal_position_yaw[0] = interfacegoal->x;
       goal_position_yaw[1] = interfacegoal->y;
       goal_position_yaw[2] = Deg2Rad(interfacegoal->yaw);

        ROS_INFO("EXECUTE NORMAL");
    }
    else
    {
       int index = find_goal_byname(interfacegoal->ID);
       if ( index != -1)
       {
           goal_data data = (goal_data)goal_list.at(index);
           goal_position_yaw[0] = data.x;
           goal_position_yaw[1] = data.y;
           goal_position_yaw[2] = Deg2Rad(data.yaw);
           string log = "find " + data.id + " ";
           cout<<log<<index<<endl;
       }
       else
       {
           success = false;
           result_.result= "ID NOT FOUND";
           ROS_INFO("NOTFIND");
       }
    }


    //=========================================
    if ( success )
    {
        MoveBase_Client ac("move_base", true);
        globalClient_movebase = &ac;
        while(!ac.waitForServer(ros::Duration(1))){
          ROS_INFO("Waiting for the move_base action server to come up");
        }

        move_base_msgs::MoveBaseGoal goal;

        while(!App_exit)
        {
           goal.target_pose.header.frame_id = "base_link";
           goal.target_pose.header.stamp = ros::Time::now();
           goal.target_pose.pose.position.x = delta_position_yaw[0] * cos(slam_position_yaw[2]) + delta_position_yaw[1]*sin(slam_position_yaw[2]);
           goal.target_pose.pose.position.y = -delta_position_yaw[0] * sin(slam_position_yaw[2]) + delta_position_yaw[1]*cos(slam_position_yaw[2]);
           goal.target_pose.pose.position.z = 0;
           geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(delta_position_yaw[2]);
           goal.target_pose.pose.orientation = quat;

           ac.sendGoal(goal);

           bool finishedBeforeTimeout = ac.waitForResult(ros::Duration(2.0));

           if ( finishedBeforeTimeout == true)
           {
               if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
               {
                  success = true;
                  break;
               }
               else
               {
                  success = false;
                  break;
               }
           }

           //r.sleep();
        }

     }
    //=========================================

    if(success)
    {
       result_.result= "DONE";
       cout<<"SLAM DONE"<<endl;
       as_.setSucceeded(result_);
    }
    else
    {
        result_.result= "MOVEBASE FAILED";
        cout<<"SLAM FAILED"<<endl;
        as_.setSucceeded(result_);
    }

      Preempt_thread.interrupt();
      Preempt_thread.join();
  }


};



int main(int argc, char** argv)
{

  ros::init(argc, argv, "navigation");
  cout << "LMT SLAM STARTED DONE" << endl;
  // init_gols();
  read_file();

  myactionserver act("slam_action");

  ros::NodeHandle node_handles[15];
  ros::Subscriber sub_handles[15];

  sub_handles[0] = node_handles[0].subscribe("/slam_out_pose", 10, chatterCallback_pose);

  ros::NodeHandle n_service;
  ros::ServiceServer service_maptools = n_service.advertiseService("AUTROBOTINSRV_maptools", checkmaptools);

  ros::NodeHandle n_client;
  client = n_client.serviceClient<lmt_msgs::maptools>("AUTROBOTINSRV_mapsaver");

  ros::NodeHandle n;
  ros::Subscriber global_map_sub_ = n.subscribe("map", 1, mapCallback);

  ros::Rate ros_rate(30);
  
  while(ros::ok())
  {
    ros::spinOnce();
    ros_rate.sleep();
  }

  App_exit = true;


  return 0;
}
