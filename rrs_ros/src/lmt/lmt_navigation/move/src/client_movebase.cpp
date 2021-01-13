#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <lmt_msgs/MasterAction.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <fstream>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <lmt_msgs/command.h>
using namespace std;

actionlib::SimpleActionClient<lmt_msgs::MasterAction> * ac;

ros::Subscriber sub_navigation_goal;
ros::Subscriber sub_navigation_goal_name;
ros::Publisher pub_navigation_status;
geometry_msgs::PoseStamped current_goal;
ros::ServiceClient client;

int status = 0;
bool thread_exit = false;
int scenario_index = 0;
bool scenario_mode = false;
bool scenario_loop = false;

std::string target_location;

struct goal_data
{
  public :
    int x; //cm
    int y; //cm
    int yaw; //angle - degree
    int height;
    string id;
};

std::vector<goal_data> goal_list;
goal_data current_goal2;

void thread_logic()
{
  while ( ros::ok() && thread_exit == false) 
  {
     boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
     if ( status == 0 )
     {
        ROS_INFO("WAIT FOR GOAL");
     }
     else if ( status == 1)
     {
        ROS_INFO("Sending Goal Raw");
        // send a goal to the action
        lmt_msgs::MasterGoal goal;
        goal.action = "exe";
        goal.goal = current_goal;

        ac->sendGoal(goal);

        //wait for the action to return
        bool finished_before_timeout = ac->waitForResult();

        if (finished_before_timeout)
        {
          actionlib::SimpleClientGoalState state = ac->getState();
          ROS_INFO("Action finished: %s",state.toString().c_str());

          lmt_msgs::MasterResult::ConstPtr _res = ac->getResult();

          ROS_INFO("Action result : %s",_res->result.c_str());
        }
        else
        {
          ac->cancelGoal ();
          ROS_INFO("Action did not finish before the time out.");
        }

        status = 0;
     }
     else if ( status == 2)
     {
        ROS_INFO("Sending Goal Name");
        // send a goal to the action
        lmt_msgs::MasterGoal goal;
        goal.action = "exe2";
        goal.iParam1 = current_goal2.x;
        goal.iParam2 = current_goal2.y;
        goal.iParam3 = current_goal2.yaw;
        goal.id = current_goal2.id;

        ac->sendGoal(goal);

        //wait for the action to return
        bool finished_before_timeout = ac->waitForResult();

        if (finished_before_timeout)
        {
          actionlib::SimpleClientGoalState state = ac->getState();
          ROS_INFO("Action finished: %s",state.toString().c_str());

          lmt_msgs::MasterResult::ConstPtr _res = ac->getResult();

          ROS_INFO("Action result : %s",_res->result.c_str());
        }
        else
        {
          ac->cancelGoal ();
          ROS_INFO("Action did not finish before the time out.");
        }

        if ( scenario_mode )
        {
          scenario_index++;
        }

        status = 0;
     }

  }

  
  //ac->cancelGoal ();

}

void read_file()
{
        std::string path_points =  ros::package::getPath("managment") + "/maps/points.txt";
        cout<<path_points<<endl;
        std::string line;
        std::ifstream text;

        goal_list.clear();
        text.open(path_points.c_str(), ios_base::in);

        if (text.is_open())
        {
          
            getline(text,line);

            while (text.good())
            {
                vector <string> fields;

                boost::split( fields, line, boost::is_any_of( "," ) );
                cout<<line<<endl;
                goal_data gdata;

                 gdata.id = fields[0].c_str();
                 gdata.x = atoi(fields[1].c_str());
                 gdata.y = atoi(fields[2].c_str());
                 gdata.yaw = atoi(fields[3].c_str());
                 gdata.height = atoi(fields[4].c_str());

                goal_list.push_back(gdata);
               
                getline(text,line);

            }
            text.close();

        }
        else
        {
            std::cout << "[Unable to open file]"  <<std::endl << std::endl;
        }

        std::cout << "read done : "  << goal_list.size()<<std::endl << std::endl;
}

void chatterCallbackGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   if ( status == 0)
   {
     current_goal = *msg;
     status = 1;
   }
   else if ( status != 0)
   {
     if ( msg->pose.position.x == 0 && msg->pose.position.y == 0 && msg->pose.position.z == 0)
     {
      ac->cancelGoal ();
      status = 0;
     }
   }
}

void sendForceStop()
{
  lmt_msgs::command srv;

  srv.request.command = "cancel";
 
  if (client.call(srv))
  {
    ROS_INFO("cancel command done");
  }
  else
  {
    ROS_ERROR("Failed to call service, lmt command");
  }
}

void chatterCallbackGoalName(const std_msgs::String::ConstPtr& msg)
{
   if ( status == 0) //idle
   {
     std::string name = msg->data;

     if ( scenario_mode == false)
     {
          if ( name == "scenario_start" )
          {
            scenario_index = 1;
            scenario_mode = true;
          }
          else
          {
              if ( scenario_mode == false)
              {
                  for ( int i =0 ; i < goal_list.size() ; i++)
                  {
                      if ( goal_list.at(i).id == name )
                      {
                          
                              current_goal2.x = goal_list.at(i).x;
                              current_goal2.y = goal_list.at(i).y;
                              current_goal2.yaw = goal_list.at(i).yaw;
                              current_goal2.id = name;
                            

                              std::cout << "next goal : "  << name << std::endl;
                              status = 2;
                              break;
                      }

                    
                  }
              }

          }
     }
     else
     {
       if ( msg->data == "cancel" || msg->data == "scenario_stop")
       {
         sendForceStop();
         ac->cancelGoal ();
         status = 0;
         scenario_mode = false;
         scenario_index  = 0;
       }
      
       
     }
     

   }
   else if ( status != 0)
   {
     if ( msg->data == "cancel" || msg->data == "scenario_stop")
     {
      sendForceStop();
      ac->cancelGoal ();
      status = 0;
      scenario_mode = false;
      scenario_index  = 0;
     }
   }
}

void publishStatus(int int_status)
{
  std_msgs::String msg;

  if ( int_status == 0)
  msg.data = "idle";

  if ( int_status == 1)
  msg.data = "move";

  pub_navigation_status.publish(msg);
}

void scnearioLoop()
{
    if ( scenario_mode )
    {
      if ( status == 0) //idle or done
      {
           if ( scenario_index < goal_list.size() )
           {
              current_goal2.x = goal_list.at(scenario_index).x;
              current_goal2.y = goal_list.at(scenario_index).y;
              current_goal2.yaw = goal_list.at(scenario_index).yaw;
              current_goal2.id = goal_list.at(scenario_index).id;

              std::cout << "next scenario goal : "  << current_goal2.id << std::endl;
              status = 2;
           }
           else
           {
               scenario_mode = false;
               scenario_index = 0;
               std::cout << "***** Scenario done. ******"  << std::endl;

               if ( scenario_loop)
               {
                 scenario_mode = true;
                 std::cout << "Scenario loop, start the scenario again"  << std::endl;
               }
           }
           
      }
    }
}



int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_movebase");

  read_file();

  ros::NodeHandle nh;

  sub_navigation_goal = nh.subscribe("goal",1, chatterCallbackGoal);
  sub_navigation_goal_name = nh.subscribe("goal_name",1, chatterCallbackGoalName);
  pub_navigation_status =  nh.advertise<std_msgs::String>("navigation/status",1);

  ROS_INFO("Waiting for action server to start...");

  ac = new actionlib::SimpleActionClient<lmt_msgs::MasterAction>("LMTMoveBaseAction", true);
  client = nh.serviceClient<lmt_msgs::command>("lmt/movebase/command");

  // wait for the action server to start
  ac->waitForServer(); //will wait for infinite time

  ROS_INFO("Action server found");

  boost::thread _thread_(&thread_logic);

  ros::Rate loop(10);

  while(ros::ok())
  {
    ros::spinOnce();
    loop.sleep();
    publishStatus(status);
    scnearioLoop();
  }

  thread_exit = true;

  _thread_.interrupt();
  _thread_.join();

  //exit
  return 0;
}
