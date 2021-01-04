#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <sepanta_msgs/command.h>
#include <sepanta_msgs/omnidata.h>
#include <sepanta_msgs/sepantaAction.h> //movex movey turngl turngllocal actions
#include <sepanta_msgs/slamactionAction.h> //slam action

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

//#include <std_msgs/Bool.h>
#include "sepanta_msgs/stop.h"

typedef actionlib::SimpleActionClient<sepanta_msgs::slamactionAction> SLAMClient;
typedef actionlib::SimpleActionClient<sepanta_msgs::sepantaAction> SepantaClient;

ros::ServiceClient serviceclient_odometryslam;
ros::ServiceClient serviceclient_facestop;
ros::ServiceClient serviceclient_manualauto;
ros::ServiceClient serviceclient_map;


SLAMClient *globalSLAM;
SepantaClient *globalSepanta;

bool App_exit = false;
ros::Publisher chatter_pub[20];

using namespace std;
bool flag1 = false;
bool flag2 = false;
int old = 0;

std::string service_command = "";
std::string service_id = "";
int service_value = 0 ;


void goWithOdometry(string mode,int value)
{

    ROS_INFO("Going %s with odometry...", mode.c_str());
    sepanta_msgs::sepantaGoal interfacegoal;
    interfacegoal.type = mode;
    interfacegoal.value = value;

    actionlib::SimpleClientGoalState state = globalSepanta->sendGoalAndWait(interfacegoal, ros::Duration(1000), ros::Duration(1000));

    if (state == actionlib::SimpleClientGoalState::PREEMPTED)
    {
      ROS_INFO("ODOMETRY PREEMPTED");
    }
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("ODOMETRY SUCCEEDED");
    }
    if (state == actionlib::SimpleClientGoalState::ABORTED)
    {
      ROS_INFO("ODOMETRY ABORTED");
    }
}

void goWithSlam(string where)
{
    ROS_INFO("Going %s with slam...", where.c_str());
    sepanta_msgs::slamactionGoal interfacegoal;
    interfacegoal.x = 0;
    interfacegoal.y = 0;
    interfacegoal.yaw = 0;
    interfacegoal.ID = where;

    ROS_INFO("goal sent to slam... waiting for reach there.");
    cout << where << endl;

     actionlib::SimpleClientGoalState state = globalSLAM->sendGoalAndWait(interfacegoal, ros::Duration(1000), ros::Duration(1000));

     if (state == actionlib::SimpleClientGoalState::PREEMPTED)
     {
       ROS_INFO("SLAM PREEMPTED");
     }
     if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
     {
       ROS_INFO("SLAM SUCCEEDED");
     }
     if (state == actionlib::SimpleClientGoalState::ABORTED)
     {
       ROS_INFO("SLAM ABORTED");
     }
}

void service_thread2()
{
	
    while ( App_exit == false && ros::ok())
    {


	    if (service_command == "gotocancle")
	    {
	         ROS_INFO("GOTOCANCLE");
	         globalSLAM->cancelGoal();
	         service_command = "";
	         service_id = "";
	         service_value = 0;

	    }
	    else if (service_command == "odometrycancle")
	    {
	         ROS_INFO("ODOMETRYCANCLE");
	         globalSepanta->cancelGoal();
	         service_command = "";
	         service_id = "";
	         service_value = 0;
	    }
    

    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
}

void service_thread()
{
	
    while ( App_exit == false && ros::ok())
    {

    if (service_command == "goto")
    {
    	
         ROS_INFO("GET GOTO");
         goWithSlam(service_id); 
         service_command = "";
         service_id = "";
         service_value = 0;
    }
    else if (service_command == "movex")
    {
         ROS_INFO("MOVEX");
         goWithOdometry(service_command,service_value);
         service_command = "";
         service_id = "";
         service_value = 0;
    }
    else if (service_command == "movey")
    {
         ROS_INFO("MOVEY");
         goWithOdometry(service_command,service_value);
         service_command = "";
         service_id = "";
         service_value = 0;
    }
    else if (service_command == "turngl")
    {
         ROS_INFO("TURNGL");
         goWithOdometry(service_command,service_value);
         service_command = "";
         service_id = "";
         service_value = 0;
    }
    else if (service_command == "turngllocal")
    {
    	 
         ROS_INFO("TURNGLLOCAL");
         goWithOdometry(service_command,service_value);
         service_command = "";
         service_id = "";
         service_value = 0;
    }
    
   

    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
}

bool checkcommand(sepanta_msgs::command::Request  &req,sepanta_msgs::command::Response &res)
{
    std::string str = req.command;
    std::string id = req.id;
    int value = req.value;

    ROS_INFO("Service Request....");

    service_command = str;
    service_id = id;
    service_value = value;
   
    res.result = "done";
    return true;
}

void omnidrive(int x,int y,int w)
{
   sepanta_msgs::omnidata msg_data;
   msg_data.d0 = x;
   msg_data.d1 = y;
   msg_data.d2 = w;

   chatter_pub[0].publish(msg_data);
}

void calibration_test()
{
    return;
    cout << "calib start start" << endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));


    //reset_position();

    omnidrive(0, 0, 30);
    boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
    omnidrive(0, 0, 0);
    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

    cout << "back" << endl;


    omnidrive(0, 0, -30);
    boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
    omnidrive(0, 0, 0);
    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

    cout << "finish" << endl;
    // cout << position[0] << " , " << position[1] << " , " << position[2] << " , " << delta<< endl;
}


void checkkeypad(const std_msgs::Int32::ConstPtr &topicMsg)
{
    int value = topicMsg->data ;
     //ROS_INFO("%d",value);

    if ( value != old)
    {

     ROS_INFO("%d",value);
     old = value;

    if(value==3)
    {
      omnidrive(0,0,0);

      // std_msgs::Bool msg;
      // msg.data = true;
      // chatter_pub[2].publish(msg);

    }
    if(value== 7)
    {
      omnidrive(200,0,0);
    }
    if(value==8)
    {
      omnidrive(-200,0,0);
    }
    if(value==9)
    {
      omnidrive(0,200,0);
    }
    if(value==10)
    {
     omnidrive(0,-200,0);
    }
    if(value==5)
    {
      omnidrive(0,0,220);
    }
    if(value==6)
    {
      omnidrive(0,0,-220);
    }

    //================

    if ( value == 1)
    {
    	//manual mode start
    	// sepanta_msgs::stop srv_stop;
  		// srv_stop.request.command = "Manual";
  		// serviceclient_manualauto.call(srv_stop);
    }
    if ( value == 4)
    {
    	//slam mode start
    	//sepanta_msgs::stop srv_stop;
   		//srv_stop.request.command = "Slam";
  		//serviceclient_manualauto.call(srv_stop);
    }
    if ( value == 2)
    {
    	//manual reached
    	//sepanta_msgs::stop srv_stop;
  		//srv_stop.request.command = "ManualReached";
  		//serviceclient_manualauto.call(srv_stop);
        // std_msgs::Bool msg;
        // msg.data = false;
        // chatter_pub[2].publish(msg);
    }


    }



}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation_client");
  ros::Time::init();

  cout << "SEPANTA SLAM CLIENTS SERVICES STARTED DONE (93/04/05)" << endl;

  ros::NodeHandle n_service;
  ros::ServiceServer service_command = n_service.advertiseService("AUTROBOTINSRV_command", checkcommand);

  ros::NodeHandle node_handles[15];
  chatter_pub[0] = node_handles[0].advertise<sepanta_msgs::omnidata>("AUTROBOTIN_omnidrive", 10);
  chatter_pub[1] = node_handles[1].advertise<std_msgs::Bool>("AUTROBOTIN_greenlight", 10);
  chatter_pub[2] = node_handles[2].advertise<std_msgs::Bool>("AUTROBOTIN_redlight", 10);

  ros::Subscriber sub_handles[1];
  sub_handles[0] = node_handles[1].subscribe("AUTROBOTOUT_keypad", 10, checkkeypad);

  ros::NodeHandle n_client1;
  serviceclient_facestop = n_client1.serviceClient<sepanta_msgs::stop>("speechOrFace_Stop");

  ros::NodeHandle n_client2;
  serviceclient_manualauto = n_client2.serviceClient<sepanta_msgs::stop>("manualOrAuto");

  SLAMClient slamAction("slam_action", true);
  globalSLAM = &slamAction ;
  slamAction.waitForServer();
  ROS_INFO("connected to slam server");

  //===========================================

  SepantaClient sepantaAction("sepanta_action", true);
  globalSepanta = &sepantaAction ;
  sepantaAction.waitForServer();
  ROS_INFO("connected to sepanta server");

  ros::Rate ros_rate(20);

   boost::thread _thread_logic(&calibration_test);
   boost::thread _thread_serevice(&service_thread);
   boost::thread _thread_serevice2(&service_thread2);

  while(ros::ok())
  {
    ros::spinOnce();
    ros_rate.sleep();
  }

  App_exit = true;

  _thread_logic.interrupt();
  _thread_logic.join();

  _thread_serevice.interrupt();
  _thread_serevice.join();

  _thread_serevice2.interrupt();
  _thread_serevice2.join();

  return 0;
} 
