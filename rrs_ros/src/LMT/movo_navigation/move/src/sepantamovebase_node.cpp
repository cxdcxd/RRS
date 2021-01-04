#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <sepanta_msgs/command.h>
#include <sepanta_msgs/MasterAction.h>
#include <actionlib/server/simple_action_server.h>
#include <sepantamovebase.h>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;
using namespace boost;
using namespace ros;

SepantaMoveBase *_sepantamovebase;

class MoveBaseAction
{

public:

    actionlib::SimpleActionServer<sepanta_msgs::MasterAction> action_server_;
    std::string action_name_;
    sepanta_msgs::MasterFeedback feedback_;
    sepanta_msgs::MasterResult result_;
  
    MoveBaseAction(ros::NodeHandle nh, std::string name) :
    action_server_(nh, name,boost::bind(&MoveBaseAction::executeCB, this, _1), false) ,
    action_name_(name)
    {

    action_server_.start();
    }


void executeCB(const sepanta_msgs::MasterGoalConstPtr &goal) 
{
    if (!action_server_.isActive()) return;

    ROS_INFO("Action Started");
    cout<<"issepantamove : "<< _sepantamovebase->isrobotmove << endl;
    std::string _cmd = goal->action;
    std::string _id = goal->id;
    int _value1 = goal->iParam1;
    int _value2 = goal->iParam2;
    int _value3 = goal->iParam3;

    if ( _cmd == "reload_points")
    {
        _sepantamovebase->read_file();
    }

    if ( _cmd == "save_map")
    {
        _sepantamovebase->sepantamapengine_savemap();
    }

    if ( _cmd == "load_map")
    {
        _sepantamovebase->sepantamapengine_loadmap();
    }

    if ( _cmd == "exe")
    {
        goal_data g;
        g.id = _id;
        g.x = goal->goal.pose.position.x;
        g.y = goal->goal.pose.position.z;
        g.yaw = 0;

        _sepantamovebase->exe_slam(g);
    }

    if ( _cmd == "exe2")
    {
        goal_data g;
        g.id = _id;
        g.x = _value1;
        g.y = _value2;
        g.yaw = _value3;

        ROS_INFO_STREAM("NAME GOAL " << "Name : " << g.id << " " << g.x << " " << g.y << " " << g.yaw);

        _sepantamovebase->exe_slam(g);
    }

    if ( _cmd == "cancel")
    {
        _sepantamovebase->exe_cancel();
    }

    if ( _cmd == "reset_hector")
    {
        _sepantamovebase->reset_hector_slam();
    }

    if ( _cmd == "update_hector_origin")
    {

        _sepantamovebase->update_hector_origin(0,0,0);
    }

    cout<<"issepantamove : "<< _sepantamovebase->getrobotmove() << endl;
    boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
    cout<<"issepantamove : "<< _sepantamovebase->getrobotmove() << endl;

    while(ros::ok() && _sepantamovebase->getrobotmove()  )
    {
       boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

        if ( action_server_.isPreemptRequested() )
        {        
                _sepantamovebase->exe_cancel();
                action_server_.setPreempted();
                return;
        }
    }

    string result = _sepantamovebase->getlastnavigationresult();
   
    result_.result = result;

    if ( result != "GOAL REACHED")
        action_server_.setAborted(result_);
    else
      action_server_.setSucceeded(result_);
}

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "movebase_action");
    ROS_INFO("sepantamovebase action server started");
    ros::Time::init();

    ros::NodeHandle n;
   
    _sepantamovebase = new SepantaMoveBase();
    MoveBaseAction *_movebaseaction = new  MoveBaseAction(n,"SepantaMoveBaseAction");

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
