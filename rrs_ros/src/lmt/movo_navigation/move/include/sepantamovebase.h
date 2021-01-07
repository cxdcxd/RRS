#ifndef _SEPANTA_MOVE_BASE_H
#define _SEPANTA_MOVE_BASE_H

#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <ctime>
#include <unistd.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <tbb/atomic.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sepanta_msgs/omnidata.h"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PolygonStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include <termios.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <sepanta_msgs/command.h>
#include <sepanta_msgs/omnidata.h>
#include <sepanta_msgs/sepantaAction.h> //movex movey turngl turngllocal actions
#include <sepanta_msgs/slamactionAction.h> //slam action
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <fstream>
#include <mutex>

//#include <nav_core/base_local_planner.h>
//#include <nav_core/base_global_planner.h>
//include <nav_core/recovery_behavior.h>
#include <geometry_msgs/PoseStamped.h>
//#include <costmap_2d/costmap_2d_ros.h>
//#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tbb/atomic.h>

#include <sepantamovebase/PointList.h>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;

using namespace std;
using namespace boost;
using namespace ros;

//DESIRE ERRORS
#define normal_desire_errorX 0.2
#define normal_desire_errorY 0.2
#define normal_desire_errorTetha 0.36 // 20 degree
#define goal_desire_errorX normal_desire_errorX / 3
#define goal_desire_errorY normal_desire_errorY / 3
#define goal_desire_errorTetha normal_desire_errorTetha / 4

struct goal_data
{
  public :
    int x; //cm
    int y; //cm
    int yaw; //angle - degree
    int height;
    string id;
};

inline double Deg2Rad(double deg)
{
    return deg * M_PI / 180;
}

inline double Rad2Deg(double rad)
{
    return rad * 180 / M_PI;
}


class SepantaMoveBase
{
public:
bool is_sim = true;
SepantaMoveBase();
~SepantaMoveBase();
double Quat2Rad(double orientation[]);
void publish_isrobotmove();
void say_message(string data);
void send_omni(double x,double y ,double w);
void force_stop();
double GetDistance(double x1, double y1, double x2, double y2);
int GetCurrentStep();
void sepantamapengine_savemap();
void sepantamapengine_loadmap();
void clean_costmaps();
void update_hector_origin(float x,float y,float yaw);
void reset_hector_slam();
void read_file();
void read_config();
void SaveLastPosition();
int find_goal_byname(string name);
nav_msgs::Path call_make_plan();
void logic_thread();
void vis_thread();
void Localization_thread();
void exe_slam(goal_data g);
void exe_cancel();
int sign(double data);
int roundData(double data);
double GetToPointsAngle(double x1, double y1, double x2, double y2);
void ResetLimits();
void ReduceLimits();
int calc_next_point();
void errors_update();
void publish_info();
void kill();
void controller_update(int x,bool y,bool theta);
void PathFwr();
void GetCostmap(const nav_msgs::OccupancyGrid::ConstPtr &msg);
bool calc_error(double x1,double y1,double t1,double x2,double y2,double t2,double delta_t);
void hector_problem_detected();
void GetAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
void GetPos(const geometry_msgs::PoseStamped::ConstPtr &msg);
void CheckHectorStatus(const std_msgs::Bool::ConstPtr &msg);
void chatterCallback_ttsfb(const std_msgs::String::ConstPtr &msg);
void chatterCallbackPoints(const sepantamovebase::PointList::ConstPtr &msg);
void chatterCallbackTags(const sepantamovebase::PointList::ConstPtr &msg);
bool checkcommand(sepanta_msgs::command::Request  &req,sepanta_msgs::command::Response &res);
bool getrobotmove();
void playVoice(std::string text);
void setrobotmove(bool value);
string getlastnavigationresult();
void setlastnavigationresult(string value);
void setsystemstate(int value,bool forced);
void setlogicstate(int value,bool forced);
int getsystemstate();
int getlogicstate();
bool getstatemutex();
void setstatemutex(bool value);
void test_vis();
void init();
std::mutex tag_mutex;
//=======================================================================================================
int tempLogicState;
int tempSystemState;
bool statemutex;
std::string coutcolor0 ;
std::string coutcolor_red ;
std::string coutcolor_green;
std::string coutcolor_blue;
std::string coutcolor_magenta ;
std::string coutcolor_brown;
bool say_enable;
bool App_exit;
bool IsCmValid;
bool IsGoalReached;
bool IsRecoveryState;
bool IsHectorReset;
bool isttsready;
string sayMessageId;
ros::ServiceClient client_makeplan;
ros::ServiceClient client_resetcostmap;
ros::ServiceClient client_map_save;
ros::ServiceClient client_map_load;
ros::ServiceClient say_service;
double maxLinSpeedX;
double maxLinSpeedY;
double maxTethaSpeed;
ros::Publisher pub_slam_origin;
ros::Publisher pub_slam_reset;
ros::Publisher pub_alarm;
nav_msgs::Path globalPath;
int globalPathSize;
nav_msgs::Path tempPath;
int temp_path_size;
nav_msgs::OccupancyGrid costmap;
ros::Publisher mycmd_vel_pub;
ros::Publisher pub_tts;
ros::Publisher pub_current_goal;
ros::Publisher pub_move;
double xSpeed;
double ySpeed;
double tethaSpeed;
double desireErrorX;
double desireErrorY;
double desireErrorTetha;
double errorX;
double errorY;
double errorTetha;
double errorX_R;
double errorY_R;
double LKpX;
double LKpY;
double WKp;
double LKiX;
double LKiY;
double WKi;
int step;
double position[2];
double hectorPosition[2];
double tempPosition[2];
double orientation[4];
double amclPosition[2];
double amclOrientation[4];
boost::array<double, 36ul> amclCovariance;
double tetha;
double amclTetha;
double hectorTetha;
double tempTetha;
double tempGoalPos[2] ;
double tempGoalTetha;
double goalPos[2];
double goalOri[4];
double goalTetha ;
geometry_msgs::PoseStamped  target_goal_stamped;
goal_data target_goal_data;
float distacne_to_goal;
double maxErrorX;
double maxErrorY;
double maxErrorTetha;
std::vector<goal_data> goal_list;
std::vector<goal_data> tag_list;
int info_counter;
ros::Time old_time;
int system_state;
int logic_state;
bool on_the_goal;
int step_size;
bool wait_flag;
bool idle_flag;
string last_navigation_result;
bool isrobotmove;
float f;
bool inited = false;
ros::Publisher marker_pub;
ros::Publisher marker_pub2;
ros::Publisher marker_pub3;
ros::Publisher marker_pub4;
ros::Publisher pub_voice_cmd;
ros::NodeHandle node_handle;
ros::Subscriber sub_handles[5];
ros::Subscriber sub_map_points;
ros::Subscriber sub_map_tags;
ros::NodeHandle n_service;
boost::thread _thread_PathFwr;
boost::thread _thread_Logic;
boost::thread _thread_Vis;
boost::thread _thread_Localization;

double _normal_max_linear_speedX;
double _normal_max_angular_speed;
double _normal_kp_linearX;
double _norma_kp_angular;

double _goal_max_linear_speedX;
double _goal_max_angular_speed;
double _goal_kp_linearX;
double _goal_kp_angular;

};

#endif
