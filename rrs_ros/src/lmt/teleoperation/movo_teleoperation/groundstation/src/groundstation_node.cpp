
#include <ros/ros.h>
#include <robot_groundstation/groundstation_ros.hh>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "robot_groundstation");

#ifdef ROBOLAND_DEBUG
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
  {
    ros::console::notifyLoggerLevelsChanged();
  }
#endif

  ROS_INFO("LMT GroundStation Interface");

  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle pnh = ros::NodeHandle("~");

  boost::shared_ptr<roboland::GroundstationRos> en(new roboland::GroundstationRos(nh,pnh,argc,argv,"1"));

  ros::Rate loop(100);

  while(ros::ok())
  {
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
