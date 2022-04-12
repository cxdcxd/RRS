
#include <ros/ros.h>
#include <joint_control/joint_control_ros.hh>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "joint_control");

#ifdef ROBOLAND_DEBUG
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
  {
    ros::console::notifyLoggerLevelsChanged();
  }
#endif

  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle pnh = ros::NodeHandle("~");

  boost::shared_ptr<roboland::JointControlRos> en(new roboland::JointControlRos(nh,pnh,argc,argv));

  ros::Rate loop(100);

  while(ros::ok())
  {
    ros::spinOnce();
    loop.sleep();
  }

  return 0;
}
