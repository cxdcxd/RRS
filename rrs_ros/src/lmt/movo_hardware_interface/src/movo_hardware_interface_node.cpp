#include <movo_hardware_interface/movo_hardware_interface.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "movo_hardware_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::MultiThreadedSpinner spinner(2);
  //spinner.start();

  movo_hardware_interface::movoHardwareInterface movo(nh);

  spinner.spin();

  return 0;
}
