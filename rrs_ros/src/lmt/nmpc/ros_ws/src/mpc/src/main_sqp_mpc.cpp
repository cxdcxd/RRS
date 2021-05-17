#include <nmpc_sqp.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc");
  ros::NodeHandle n;
  ros::Rate loop_rate(5);

  NmpcSqp nlmpc_controller(n);

  // define an arbitary obstacle
  shapes::ShapeConstPtr obj_shape(new shapes::Box(0.5,0.1,0.5));
  Eigen::Quaterniond quater;
  quater.w()=1.0;
  quater.x()=0.0;
  quater.y()=0.0;
  quater.z()=0.0;

  Eigen::Translation3d translation(-0.1,0.5,0.5);
  Eigen::Affine3d obj_pose = translation*quater.toRotationMatrix();
//   add the obstacle into controller
  nlmpc_controller.addObstacle("obstacle1",obj_shape,obj_pose);

  while(ros::ok())
  {
    nlmpc_controller.controlLoop();
    loop_rate.sleep();
    ros::spinOnce();
  }
  ros::waitForShutdown();



}
