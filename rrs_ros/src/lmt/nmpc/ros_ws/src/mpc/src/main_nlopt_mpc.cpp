#include <nmpc_nlopt.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nmpc");
  ros::NodeHandle n;
  ros::Rate loop_rate(5);
  NmpcNlopt nlmpc_controller(n);

//   Single Large box Obstacle
//   shapes::ShapeConstPtr obj_shape(new shapes::Box(0.5,0.1,0.5));
//   //   shapes::ShapeConstPtr obj_shape(new shapes::Sphere(0.2));
//   Eigen::Quaterniond quater;
//   quater.w()=1.0;
//   quater.x()=0.0;
//   quater.y()=0.0;
//   quater.z()=0.0;

//   Eigen::Translation3d translation(-0.1,0.5,0.5);
//   Eigen::Affine3d obj_pose = translation*quater.toRotationMatrix();
//   //    add the obstacle into controller
//   nlmpc_controller.addObstacle("obstacle1",obj_shape,obj_pose);


  // ground
  // shapes::ShapeConstPtr obj_shape(new shapes::Plane(0,0,1,0));
  // Eigen::Quaterniond quater;
  // quater.w()=1.0;
  // quater.x()=0.0;
  // quater.y()=0.0;
  // quater.z()=0.0;

  // Eigen::Translation3d translation(-0.1,0.5,0.5);
  // Eigen::Affine3d obj_pose = translation*quater.toRotationMatrix();
  // //    add the obstacle into controller
  // nlmpc_controller.addObstacle("ground",obj_shape,obj_pose);






   // Dynamic Small Obstacle
  shapes::ShapeConstPtr obj_shape(new shapes::Box(0.2,0.2,0.2));
  Eigen::Quaterniond quater;
  quater.w()=1.0;
  quater.x()=0.0;
  quater.y()=0.0;
  quater.z()=0.0;

  Eigen::Translation3d translation1(-0.3,0.5,0.5);
  Eigen::Affine3d obj_pose = translation1*quater.toRotationMatrix();
 //   add the obstacle into controller
  nlmpc_controller.addObstacle("obstacle1",obj_shape,obj_pose);
 // Eigen::Translation3d translation2(-0.1,0.5,0.5);
 // obj_pose = translation2*quater.toRotationMatrix();
 // nlmpc_controller.addObstacle("obstacle2",obj_shape,obj_pose);


  // build an octomap obstacle
// auto tree = std::make_shared<octomap::OcTree>(0.05);
// for (int x=-50; x<50; x++) {
//   for (int y=-10; y<10; y++) {
//     for (int z=-50;z<50; z++) {
//       octomap::point3d endpoint ((float) x*0.005f, (float) y*0.005f, (float) z*0.005f);
//       tree->updateNode(endpoint, true); // integrate 'occupied' measurement
//     }
//   }
// }


// Eigen::Translation3d translation(-0.1,0.5,0.5);
// Eigen::Quaterniond quater;
// quater.w()=1.0;
// quater.x()=0.0;
// quater.y()=0.0;
// quater.z()=0.0;
// Eigen::Affine3d obj_pose = translation*quater.toRotationMatrix();
// shapes::ShapeConstPtr obj_shape(new shapes::OcTree(tree));
// nlmpc_controller.addObstacle("obstacle1",obj_shape,obj_pose);


  while(ros::ok())
  {
    ros::spinOnce();
    nlmpc_controller.controlLoop();
    loop_rate.sleep();
  }
  ros::waitForShutdown();
}
