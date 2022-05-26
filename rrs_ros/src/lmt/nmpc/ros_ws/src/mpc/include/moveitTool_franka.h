/*
 * moveitTool.h
 *
 *  Created on: July 12, 2020
 *  Modified on: July 12, 2020
 *      Author: Siqi Hu
 *  A tool that provides functionalities:
 *      1. get transform
 *      2. get jacobian
 *      3. get distances between bodies and obstacles
 *      4. some visulizations
 * 
 *
 */

#ifndef MOVEIT_TOOL_H
#define MOVEIT_TOOL_H
#include <ros/ros.h>

// moveit for fk and jacobian calculation (slow!!!)
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// use kdl instead of moveit
// #include <kdl_parser/kdl_parse.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>


// fcl0.6
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>

#include <fcl/common/types.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/geometry/octree/octree.h>

// moveit visual tools
//#include <moveit_visual_tools/moveit_visual_tools.h>

// eigen conversion
#include <eigen_conversions/eigen_msg.h>

// shapes operations
#include <geometric_shapes/shape_operations.h>


/// @brief Distance data stores the distance request and the result given by distance algorithm.
struct DistanceData
{
  /// @brief Distance request
  fcl::DistanceRequestd request;

  /// @brief Distance result
  std::map<fcl::CollisionObjectd*,fcl::DistanceResultd> result;

  /// @brief Whether the distance iteration can stop
  bool done;

};




class MoveitTool
{
public:

  MoveitTool(ros::NodeHandle n);
  ~MoveitTool();
  // return current end effector transform
  const Eigen::Isometry3d& getEEFTransform() const;


  //  Operations on obstacle object in collision computation and obstacle object in rviz visualzation
  void addObstacle(const std::string& obj_name,const shapes::ShapeConstPtr& obj_shape, const Eigen::Affine3d& obj_transform, const bool plot=false);
  void updateObstacle(const std::string& obj_name, const Eigen::Affine3d& obj_transform, const bool plot=false); 
  void removeObstacle(const std::string& obj_name);
  
  int getObstacleNum();

//protected:
  // calculate distances and nearest points between robot links and obstacles
public: // 24.11.2020, make it public for multi threading
  void getDistsAndPoints(std::vector< std::vector<double> >& dists,
                         std::vector<std::vector<Eigen::Vector3d> > &points1,
                         std::vector<std::vector<Eigen::Vector3d> > &points2);

  // distance and points between first and third cylinder
  double selfCollisionDistAndPoints(Eigen::Vector3d& point1, Eigen::Vector3d& point2);


  // update current robot links cartesion poses given current joint values
  // update_co is a flag indicating whether update robot link collision object along with joint values
  // or just update joint values

  // 11.11.2020, make it public for testing
  void updateJointState(const std::vector<double>& joint_values,bool update_co=false,bool update_cylinder_pose=false);

  // 14.12.2020. make it public for calculating eef position and jacobian in multi threading, with safe quaternion coversion considered
   const Eigen::MatrixXd getJacobian(const bool use_quat = true) const; 
protected:
  // get jacobian for current joint state
  void getjacobian(const int link_index, Eigen::MatrixXd& jacobian,
                   const Eigen::Vector3d& ref_point_pos = Eigen::Vector3d(0.0, 0.0, 0.0)
                   ) const;
  ros::NodeHandle nh;
  std::vector<std::string> joint_names;
  int joint_num;

private:
  // robot model
  robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelConstPtr robot_model;
  std::string model_frame;
  std::string global_frame;

  // kdl configuration
  // string robot_description;
  // string chain_root, chain_tip;
  // KDL::Chain my_chain;

  // end effector link
  const robot_model::LinkModel* eef_link;

  // all links
  std::vector<const robot_model::LinkModel*> link_model_ptrs;
  // joint model group for arm
  const robot_state::JointModelGroup* arm_jmg;


  // current robot states
  // current state  2020,11,10: make it protected for debugging
  robot_state::RobotStatePtr current_state;

  // collision
  //container for robot
  std::vector<fcl::CollisionObjectd*> crobot_ptrs;

  // shapes for robot collision
  shapes::ShapeConstPtr link_shape76;
  shapes::ShapeConstPtr link_shape54;
  shapes::ShapeConstPtr link_shape32;
  shapes::ShapeConstPtr link_shape;

  //containers for obstacle, because we need to add, delete, move certain obstacle, we need to know the name of the operated obstacle;
  // derived class will searchin co_ptrs to check if the detected obstacle already stored in system
protected:
  std::map<std::string,std::shared_ptr<fcl::CollisionObjectd>> co_ptrs;
  // store robot cylinders' centroids position and z-axis for current joint state
  std::vector<Eigen::Vector3d> centroids,x_axes,y_axes,z_axes;
private:
  std::map<std::string,shapes::ShapeConstPtr> shape_ptrs;
  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> manager;

  // defined distanceData that will be used in manger distance function
  DistanceData distance_data;
  fcl::DistanceRequestd dist_request;
//  fcl::DistanceResultd dist_result;
  fcl::CollisionRequestd coll_request;
  fcl::CollisionResultd coll_result;
  // static distance call back that will be used in manager distance function
  static bool defaultDistanceFunction(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* cdata_, double& dist);

  // helper function-- build geometry for fcl collision object and rviz message
  void buildGeometry(const shapes::ShapeConstPtr& obj_shape,std::shared_ptr<fcl::CollisionGeometryd> &co);

public:
  // for visulization
  //moveit_visual_tools::MoveItVisualTools visual_tools;
  ros::Publisher marker_publisher;
  // distance to obstacle
  visualization_msgs::Marker line_strip;
  geometry_msgs::Point p;
  // obstacle
  visualization_msgs::Marker obstacle_marker;

};
#endif
