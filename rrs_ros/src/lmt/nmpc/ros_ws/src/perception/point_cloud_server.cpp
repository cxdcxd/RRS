#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <sensor_msgs/PointCloud2.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>

#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <perception_msgs/Cylinders.h>

// #include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>



class cloudServer
{
private:
  ros::NodeHandle nh_;
  // publishers and subscribers
  // subscribing raw point cloud from arbitrary sensors
  ros::Subscriber raw_cloud_sub;
  // subscribing  cylinders positions from the nmpc controller
  ros::Subscriber left_cylinders_sub, right_cylinders_sub;
  // after pre processing, publishing pre processed point cloud to octomap server
  ros::Publisher filtered_cloud_pub;
  // subsribing generated octomap from octomap server
  ros::Subscriber octomap_sub;
  // publishing octomap msg to mpc controller node
  ros::Publisher octomap_pub;


  // flags
  bool right_cylinders_ready, left_cylinders_ready, cloud_processed, octomap_received;

  // local cylinder positions
   Eigen::Isometry3d world2camera, camera2world;
   Eigen::Matrix3d world2cameraLinear;
//  Eigen::Isometry3d left_base2camera, camera2base, right_base2camera;
//  Eigen::Matrix3d  left_base2cameraLinear, right_base2cameraLinear;
  std::vector<Eigen::Vector3d> left_centers_camera, left_z_axes_camera, right_centers_camera, right_z_axes_camera;

  // tf listnener for camera to world;
  tf2_ros::Buffer tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;




  // local octree
  std::shared_ptr<octomap::OcTree> tree_ptr;

public:
  cloudServer(ros::NodeHandle nh):nh_(nh)
  {
     raw_cloud_sub = nh.subscribe("/camera/depth/points", 1, &cloudServer::raw_cloud_sub_cb,this);
//    raw_cloud_sub = nh.subscribe("points2", 1, &cloudServer::raw_cloud_sub_cb,this);
    left_cylinders_sub = nh.subscribe("left/nmpc_controller/out/cylinders_ps",1,&cloudServer::left_cylinders_sub_cb,this);
//    right_cylinders_sub = nh.subscribe("right/nmpc_controller/out/cylinders_ps",1,&cloudServer::right_cylinders_sub_cb,this);
    filtered_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_server/out/filtered_cloud",1);
    octomap_pub = nh.advertise<octomap_msgs::Octomap>("cloud_server/out/octree",1);
    right_cylinders_ready = false;
    left_cylinders_ready = false;
    cloud_processed = false;
    octomap_received = false;
    // tfs for kinova ros simulation
     world2camera.translation() << -0.106243,0.340641,1.49118;
     world2camera.linear() << 0.99827,-0.0519514,0.0275271,
                             -0.0153,-0.681616,-0.73155,
                             0.056768,0.729863,-0.681232;
     world2cameraLinear = world2camera.linear();

     camera2world.translation() << 0.0266, -0.8617, 1.2680;
     camera2world.linear() << 0.9983, -0.0153, 0.0568,
                              -0.0520, -0.6816, 0.7299,
                              0.0275, -0.7315, -0.6812;

     left_centers_camera.resize(4);
     left_z_axes_camera.resize(4);
//     right_centers_camera.resize(4);
//     right_z_axes_camera.resize(4);
     left_centers_camera[3] = world2camera*Eigen::Vector3d(0,0,0.137);
     left_z_axes_camera[3] = world2camera.linear()*Eigen::Vector3d(0,0,1);

    // tfs for aruze kinect in real movo robot
    //    tfListener =  std::make_shared<tf2_ros::TransformListener>(tfBuffer);
    //    geometry_msgs::TransformStamped transformStamped;
    //    ros::Duration(1).sleep();
    //    try{
    //    transformStamped = tfBuffer.lookupTransform("depth_link", "left_mpc_base_link",
    //                            ros::Time(0));
    //    }
    //    catch (tf2::TransformException &ex) {
    //    ROS_WARN("%s",ex.what());
    //    return;
    //    }
    //    auto& tf_translation = transformStamped.transform.translation;
    //    Eigen::Translation3d translation(tf_translation.x,tf_translation.y,tf_translation.z);
    //    auto& tf_quat = transformStamped.transform.rotation;
    //    Eigen::Quaterniond quat(tf_quat.w,tf_quat.x,tf_quat.y,tf_quat.z);
    //    left_base2camera = translation*quat;
    //    left_base2cameraLinear = left_base2camera.linear();

    //    try{
    //    transformStamped = tfBuffer.lookupTransform("base_link", "depth_link",
    //                            ros::Time(0));
    //    }
    //    catch (tf2::TransformException &ex) {
    //    ROS_WARN("%s",ex.what());
    //    return;
    //    }
    //    tf_translation = transformStamped.transform.translation;
    //    translation.x() = tf_translation.x; translation.y() = tf_translation.y; translation.z() = tf_translation.z;
    //    quat.w() = tf_quat.w; quat.x() = tf_quat.x; quat.y() = tf_quat.y; quat.z() = tf_quat.z;
    //    camera2base = translation*quat;

    //    try{
    //    transformStamped = tfBuffer.lookupTransform("depth_link", "right_mpc_base_link",
    //                            ros::Time(0));
    //    }
    //    catch (tf2::TransformException &ex) {
    //    ROS_WARN("%s",ex.what());
    //    return;
    //    }
    //    tf_translation = transformStamped.transform.translation;
    //    translation.x() = tf_translation.x; translation.y() = tf_translation.y; translation.z() = tf_translation.z;
    //    quat.w() = tf_quat.w; quat.x() = tf_quat.x; quat.y() = tf_quat.y; quat.z() = tf_quat.z;
    //    right_base2camera = translation*quat;
    //    right_base2cameraLinear = right_base2camera.linear();


    //    left_centers_camera[3] = left_base2camera*Eigen::Vector3d(0,0,0.137);
    //    left_z_axes_camera[3] = left_base2cameraLinear*Eigen::Vector3d(0,0,1);
    //    right_centers_camera[3] = right_base2camera*Eigen::Vector3d(0,0,0.137);
    //    right_z_axes_camera[3] = right_base2cameraLinear*Eigen::Vector3d(0,0,1);

    ROS_INFO("cloud server initialization completed");
  }

private:
  bool isInCylinder(const Eigen::Vector3d& test_point,
                               const Eigen::Vector3d& centroid,
                               const Eigen::Vector3d& z_axis,
                               const double height,
                               const double radius)
  {
    Eigen::Vector3d d = test_point-centroid;
    double z_length = std::abs(d.dot(z_axis));
    if(z_length>height)
    {
      return false;
    }
    else
    {
      double vertical_length_sq = d.dot(d)-z_length*z_length;
      if(vertical_length_sq>radius*radius)
      {
        return false;
      }
      else
      {
        return true;
      }
    }
  }

  void left_cylinders_sub_cb(const perception_msgs::CylindersConstPtr cylinders_msg)
  {
    for(int i=0;i<3;i++)
    {
      Eigen::Vector3d temp;
      const auto& center = cylinders_msg->centers[i];
      temp << center.x,center.y,center.z;
       left_centers_camera[i]=world2camera*temp;
//      left_centers_camera[i]=left_base2camera*temp;

      const auto& z_axis = cylinders_msg->z_axes[i];
      temp << z_axis.x,z_axis.y,z_axis.z;
       left_z_axes_camera[i]=world2cameraLinear*temp;
//      left_z_axes_camera[i]=left_base2cameraLinear*temp;
    }
    left_cylinders_ready=true;

    return;
  }
//  void right_cylinders_sub_cb(const perception_msgs::CylindersConstPtr cylinders_msg)
//  {
//    for(int i=0;i<3;i++)
//    {
//      Eigen::Vector3d temp;
//      const auto& center = cylinders_msg->centers[i];
//      temp << center.x,center.y,center.z;
//      // centers_camera[i]=world2camera*temp;
//      right_centers_camera[i]=right_base2camera*temp;

//      const auto& z_axis = cylinders_msg->z_axes[i];
//      temp << z_axis.x,z_axis.y,z_axis.z;
//      // z_axes_camera[i]=world2cameraLinear*temp;
//      right_z_axes_camera[i]=right_base2cameraLinear*temp;
//    }
//    right_cylinders_ready=true;
//    return;
//  }

  void raw_cloud_sub_cb(const sensor_msgs::PointCloud2ConstPtr pc2_msg)
  {
    if(left_cylinders_ready&&/*right_cylinders_ready&&*/!cloud_processed)
    {

      // remove NAN and down sample
      ROS_INFO("Removing NaN point...");
      pcl::PointCloud< pcl::PointXYZ > pc,pc_valid,pc_filtered;
      pcl::fromROSMsg(*pc2_msg,pc);
      std::vector<int> index;
      pcl::removeNaNFromPointCloud(pc,pc_valid,index);
      ROS_INFO("Down Sampling...");
      std::cout << "before down sampling point cloud has " << pc_valid.size() <<" points" << std::endl;
      pcl::VoxelGrid<pcl::PointXYZ> filter;
      filter.setInputCloud(pc_valid.makeShared());
      filter.setLeafSize(0.05f, 0.05f, 0.05f);
      filter.filter(pc_filtered);
      std::cout << "after down sampling point cloud has " << pc_filtered.size() <<" points" << std::endl;


      //TODO segment out ground

      // filter out point on  and in the robot cylinders
      for(auto it=pc_filtered.begin();
          it!=pc_filtered.end();)
      {
        Eigen::Vector3d curr_point_eigen(it->x,it->y,it->z);
         Eigen::Vector3d point2world = camera2world*curr_point_eigen;
//        Eigen::Vector3d point2world = camera2base*curr_point_eigen;
        if(/*isInCylinder(curr_point_eigen,right_centers_camera[0],right_z_axes_camera[0],0.21,0.15)||
           isInCylinder(curr_point_eigen,right_centers_camera[1],right_z_axes_camera[1],0.3,0.15)||
           isInCylinder(curr_point_eigen,right_centers_camera[2],right_z_axes_camera[2],0.3,0.15)||*/
           isInCylinder(curr_point_eigen,left_centers_camera[0],left_z_axes_camera[0],0.17,0.07)||
           isInCylinder(curr_point_eigen,left_centers_camera[1],left_z_axes_camera[1],0.17,0.07)||
           isInCylinder(curr_point_eigen,left_centers_camera[2],left_z_axes_camera[2],0.27,0.07)||
            isInCylinder(curr_point_eigen,left_centers_camera[3],left_z_axes_camera[3],0.17,0.07)||
           point2world(0)>2||
           point2world(1)<-2||point2world(0)>2||
           point2world(2)<0.06||point2world(2)>3

           )
          pc_filtered.erase(it);
        else
          it++;
      }
      std::cout << "after processing point cloud has " << pc_filtered.size() <<" points" << std::endl;
      sensor_msgs::PointCloud2 post_pc2_msg;
      pcl::toROSMsg(pc_filtered,post_pc2_msg);
      filtered_cloud_pub.publish(post_pc2_msg);
      cloud_processed = true;
      right_cylinders_ready=false;
      left_cylinders_ready=false;
      ROS_INFO("Processed point cloud published.");
      ros::shutdown();
      return;
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_server_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(5);
  cloudServer cloud_server(n);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::waitForShutdown();
}
