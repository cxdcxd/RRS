// inhereted class
#include <moveitTool.h>

// ros related
#include <std_srvs/Empty.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

//tf related
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
 #include <eigen_conversions/eigen_msg.h>

// ros angle interface
#include <angles/angles.h>

// ros gazebo msgs
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>

// eigen library
#include <Eigen/Dense>

// nlopt library
#include <nlopt.hpp>

// hessian calculator
#include <hessianCal.h>

// kinova real robot interface
#include <kinova_msgs/JointVelocity.h>
#include <kinova_msgs/JointAngles.h>
// perception msgs
#include <perception_msgs/Cylinders.h>

// octomap related
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

// multi threading
#include <thread>

// atomic
#include <atomic>

// msg type used by movo
#include <control_msgs/JointTrajectoryControllerState.h>
// movo msgs
#include <movo_msgs/JacoAngularVelocityCmd7DOF.h>

// convert angles
#include <angles/angles.h>




typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,7,1> Vector7d;
typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,7,7> Matrix7d;

class NmpcNlopt: public MoveitTool
{
public:
  // construnctor and deconstrunctor
  NmpcNlopt(ros::NodeHandle nh);
  ~NmpcNlopt();

  void initialize();
  void controlLoop();

  // redefine addObstacle/removeObstacle/updateObstacle functions, so that obstacle related constraints are
  // incorperated into/removed from/modified in optimizer.
  void addObstacle(const std::string& obj_name,const shapes::ShapeConstPtr& obj_shape, const Eigen::Affine3d& obj_transform, const bool plot=false);
  //  void removeObstacle(const std::string& obj_name);
  void updateObstacle(const std::string& obj_name, const Eigen::Affine3d& obj_transform, const bool plot=false);

private:
  bool plot_obstacle;

  // subsciber for current joint positions
  std::vector<double> curr_joint_values;
  std::vector<double> last_joint_values;
  std::vector<double> curr_joint_vels;
  std::vector<double> error_integral;
  Vector7d temp_joint_vels;
  Vector7d temp_joint_values;


  // name space that indicates this is left or right arm
  string arm_name_space, othter_arm_name_space;

  void jointStateSubCB_sim(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
  void jointStateSubCB_real(const sensor_msgs::JointState::ConstPtr& msg);
  std::vector<double> movo2kinova_offset;
  ros::Subscriber joint_state_sub;


  // flag that indicates whether controller starts to collect current joint states
  bool state_sub_started;
  bool goal_sub_started;
   shapes::ShapeConstPtr obstacleTable_shape;

  // subsriber for goal from interactive marker in rviz
  void goalSubCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg);
  void goalSubCBFromOut(const geometry_msgs::PoseConstPtr& msg);
  void otherCylindersSubCB(const perception_msgs::CylindersConstPtr& msg);
  void obstacleTable();
  // flag that indicates if cylinders from anothter arm are added as obstacles
  bool cylinders_added, cylinders_sent;
  ros::Subscriber goal_sub, other_cylinders_sub;
  // store received goal position and goal quaternion
  Vector7d goal_position;
  // time interval
  ros::Duration interval;
  inline void publish_cylinders();
  void octomap_sub_cb(const octomap_msgs::OctomapConstPtr octo_msg);
  ros::Subscriber octomap_sub;
  shapes::ShapeConstPtr octree_shape;




  // this is for using a trajectory controller
  // publisher for sending desired joint goal and allowed time
  ros::Publisher velocity_pub, eef_pose_pub, self_cylinders_pub;

  // current eef pose msg to be sent
  geometry_msgs::Pose curr_eef_pose;

  // server for toggle tracking mode
  bool track_mode;
  ros::ServiceServer track_toggle_server;
  bool track_toggle_cb(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

  // nlmpc optimizer, lower and upper bounds on u
  nlopt::opt optimizer;
  std::vector<double> lb;
  std::vector<double> ub;

  // optimize function
  void optimize(std::vector<double> &u, double mincost);

  // mpc related params
  int ph;  // predict horizon, 5 in matlab
  int ch; // control horizon, 3 in matlab
  double Ts; // sample time
  std::vector<double> u;
  double mincost;
  bool new_goal;
  // optimization related params
  Matrix6d Q,Pf;
  Matrix7d R;

  // cost function
  static double costFuncMultiThread(unsigned n, const double *u, double *grad, void *data);
  // constraint function 
  static void obstacleConstraintsMultiThread(unsigned m, double* result,unsigned n,const double* x, double* grad, void* data);
  // hessian calculator
  HessianCal jaco_hess_cal;

  // thread for sending joint velocities to robot
  std::vector<double> joint_velocities, position_goal;
  bool exitFlag;

  std::thread thread1;
  static void velocitiesSend_thread_sim(std::vector<double>* c_ptr,ros::Publisher publisher, bool* exitFlag,
                                    std::vector<double>* v_ptr, trajectory_msgs::JointTrajectory* msg,
                                    pthread_mutex_t* joint_velocities_mutex, pthread_mutex_t* position_goal_mutex,
                                    bool *new_goal_got,  std::vector<double>* position_goal_ptr);
  static void velocitiesSend_thread_real(ros::Publisher publisher2real, bool* exitFlag,
                                    std::vector<double>* v_ptr,
                                    pthread_mutex_t* joint_velocities_mutex,
                                    bool *new_goal_got);
  pthread_mutex_t joint_velocities_mutex, position_goal_mutex;
  bool new_goal_got;
  trajectory_msgs::JointTrajectory goal_msg;



 // thread for calculating cost and constraint
  std::thread* threads_ptr;
  std::thread* cost_threads_ptr;
  int num_threads;
  int num_loops;
  int last_num_loops;

  int num_cost_threads;
  int num_cost_loops;
  int last_num_cost_loops;

  int num_cnt_threads;
  int num_cnt_loops;
  int last_num_cnt_loops;

  // shared container for inputs of cost cal thread
  double* u_ptr;
  Vector7d* jnt_vals_ptr;
  Vector7d* costCal_jnt_vals_ptr;
  std::atomic_bool cost_activated, activated;
  bool cost_calculate_grad, calculate_grad;

  // output of thread
  double* partial_costs_ptr;
  double** partial_grads_ptr;
  std::atomic_bool* finished_ptr;
  std::atomic_bool* cost_finished_ptr;
  static void costCalThread(const int thread_index, NmpcNlopt* class_ptr, const double *u_ptr);
  // shared in-output constainer for constraint cal thread
  std::vector< std::vector<double> > partial_cnt_val;
  std::vector< std::vector<double> > partial_cnt_grads;
  std::vector<MoveitTool*> MoveitTools_array;
  static void cntCalThread(const int thread_index, NmpcNlopt *class_ptr);



  // tf listnener
  tf2_ros::Buffer tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;
  Eigen::Affine3d other_arm_base2this_arm_base, base2this_arm_base;
  double body_height;


public:
  // performance measuring parameters
  int cal_cost_count;
  double time_avg;

  // broadcost the transform of goal sent from outside device
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped_goal;
};
