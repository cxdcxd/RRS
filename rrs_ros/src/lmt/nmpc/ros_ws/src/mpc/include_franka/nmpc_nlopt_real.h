// inhereted class
#include <moveitTool.h>

// ros related
#include <std_srvs/Empty.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

//tf related
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
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

// franka real robot interface
#include <franka_core_msgs/JointCommand.h>

// multi threading
#include <thread>

// atomic
#include <atomic>

// msg type used by movo
#include <control_msgs/JointTrajectoryControllerState.h>




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

  void controlLoop();

  // redefine addObstacle/removeObstacle/updateObstacle functions, so that obstacle related constraints are
  // incorperated into/removed from/modified in optimizer.
  void addObstacle(const std::string& obj_name,const shapes::ShapeConstPtr& obj_shape, const Eigen::Affine3d& obj_transform);
  //  void removeObstacle(const std::string& obj_name);
  void updateObstacle(const std::string& obj_name, const Eigen::Affine3d& obj_transform);

private:
  // subsciber for current joint positions
  std::vector<double> curr_joint_values;
  std::vector<double> last_joint_values;
  std::vector<double> curr_joint_vels;
  std::vector<double> error_integral;
  Vector7d temp_joint_vels;
  Vector7d temp_joint_values;



  void jointStateSubCB(const sensor_msgs::JointState::ConstPtr& msg);
  void obstacle1_sub_cb(const geometry_msgs::Pose::ConstPtr& msg);
  void obstacle2_sub_cb(const geometry_msgs::Pose::ConstPtr& msg);
  // predefine two obstacles and update their pose from subsriber msg
  shapes::ShapeConstPtr obstacle1_shape;
  shapes::ShapeConstPtr obstacle2_shape;
  // tf from vive system frame to robot arm base frame
  Eigen::Vector3d base2vive_offset;


  ros::Subscriber joint_state_sub, obstacle_sub1, obstacle_sub2;


  // flag that indicates whether controller starts to collect current joint states
  bool state_sub_started;
  bool goal_sub_started;

  // subsriber for goal from interactive marker in rviz
  void goalSubCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg);
  void goalSubFromOutsideCB(const geometry_msgs::PoseConstPtr &msg);
  ros::Subscriber goal_sub, goal_sub_from_outside;
  // store received goal position and goal quaternion
//  double goal_pos[3];
//  double goal_quat[4];
  Vector7d goal_position;
  // time interval
  ros::Duration interval;




  // this is for using a trajectory controller
  // publisher for sending desired joint goal and allowed time
  ros::Publisher velocity_pub, eef_pose_pub;

  // current eef pose msg to be sent
  geometry_msgs::Pose curr_eef_pose;

  // server for toggle tracking mode
  bool track_mode;
  ros::ServiceServer track_toggle_server;
  bool track_toggle_cb(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

  // nlmpc optimizer, lower and upper bounds on u
  nlopt::opt optimizer;
  nlopt::opt global_optimizer;
  nlopt::opt sub_optimizer;
  std::vector<double> lb;
  std::vector<double> ub;

  // optimize function
  void optimize(std::vector<double> &u, double mincost);
  void global_optimize(std::vector<double> &u, double mincost);

  // mpc related params
  int ph;  // predict horizon, 5 in matlab
  int ch; // control horizon, 3 in matlab
  std::vector<double> u;
  double mincost;
  bool new_goal;
  // optimization related params
  Matrix6d Q,Pf;
  Matrix7d R;

  // cost function
  static double costFunc(unsigned n, const double *u, double *grad, void *data);
  static double costFuncMultiThread(unsigned n, const double *u, double *grad, void *data);
  // constraint function 
  static void obstacleConstraints(unsigned m, double* result,unsigned n,const double* x, double* grad, void* data);
  static void obstacleConstraintsMultiThread(unsigned m, double* result,unsigned n,const double* x, double* grad, void* data);
  // hessian calculator
  HessianCal jaco_hess_cal;


  // interface to perception
  // hard coding environment in the simulation, will be replaced by actual perception of
  // the environment
  double box_size;
  // ros::Publisher gazebo_model_pub;
  // gazebo_msgs::ModelState obstacle_state;

  // ros::Subscriber gazebo_model_sub;
  // void gazeboModel_sub_cb(const gazebo_msgs::ModelStatesConstPtr& msg);

  double robot_v;
  double obstacle_v;

  // thread for sending joint velocities to robot
  std::vector<double> joint_velocities;
  bool exitFlag;

  std::thread thread1;
  static void velocitiesSend_thread(ros::Publisher publisher2real, bool* exitFlag,
                                    std::vector<double>* v_ptr,
                                    pthread_mutex_t* joint_velocities_mutex,
                                    bool *new_goal_got);
  pthread_mutex_t joint_velocities_mutex;
  bool new_goal_got;


 // thread for calculating cost and constraint
  std::thread* threads_ptr;
  std::thread* cost_threads_ptr;
  int num_threads;
  int num_loops;
  int last_num_loops;

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

  int cal_cost_count;
  double time_avg;



  // tf between arm base and world frame
  Eigen::Affine3d origin2base;
  tf2_ros::Buffer tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;

  // broadcost the transform of goal sent from outside device
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped_goal;

public:
  double Ts;
  int iter;

  // nmpc performance data
  double average_time;
  double average_norm_length;
  double straight_length;
  int iter_num;


  // tf check
  Eigen::Isometry3d transform_eigen;
  void test_tf_listener();


};
