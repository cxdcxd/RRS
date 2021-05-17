// inhereted class
#include <moveitTool.h>
// ros related
#include <std_srvs/Empty.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
// ros angle and quaternion
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
// eigen library
#include <Eigen/Dense>
// nlopt library
#include <nlopt.hpp>
// hessian calculator
#include <hessianCal.h>
// multi threading
#include <thread>
#include <mutex>
// atomic
#include <atomic>
// for storing data
#include <fstream>
// perception msgs
#include <perception_msgs/Cylinders.h>
// octomap msgs
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

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
  void testJacobian();
  void checkGrad();
  void checkHessian();

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
  ros::Subscriber joint_state_sub;
  // flag that indicates whether controller starts to collect current joint states
  bool state_sub_started;
  bool goal_sub_started;

  // subsriber for goal from interactive marker in rviz
  void goalSubCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg);
  ros::Subscriber goal_sub;
  // store received goal position and goal quaternion
  Vector7d goal_position;
  Vector6d goal1;


  // time interval
  ros::Duration interval;




  // this is for using a trajectory controller
  // publisher for sending desired joint goal and allowed time
  ros::Publisher  goal_pub;
  // constainer storing goal_msg to send
  trajectory_msgs::JointTrajectory goal_msg;

  // server for toggle tracking mode
  bool track_mode;
  ros::ServiceServer track_toggle_server;
  bool track_toggle_cb(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);


  // A simple control scheme example
  void simpleControlScheme();
  // pid parameter for this controller
  double p,i,d;

  // nlmpc optimizer, lower and upper bounds on u
  nlopt::opt optimizer;
  nlopt::opt simple_optimizer;
  std::vector<double> lb;
  std::vector<double> ub;

  // optimize function
  void optimize(std::vector<double> &u, double mincost);
  // mpc related params
  int ph;  // predict horizon, 5 in matlab
  int ch; // control horizon, 3 in matlab
  std::vector<double> u;
  double mincost;
  bool new_goal;
  // optimization related params
  Matrix6d Q,Pf;
  Matrix7d R;

  // system state function
  static void mpcRobotModel(Vector7d& x1, const Vector7d& x, const Vector7d& u,const double Ts);
  static void mpcRobotModelJaco(Matrix7d& A, Matrix7d& B, const Vector7d& x, const Vector7d& u,const double Ts);
  // cost function
  static double costFunc(unsigned n, const double *u, double *grad, void *data);
  static double costFuncMultiThread(unsigned n, const double *u, double *grad, void *data);
  static double constantCostFunc(unsigned n, const double *u, double *grad, void *data);
  // constraint function 
  static void obstacleConstraints(unsigned m, double* result,unsigned n,const double* x, double* grad, void* data);
  static void obstacleConstraintsMultiThread(unsigned m, double* result,unsigned n,const double* x, double* grad, void* data);
  // hessian calculator
  static HessianCal jaco_hess_cal;


  // thread for sending joint velocities to robot
  std::vector<double> joint_velocities;
  std::vector<double> position_goal;
  bool exitFlag;

  std::thread thread1;
  static void velocitiesSend_thread(ros::Publisher publisher, bool* exitFlag,
                                    std::vector<double>* v_ptr, trajectory_msgs::JointTrajectory* msg,
                                    pthread_mutex_t* joint_velocities_mutex, pthread_mutex_t* position_goal_mutex,
                                    bool *new_goal_got,  std::vector<double>* position_goal_ptr);
  pthread_mutex_t joint_velocities_mutex, position_goal_mutex;
  bool new_goal_got;


  // thread for calculating cost values
  std::thread* threads_ptr;
  std::thread* cost_threads_ptr;
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

  // data file handle
  bool init_prediction;
  std::ofstream fout1, fout2, fout3;
  unsigned int data_number_counter;

  // cylinder publisher
  ros::Publisher cylinders_pos_pub;
  // inline function for preparation of publihshing cylinders
  bool cylinders_sent;
  inline void publish_cylinders();
  // octomap subscriber
  ros::Subscriber octomap_sub;
  void octomap_sub_cb(const octomap_msgs::OctomapConstPtr octo_msg);
  shapes::ShapeConstPtr octree_shape;


  // visualization for figures
  // actual trajectory visualizer
  visualization_msgs::Marker actual_traj_strip;
  // goal tf publisher
  tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  // simulate obstacle motion
  double obstacle1_y_v, obstacle1_y_p;


public:
  double Ts;
  int iter;

  // nmpc performance data
  Vector7d total_joint_velocities;
  double average_time, total_time;
  double average_norm_length, straight_length, total_traj_length;
  Eigen::Vector3d prev_point_x_position;
  Eigen::Vector4d curr_point_quat;
  int iter_num;
  double total_diff_position, total_diff_orientation;

  // jacobia check
  void checkJacobian(double x, double y, double z);

  // time measurement
  int cal_cost_count;
  double time_avg;
  std::vector<std::chrono::microseconds> time_consumed;
  void testTime();

};
