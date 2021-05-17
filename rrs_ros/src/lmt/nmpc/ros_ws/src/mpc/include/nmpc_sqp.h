// inhereted class
#include <moveitTool.h>

// ros related
#include <std_srvs/Empty.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

//tf related
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

// ros angle interface
#include <angles/angles.h>

// eigen library
#include <Eigen/Dense>

// nlopt library
#include <nlopt.hpp>

// OSQP- eigen librry
#include <OsqpEigen/OsqpEigen.h>


// hessian calculator
#include <hessianCal.h>

// multi threading
#include <thread>

// atomic
#include <atomic>

typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,7,1> Vector7d;
typedef Eigen::Matrix<double,6,6> Matrix6d;
typedef Eigen::Matrix<double,7,7> Matrix7d;

class NmpcSqp: public MoveitTool
{
public:
  // construnctor and deconstrunctor
  NmpcSqp(ros::NodeHandle nh);
  ~NmpcSqp();

  void controlLoop();
//  void testJacobian();
//  void checkGrad();
//  void checkHessian();

  // redefine addObstacle/removeObstacle/updateObstacle functions, so that obstacle related constraints are
  // incorperated into/removed from/modified in optimizer.
  void addObstacle(const std::string& obj_name,const shapes::ShapeConstPtr& obj_shape, const Eigen::Affine3d& obj_transform);
  //  void removeObstacle(const std::string& obj_name);
  //  void updataObstacle(const std::string& obj_name, const Eigen::Affine3d& obj_transform);

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

  // time interval
  ros::Duration interval;




  // this is for using a trajectory controller
  // publisher for sending desired joint goal and allowed time
  ros::Publisher  goal_pub;

  ros::Publisher vis_goal_pub;
  // helper function for sending given goal joint position/velocity/acceleration and allowed time
  // overload function for only sending position and for only sendinf position and velocity
  void sendCommand(std::vector<double>& joint_position,
                   ros::Duration  time);
  void sendCommand(std::vector<double>& joint_position,
                   std::vector<double>& joint_velocity,
                   ros::Duration  time);
  void sendCommand(std::vector<double>& joint_position,
                   std::vector<double>& joint_velocity,
                   std::vector<double>& joint_acceleration,
                   ros::Duration  time);
  // constainer storing goal_msg to send
  trajectory_msgs::JointTrajectory goal_msg;
  // server for toggle tracking mode
  bool track_mode;
  ros::ServiceServer track_toggle_server;
  bool track_toggle_cb(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);


  // mpc related params
  int ph;  // predict horizon, 5 in matlab
  int ch; // control horizon, 3 in matlab
//  std::vector<double> u;
  Eigen::VectorXd u;



  // n stores the dimentionality of input, should be constant during one process
  // m stores the nubmber of mpc constraints, will be changed when number of observed obstacles changes
  size_t n,m;

  double mincost;
  bool new_goal;
  // optimization related params
  Matrix7d Q,Pf;
  Matrix7d R;
  // jaco and position calculator
  HessianCal jaco_pos_cal;


  // scp algorithm
  void sqpOptimizer();
//  void costValGradHessCntValGrad(const std::vector<double> u, double& cost_val, Eigen::VectorXd& cost_grad, Eigen::MatrixXd& cost_hess, Eigen::VectorXd& constraint_vals, Eigen::MatrixXd& constraint_grads);
  double costValGradCntValGrad(const Eigen::VectorXd& u, Eigen::VectorXd* cost_grad, Eigen::VectorXd* constraint_vals, Eigen::MatrixXd* constraint_grads);
  double getCost(const std::vector<double> u);
  // scp parameters
  double improve_ratio_threshold;
  double min_trust_box_size;
  double min_approx_improve;
  double min_approx_improve_frac;
  int max_iter;
  double trust_shrink_ratio;
  double trust_expand_ratio;
  double cnt_tolerance;
  int max_penalty_coe_inc_num;
  double penalty_coe_increase_ratio;
  double penalty_coe;
  double trust_box_size;
  bool  QP_solver_verbosity;

  // qp solver
  OsqpEigen::Solver solver;
  Eigen::VectorXd improve_direction;
  double cost_val;
  Eigen::VectorXd cost_grad;
  Eigen::VectorXd constraint_vals;
  Eigen::MatrixXd constraint_grads;

  Eigen::VectorXd lb_onVar, ub_onVar, lb_total,ub_total, qp_gradient;
  Eigen::SparseMatrix<double> qp_hessian, qp_linearMatrix;

  // -inf lower bound for constraint values
  Eigen::VectorXd Inf_Vec,Zero_Vec,aux_var_grad;
  bool sub_qp_converged;

  void initSolver();
  double Inf;


  // used for multi threads
  // threads and parameters
  std::vector<std::thread> threads;
  int num_threads, num_loops, last_num_loops;
  // thread flags
  bool calculate_grad;
  std::atomic<bool> activate;
  std::atomic<bool>* deactivates_ptr;
  // output swap contrainers
  std::vector<double> partial_cost_vals;
  std::vector<Eigen::VectorXd> partial_cost_grads;
  std::vector<Eigen::VectorXd> partial_cnt_vals;
  std::vector<Eigen::MatrixXd> partial_cnt_grads;

  // joint values in all time steps
  std::vector<Vector7d> x_total;

  Eigen::VectorXd u_temp;
  static void costCntCalThread(int thread_index, NmpcSqp* class_ptr, const Eigen::VectorXd* u_ptr);


  // moveittools for each thread
  std::vector< std::shared_ptr<MoveitTool> > MTs;
  // for debugging
  int print_count;


  // flag indicating exiting
  bool exitFlag;


  // parameter tunning
public:
  double Ts;
  void testMain();

};
