#include <nmpc_sqp.h>
#include <chrono>
NmpcSqp::NmpcSqp(ros::NodeHandle nh):MoveitTool(nh)
{
  // load parameters
  bool param_ok=true;
  double Q1v,Q2v,Pf1v,Pf2v,Rv;
  if(!nh.getParam("sqp_mpc_main_node/nlmpc/cost_weight/Q1v",Q1v))
  {
    ROS_ERROR("error loading  Q1v param");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/nlmpc/cost_weight/Q2v",Q2v))
  {
    ROS_ERROR("error loading  Q2v param");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/nlmpc/cost_weight/Pf1v",Pf1v))
  {
    ROS_ERROR("error loading  Pf1v param");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/nlmpc/cost_weight/Pf2v",Pf2v))
  {
    ROS_ERROR("error loading  Pf2v param");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/nlmpc/cost_weight/Rv",Rv))
  {
    ROS_ERROR("error loading  Rv param");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/nlmpc/ph",ph))
  {
    ROS_ERROR("error loading  prediction horizon param");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/nlmpc/ch",ch))
  {
    ROS_ERROR("error loading  control horizon param");
     param_ok = false;
  }
  // sqp parameters
  if(!nh.getParam("sqp_mpc_main_node/scp/improve_ratio_threshold",improve_ratio_threshold))
  {
    ROS_ERROR("error loading  improve_ratio_threshold");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/scp/min_trust_box_size",min_trust_box_size))
  {
    ROS_ERROR("error loading  min_trust_box_size");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/scp/min_approx_improve",min_approx_improve))
  {
    ROS_ERROR("error loading  min_approx_improve");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/scp/min_approx_improve_frac",min_approx_improve_frac))
  {
    ROS_ERROR("error loading  min_approx_improve_frac");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/scp/max_iter",max_iter))
  {
    ROS_ERROR("error loading  max_iter");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/scp/trust_shrink_ratio",trust_shrink_ratio))
  {
    ROS_ERROR("error loading  trust_shrink_ratio");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/scp/trust_expand_ratio",trust_expand_ratio))
  {
    ROS_ERROR("error loading  trust_expand_ratio");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/scp/cnt_tolerance",cnt_tolerance))
  {
    ROS_ERROR("error loading  cnt_tolerance");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/scp/max_penalty_coe_inc_num",max_penalty_coe_inc_num))
  {
    ROS_ERROR("error loading  max_penalty_coe_inc_num");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/scp/penalty_coe_increase_ratio",penalty_coe_increase_ratio))
  {
    ROS_ERROR("error loading  penalty_coe_increase_ratio");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/scp/penalty_coe",penalty_coe))
  {
    ROS_ERROR("error loading  penalty_coe");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/scp/trust_box_size",trust_box_size))
  {
    ROS_ERROR("error loading  trust_box_size");
     param_ok = false;
  }
  if(!nh.getParam("sqp_mpc_main_node/scp/verbosity",QP_solver_verbosity))
  {
    ROS_ERROR("error loading  QP_solver_verbosity");
     param_ok = false;
  }


  if(!param_ok)
  {
    ros::shutdown();
  }
  Vector7d diag1;
  diag1 << Q1v,Q1v,Q1v,Q2v,Q2v,Q2v,Q2v;
  Q = diag1.asDiagonal();
  diag1 << Pf1v,Pf1v,Pf1v,Pf2v,Pf2v,Pf2v,Pf2v;
  Pf = diag1.asDiagonal();
  Vector7d diag2;
  diag2 << Rv,Rv,Rv,Rv,Rv,Rv,Rv;
  R = diag2.asDiagonal();


  curr_joint_values.resize(joint_num);
  last_joint_values.resize(joint_num);
  curr_joint_vels.resize(joint_num);

  joint_state_sub = nh.subscribe("/j2s7s300/joint_states", 100, &NmpcSqp::jointStateSubCB,this);
  state_sub_started = false;
  goal_sub_started = false;
  goal_sub = nh.subscribe("/simple_marker/feedback",100,&NmpcSqp::goalSubCB,this);

  // this is for using a trajectory controller
  goal_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/j2s7s300/position_joint_trajectory_controller/command",1);

  // feed unchanged content into goal_msg
  goal_msg.joint_names = {"j2s7s300_joint_1","j2s7s300_joint_2","j2s7s300_joint_3","j2s7s300_joint_4",
                         "j2s7s300_joint_5","j2s7s300_joint_6","j2s7s300_joint_7"};

  // server for toggle tracking mode
  track_toggle_server = nh.advertiseService("/j2s7s300/track_toggle",&NmpcSqp::track_toggle_cb,this);
  track_mode = false;

  Ts = 0.2;
  interval = ros::Duration(Ts);

  // m here directly set to constant for testing, in which only one obstacle is considered.
  m = 0;
  n = 7*ch;


  // initialize input u with random values within physical bounds
//  u.resize(n);
//  std::fill(u.begin(),u.end(),0);
  u = Eigen::VectorXd::Zero(n);
//  struct timespec ts;
//  for(int i = 0;i<n;i++)
//  {
//    clock_gettime(CLOCK_MONOTONIC, &ts);
//    srand((time_t)ts.tv_nsec);
//    u[i] = (double)rand()/(double)RAND_MAX;
//    u[i] = u[i]*1.6-0.8;
//  }


  //qp solver initialization
  initSolver();
  // constant variable used for qp constraints
  lb_onVar = -0.8*Eigen::VectorXd::Ones(n);
  ub_onVar = 0.8*Eigen::VectorXd::Ones(n);
  Inf = std::numeric_limits<double>::infinity();

  new_goal=false;

// multi thread init
//  x_total.resize(ph);
//  u_temp = Eigen::VectorXd::Zero(n);

//  num_threads = 3;
//  num_loops = ph/num_threads;
//  last_num_loops = num_loops+ph%num_threads;

//  partial_cost_vals.resize(num_threads);
//  partial_cost_grads.resize(num_threads);
//  partial_cnt_vals.resize(num_threads);
//  partial_cnt_grads.resize(num_threads);

//  activate = false;
//  calculate_grad = false;
//  deactivates_ptr = new std::atomic<bool>[num_threads];

//  threads.resize(num_threads);
//  MTs.resize(num_threads);
//  for(int i=0;i<num_threads;i++)
//  {
//    threads[i] = std::thread(costCntCalThread, i, this, &u_temp);
//    MTs[i] = std::make_shared<MoveitTool>(nh);
//    ros::Duration(0.1).sleep();
//  }

  exitFlag = false;
  print_count = 0;


  // If there is no other node like gazebo subscribe topic msg published by goal_pub, then wait
  ros::WallDuration sleep_t(0.5);
  while(goal_pub.getNumSubscribers() < 1)
    sleep_t.sleep();
  ROS_INFO("Controller Initialization completed.");
}

NmpcSqp::~NmpcSqp()
{
  exitFlag = true;
//  for(int i=0;i<num_threads;i++)
//    threads[i].join();
//  delete []deactivates_ptr;
}

void NmpcSqp::initSolver()
{
  solver.clearSolver();
  solver.settings()->setVerbosity(QP_solver_verbosity);
  solver.settings()->setWarmStart(true);

  qp_hessian.resize(n+m,n+m);
  qp_gradient = Eigen::VectorXd::Zero(n+m);
  qp_linearMatrix.resize(n+2*m,n+m);


  lb_total = Eigen::VectorXd::Zero(n+2*m);
  ub_total = Eigen::VectorXd::Ones(n+2*m);

  solver.data()->setNumberOfVariables(n+m);
  solver.data()->setNumberOfConstraints(n+2*m);
  solver.data()->clearHessianMatrix();
  solver.data()->setHessianMatrix(qp_hessian);
  solver.data()->setGradient(qp_gradient);
  solver.data()->clearLinearConstraintsMatrix();
  solver.data()->setLinearConstraintsMatrix(qp_linearMatrix);
  solver.data()->setLowerBound(lb_total);
  solver.data()->setUpperBound(ub_total);

  if(!solver.initSolver())
  {
    ROS_ERROR("qp solver initialization failed.");
    ros::shutdown();
  }
  ROS_INFO("qp solver Initialization completed.");
  return;
}

void NmpcSqp::jointStateSubCB(const sensor_msgs::JointState::ConstPtr& msg)
{
  if(!state_sub_started)
  {
    state_sub_started = true;
    for(int i=0;i<joint_num;i++)
    {
      curr_joint_values[i] = msg->position[i];
      last_joint_values[i] = curr_joint_values[i];
      curr_joint_vels[i] = 0;
    }

  }
  else
  {
    for(int i=0;i<joint_num;i++)
    {
      last_joint_values[i] = curr_joint_values[i];
      curr_joint_values[i] = msg->position[i];
      curr_joint_vels[i] = (curr_joint_values[i] - last_joint_values[i])/interval.toSec();
    }

  }
}

void NmpcSqp::goalSubCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg)
{
  goal_position << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z,
                   msg->pose.orientation.w,msg->pose.orientation.x,
                   msg->pose.orientation.y,msg->pose.orientation.z;

  if(!goal_sub_started)
    goal_sub_started = true;
  if(!new_goal)
    new_goal=true;
}

//this is for using a trajectory controller
void NmpcSqp::sendCommand(std::vector<double>& joint_position,
                 std::vector<double>& joint_velocity,
                 std::vector<double>& joint_acceleration,
                 ros::Duration time)
{
//  goal_msg.header.seq++;
  goal_msg.points.clear();
  trajectory_msgs::JointTrajectoryPoint goal_msg_point;
  goal_msg_point.time_from_start = time;
  goal_msg_point.positions = joint_position;
  goal_msg_point.velocities = joint_velocity;
  goal_msg_point.accelerations = joint_acceleration;
  goal_msg.points.push_back(goal_msg_point);
  goal_pub.publish(goal_msg);
}
void NmpcSqp::sendCommand(std::vector<double>& joint_position,
                 std::vector<double>& joint_velocity,
                 ros::Duration time)
{
  goal_msg.header.seq++;
  goal_msg.points.clear();
  trajectory_msgs::JointTrajectoryPoint goal_msg_point;
  goal_msg_point.time_from_start = time;
  goal_msg_point.positions = joint_position;
  goal_msg_point.velocities = joint_velocity;
  goal_msg.points.push_back(goal_msg_point);
  goal_pub.publish(goal_msg);
}
void NmpcSqp::sendCommand(std::vector<double>& joint_position,
                 ros::Duration time)
{
  goal_msg.header.seq++;
  goal_msg.points.clear();
  trajectory_msgs::JointTrajectoryPoint goal_msg_point;
  goal_msg_point.time_from_start = time;
  goal_msg_point.positions = joint_position;
  goal_msg.points.push_back(goal_msg_point);
  goal_pub.publish(goal_msg);
}

bool NmpcSqp::track_toggle_cb(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
  track_mode = track_mode?false:true;
  return true;
}



// singularityCheck
//void NmpcSqp::testJacobian()
//{
//  using namespace std;
//  while(!state_sub_started)
//  {
//    ROS_INFO("Fisrt joint states not come in yet, wait until robot knows its current joint state.");
//    ros::Duration(1).sleep();
//  }
//  std::vector<double> curr_values = curr_joint_values;
//  updateJointState(curr_values);

//  auto eef_transform1 = getEEFTransform();

//  Eigen::Quaterniond quat1(eef_transform1.rotation());
//  double w=quat1.w(),x=quat1.x(),y=quat1.y(),z=quat1.z();


//  auto t1 = std::chrono::high_resolution_clock::now();
//  Eigen::Matrix<double,6,7> Ji = getJacobian();
//  auto t2 = std::chrono::high_resolution_clock::now();
//  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
//  std::cout << duration << std::endl;
//  Eigen::Matrix<double,4,3> quaternion_update_matrix;
//  quaternion_update_matrix << -x,-y,-z,w,z,-y,-z,w,x,y,-x,w;
//  Eigen::Matrix<double,7,7> Ji_quat;
//  Ji_quat.topRows<3>() = Ji.topRows<3>();
//  Ji_quat.bottomRows<4>() = 0.5*quaternion_update_matrix*Ji.bottomRows<3>();








//  cout << "joint values: " << endl;
//  for(int i=0;i<7;i++)
//    cout << curr_values[i] << " ";
//  cout << endl;
//  cout << "rotation matrix:" << endl;
//  cout << eef_transform1.linear() << endl;
//  cout << "tranlation:" << endl;
//  cout << eef_transform1.translation() << endl;




//  Eigen::Quaterniond quat1(eef_transform1.rotation());


//  Eigen::Vector3d z_axis = eef_transform1.rotation().rightCols(1);
//  //  tf::Transform eef_tf;
////  tf::transformEigenToTF(eef_transform1,eef_tf);

//  // note that getRPY method outputs roll around x axis, pitch around y axis and yaw around z axis
//  // not roll around z axis, pitch around y axis and yaw around x axis!!!!!!

////  double r1,p1,y1,r2,p2,y2;
////  eef_tf.getBasis().getRPY(r1,p1,y1);

////  Eigen::Matrix3d B,B_inv;
////  B << 0, -sin(y1), cos(y1)*cos(p1),
////       0, cos(y1), sin(y1)*cos(p1),
////       1, 0, -sin(p1);
////  B_inv << (cos(y1)*sin(p1))/cos(p1), (sin(y1)*sin(p1))/cos(p1), 1,
////            -sin(y1), cos(y1), 0,
////            cos(y1)/cos(p1), sin(y1)/cos(p1), 0;

////  Eigen::JacobiSVD<Eigen::Matrix3d> svd(B, Eigen::ComputeFullU | Eigen::ComputeFullV);
////  Eigen::Vector3d singular_v =  svd.singularValues();
////  Eigen::Vector3d singular_v_inv;
////  double pinvtoler = 1e-12;
////  for(int i = 0; i < 3; i++)
////  {
////    if( singular_v(i) > pinvtoler)
////      singular_v_inv(i) = 1.0/singular_v(i);
////    else
////      singular_v_inv(i) = 0;
////  }
////  B_inv = svd.matrixV() * singular_v_inv.asDiagonal() * svd.matrixU().transpose();

////  Eigen::Matrix<double,6,6> IB_inv = Eigen::Matrix<double,6,6>::Identity();
////  IB_inv.topLeftCorner<3,3>() = Eigen::Matrix3d::Identity();
////  IB_inv.bottomRightCorner<3,3>() = B_inv;

////  Eigen::Matrix<double,6,7> Ji = IB_inv*getJacobian();//.cast<float>();


//  // attention: calculation of quaternion based jacobian from moveit is wrong!!!!
//  // use moveit to calculate the original euler angle based jacobian, then manually update it to quaternion based.
//  Eigen::Matrix<double,6,7> Ji = getJacobian();
//  Ji(2,0) = 0;
//  Ji(3,0) = 0;
//  Ji(4,0) = 0;
//  Ji(5,0) = -1;
//  Ji(5,1) = 0;
//  Eigen::Matrix<double,6,6> J = Ji.leftCols(6);
//  for(int i=0;i<6;i++)
//  {
//    Eigen::Matrix3d skew;
//    skew << 0,-J(5,i),J(4,i),J(5,i),0,-J(3,i),-J(4,i),J(3,i),0;
//    J.block<3,1>(3,i) = skew*z_axis;
//  }

//  Eigen::Matrix<double,4,3> quaternion_update_matrix;
//  Eigen::Vector4d quatv = quat1.coeffs();
//  double x = quatv(0),y=quatv(1),z=quatv(2),w=quatv(3);
//  quaternion_update_matrix << -x,-y,-z,w,z,-y,-z,w,x,y,-x,w;
//  Eigen::Matrix<double,7,7> Ji_quat;
//  Ji_quat.topRows(3) = Ji.topRows(3);
//  Ji_quat.bottomRows(4) = 0.5*quaternion_update_matrix*Ji.bottomRows(3);
//  std::cout << "Jacobian analytical: " << std::endl;
//  std::cout << Ji << std::endl;




//  Eigen::Matrix<double,6,6> J_numerical;
//  // numeical jacobian
//  double delta = 1e-6;
//  for(int i=0;i<6;i++)
//  {
//    std::vector<double> values = curr_values;
//    values[i]+=delta;
//    updateJointState(values);
//    auto eef_transform2 = getEEFTransform();

//    Vector6d diff;
//    Eigen::Vector3d diff_position;
//    Eigen::Vector3d diff_orientation;

//    diff_position = eef_transform2.translation() - eef_transform1.translation();
//    Eigen::Vector3d z_axis2 = eef_transform2.rotation().rightCols(1);
//    diff_orientation = z_axis2 - z_axis;
////    Eigen::Quaterniond quat2(eef_transform2.rotation());
////    diff_orientation << quat2.w()-quat1.w(),quat2.x()-quat1.x(),quat2.y()-quat1.y(),quat2.z()-quat1.z();
//    diff<< diff_position,
//           diff_orientation;

//    J_numerical.col(i) = diff/delta;

//  }

//  std::cout << "jacobian numerical: " << std::endl;
//  std::cout << J_numerical<< std::endl;

//}

void NmpcSqp::addObstacle(const std::string &obj_name, const shapes::ShapeConstPtr &obj_shape, const Eigen::Affine3d &obj_transform)
{
  m += 3*ph;
  // dimension of constraints changes, so the osqp solver needs to be re-initialized.
  initSolver();
  MoveitTool::addObstacle(obj_name,obj_shape,obj_transform);
//  for(int i=0;i<num_threads;i++)
//    MTs[i]->addObstacle(obj_name,obj_shape,obj_transform);
}

void NmpcSqp::costCntCalThread(int thread_index, NmpcSqp *class_ptr, const Eigen::VectorXd *u_ptr)
{
  std::cout << "Cost cal thread[" << thread_index+1 << "] started" << std::endl;
  while(ros::ok() && !(class_ptr->exitFlag))
  {
    if(class_ptr->activate && !(class_ptr->deactivates_ptr[thread_index]))
    {
      // parameters
      const int n = class_ptr->n;
      const Matrix7d& Q = class_ptr->Q;
      const Matrix7d& Pf = class_ptr->Pf;
      const Matrix7d& R = class_ptr->R;
      const double Ts = class_ptr->Ts;
      const int ph = class_ptr->ph;
      const int ch = class_ptr->ch;
      bool cal_grad = class_ptr->calculate_grad;
      double safe_distance = 0.02;
      unsigned int num_obstacles = class_ptr->getObstacleNum();
      unsigned int num_links = 3;
      unsigned int num_dists = num_links*num_obstacles;

      Vector7d eef_position, diff_position, u_eigen;
      const Vector7d& goal_position = class_ptr->goal_position;
      double& cost_running = class_ptr->partial_cost_vals[thread_index];
      Eigen::VectorXd& cost_grad = class_ptr->partial_cost_grads[thread_index];
      Eigen::VectorXd& constraint_vals = class_ptr->partial_cnt_vals[thread_index];
      Eigen::MatrixXd& constraint_grads = class_ptr->partial_cnt_grads[thread_index];
      std::shared_ptr<MoveitTool> thread_moveit_tool = class_ptr->MTs[thread_index];

      cost_running = 0;
      cost_grad = Eigen::VectorXd::Zero(n);
      constraint_vals = Eigen::VectorXd::Zero(num_dists*ph);
      constraint_grads = Eigen::MatrixXd::Zero(num_dists*ph,n);

      HessianCal& jaco_pos_calculator = class_ptr->jaco_pos_cal;
      std::vector<double> curr_x(7);

      int local_num_loops = (thread_index==(class_ptr->num_threads-1))?class_ptr->last_num_loops:class_ptr->num_loops;
      for(int i=0;i<local_num_loops;i++)
      {
        int global_i = i + thread_index*class_ptr->num_loops;
        for(int j =0;j<7;j++)
          curr_x[j] = class_ptr->x_total[global_i][j];
        // cost value
        jaco_pos_calculator.getPosition(curr_x, eef_position);
        diff_position = goal_position - eef_position;
        Eigen::Matrix<double,1,7> diffTQPf;
        Matrix7d QPf;
        if(global_i==ph-1)
        {
          diffTQPf = diff_position.transpose()*Pf;
          QPf = Pf;
        }
        else
        {
          diffTQPf = diff_position.transpose()*Q;
          QPf = Q;
        }
        double cost_running_temp = diffTQPf*diff_position;
        int u_index = std::min(global_i,ch-1);
        u_eigen = u_ptr->segment<7>(7*u_index);
        Eigen::Matrix<double,1,7> uTR = u_eigen.transpose()*R;
        double temp = uTR*u_eigen;
        cost_running = cost_running + cost_running_temp + temp;


        // constraint values and gradients if needed
        thread_moveit_tool->updateJointState(curr_x,true);
        std::vector< std::vector<double> > dists;
        std::vector<std::vector<Eigen::Vector3d> > points1;
        std::vector<std::vector<Eigen::Vector3d> > points2;
        thread_moveit_tool->getDistsAndPoints(dists,points1,points2);
        for(int j=0;j<num_obstacles;j++)
          for(int k=0;k<num_links;k++)
          {
            constraint_vals(global_i*num_dists+j*num_links+k) = -dists[j][k]+safe_distance;
            if(cal_grad)
            {
              Eigen::Matrix<double,3,7> J_link;
              jaco_pos_calculator.getAnalyJaco4Ref(k,curr_x,J_link,points1[j][k]);
              Eigen::Vector3d normal = points1[j][k] - points2[j][k];
              normal = normal/normal.norm();
              if(dists[j][k]<0)
                normal = -normal;
              Eigen::Matrix<double,1,7> dc_ijk_dq_i = -normal.transpose() * J_link;
              Eigen::Matrix<double,1,7> dc_ijk_du_n = dc_ijk_dq_i*Ts;
              for(int l=0;l<std::min(ch-1,global_i+1);l++)
                constraint_grads.block<1,7>(global_i*num_dists+j*num_links+k,l*7) = dc_ijk_du_n;
              if(global_i>=ch-1)
              {
                Eigen::Matrix<double,1,7> dc_ijk_du_last = dc_ijk_dq_i*Ts*(global_i-ch+2);
                constraint_grads.block<1,7>(global_i*num_dists+j*num_links+k,(ch-1)*7) = dc_ijk_du_last;
              }
            }
          }

        // cost gradient if needed
        if(cal_grad)
        {
          Eigen::Matrix<double,7,7> Ji_quat;
          jaco_pos_calculator.getAnalyticalJacobian(curr_x,Ji_quat);
          Eigen::Matrix<double,1,7> dfidqi = diffTQPf*Ji_quat;
          Eigen::Matrix<double,1,7> dfiduj_past = -2*dfidqi*Ts;
          Eigen::Matrix<double,1,7> dfiduj_now = dfiduj_past+2*uTR;
          for(int j=0;j<std::min(ch-1,global_i+1);j++)
          {
            if(j<global_i)
              cost_grad.segment<7>(j*7) += dfiduj_past.transpose();
            else
              cost_grad.segment<7>(j*7) += dfiduj_now.transpose();
          }
          if(global_i>=ch-1)
          {
            Eigen::Matrix<double,1,7> dfiduj = 2*uTR - 2*dfidqi*Ts*(global_i-ch+2);
            cost_grad.segment<7>((ch-1)*7) += dfiduj.transpose();
          }
        }
      }
      class_ptr->deactivates_ptr[thread_index] = true;
    }
    else
      std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
  if(thread_index==0)
    std::cout << "Threads ended" << std::endl;
}


double NmpcSqp::costValGradCntValGrad(const Eigen::VectorXd& u, Eigen::VectorXd* cost_grad, Eigen::VectorXd* constraint_vals, Eigen::MatrixXd* constraint_grads)
{
//  auto t1 = std::chrono::high_resolution_clock::now();
  // some parameters
  double safe_distance = 0.02;
  unsigned int num_obstacles = getObstacleNum();
  unsigned int num_links = 3;
  unsigned int num_dists = num_links*num_obstacles;
  Vector7d x_old = temp_joint_values, u_eigen;
  std::vector<double> x_new_vector(7);

  // multi thread preparation
//  u_temp = u;
//  for(int i=0;i<ph;i++)
//  {
//    int u_index = std::min(i,ch-1);
//    u_eigen << u[7*u_index],u[7*u_index+1],u[7*u_index+2],u[7*u_index+3],u[7*u_index+4],u[7*u_index+5],u[7*u_index+6];
//    x_total[i] = x_old+u_eigen*Ts;
//    x_old = x_total[i];
//  }
//  *constraint_vals = Eigen::VectorXd::Zero(num_dists*ph);
//  if(cost_grad)
//  {
//    calculate_grad = true;
//    *cost_grad = Eigen::VectorXd::Zero(n);
//    *constraint_grads = Eigen::MatrixXd::Zero(num_dists*ph,n);
//  }
//  else
//    calculate_grad = false;

//  activate=true;
//  for(int i=0;i<num_threads;i++)
//    deactivates_ptr[i] = false;

//  while(ros::ok())
//  {
//    bool all_threads_done = true;
//    for(int i=0;i<num_threads;i++)
//      all_threads_done = all_threads_done&&deactivates_ptr[i];
//    if(all_threads_done)
//      break;
//    else
//      std::this_thread::sleep_for(std::chrono::microseconds(1));
//  }
//  activate = false;

//  double multi_cost=0;
//  for(int i=0;i< num_threads;i++)
//  {
//    multi_cost += partial_cost_vals[i];
//    *constraint_vals += partial_cnt_vals[i];
//  }
//  if(cost_grad)
//  {
//    for(int i=0;i< num_threads;i++)
//    {
//      *cost_grad+=partial_cost_grads[i];
//      *constraint_grads+=partial_cnt_grads[i];
//    }

//  }
//  return multi_cost;



  // single thread
  // initialize all data to zeros.
  double cost_val = 0;
  if(cost_grad)
  {
    *cost_grad = Eigen::VectorXd::Zero(n);
  }
  if(constraint_vals)
  {
    *constraint_vals = Eigen::VectorXd::Zero(num_dists*ph);
  }
  if(constraint_grads)
  {
    *constraint_grads = Eigen::MatrixXd::Zero(num_dists*ph,n);
  }

  Vector7d x_new;
  x_old = temp_joint_values;
  for(int i=0;i<ph;i++)
  {
    int u_index = std::min(i,ch-1);
    u_eigen << u[7*u_index],u[7*u_index+1],u[7*u_index+2],u[7*u_index+3],u[7*u_index+4],u[7*u_index+5],u[7*u_index+6];
    x_new = x_old+u_eigen*Ts;
    x_old = x_new;
    for(int ii=0;ii<7;ii++)
      x_new_vector[ii] = x_new(ii);
    // cost value in one iteration
    // quaternions are in the order of w,x,y,z.
    Vector7d curr_position;
    jaco_pos_cal.getPosition(x_new_vector,curr_position);
    Vector7d diff_running = goal_position-curr_position;
    Eigen::Matrix<double,1,7> diffTQPf;
    Matrix7d QPf;

    // seperate running cost and terminal cost
    if(i==ph-1)
    {
      diffTQPf = diff_running.transpose()*Pf;
      QPf = Pf;
    }
    else
    {
      diffTQPf = diff_running.transpose()*Q;
      QPf = Q;
    }
    double cost_running_temp = diffTQPf*diff_running;
    Eigen::Matrix<double,1,7> uTR = u_eigen.transpose()*R;
    double temp = uTR*u_eigen;
    cost_val += cost_running_temp + temp;

    // cost grad
    Eigen::Matrix<double,7,7> Ji_quat;
    if(cost_grad)
    {
      jaco_pos_cal.getAnalyticalJacobian(x_new_vector,Ji_quat);
      Eigen::Matrix<double,1,7> dfidqi = diffTQPf*Ji_quat;
      Eigen::Matrix<double,1,7> dfiduj_past = -2*dfidqi*Ts;
      Eigen::Matrix<double,1,7> dfiduj_now = dfiduj_past+2*uTR;
      for(int j=0;j<std::min(ch-1,i+1);j++)
      {
        if(j<i)
          cost_grad->block<7,1>(j*7,0) += dfiduj_past.transpose();
        else
          cost_grad->block<7,1>(j*7,0) += dfiduj_now.transpose();
      }
      if(i>=ch-1)
      {
        Eigen::Matrix<double,1,7> dfiduj = 2*uTR - 2*dfidqi*Ts*(i-ch+2);
        cost_grad->block<7,1>((ch-1)*7,0) += dfiduj.transpose();
      }
    }

    // constraint value in one iteration
    if(constraint_vals)
    {
      updateJointState(x_new_vector,true);
      std::vector< std::vector<double> > dists;
      std::vector<std::vector<Eigen::Vector3d> > points1;
      std::vector<std::vector<Eigen::Vector3d> > points2;
      getDistsAndPoints(dists,points1,points2);
      for(int j=0;j<num_obstacles;j++)
        for(int k=0;k<num_links;k++)
        {
          (*constraint_vals)(i*num_dists+j*num_links+k) = -dists[j][k] + safe_distance;

          // in case constraint grads are not needed
          if(constraint_grads)
          {
            Eigen::MatrixXd J_link;
            getjacobian(k,J_link,points1[j][k]);
            J_link(2,0) = 0;
            Eigen::Vector3d normal = points1[j][k] - points2[j][k];
            normal = normal/normal.norm();
            if(dists[j][k]<0)
              normal = -normal;
            Eigen::Matrix<double,1,7> dc_ijk_dq_i = -normal.transpose() * J_link.topRows<3>();
            Eigen::Matrix<double,1,7> dc_ijk_du_n = dc_ijk_dq_i*Ts;
            for(int l=0;l<std::min(ch-1,i+1);l++)
              constraint_grads->block<1,7>(i*num_dists+j*num_links+k,l*7) = dc_ijk_du_n;
            if(i>=ch-1)
            {
              Eigen::Matrix<double,1,7> dc_ijk_du_last = dc_ijk_dq_i*Ts*(i-ch+2);
              constraint_grads->block<1,7>(i*num_dists+j*num_links+k,(ch-1)*7) = dc_ijk_du_last;
            }
          }
        }
    }
  }
  return cost_val;
}


void NmpcSqp::sqpOptimizer()
{
  Eigen::VectorXd improve_direction;

  std::cout << "start optimization." << std::endl;
  double cost_val;
  Eigen::VectorXd cost_grad;
  Eigen::VectorXd constraint_vals;
  Eigen::MatrixXd constraint_grads;

  // get hessian gradient and constraint
  cost_val = costValGradCntValGrad(u,&cost_grad,&constraint_vals,&constraint_grads);

  // -inf lower bound for constraint values
  Eigen::VectorXd Inf_Vec = Inf * Eigen::VectorXd::Ones(2*m);
  Eigen::VectorXd Zero_Vec = Eigen::VectorXd::Zero(m);
  // form QP problem

  bool sub_qp_converged = false;
  for(int penalty_it=0;penalty_it<max_penalty_coe_inc_num;penalty_it++)
  {
    qp_hessian.setZero();
    for(int j=0;j<n;j++)
      qp_hessian.insert(j,j) = 1;

    Eigen::VectorXd aux_var_grad = penalty_coe*Eigen::VectorXd::Ones(m);

    for(int convex_it=0;convex_it<max_iter;convex_it++)
    {
      std::cout << "convex iteration " << convex_it << std::endl;
      // convert constraint grad matrix to sparse matrix
      //and add bounds on variables onto constraint matrix
      lb_total.resize(n+2*m);
      ub_total.resize(n+2*m);
      lb_total.tail(2*m) << Zero_Vec, constraint_vals;
      ub_total.tail(2*m) << Inf_Vec;

      // update linear constraint matrix
      qp_linearMatrix.setZero();
      for(int i=0;i<m+n;i++)
        qp_linearMatrix.insert(i,i)=1;
      for(int i=0;i<m;i++)
        qp_linearMatrix.insert(m+n+i,n+i)=1;
      for(int mi=0;mi<m;mi++)
        for(int ni=0;ni<n;ni++)
        {
          double cnt_grads_val = constraint_grads(mi,ni);
          if(cnt_grads_val>1e-5 || cnt_grads_val<-1e-5)
            qp_linearMatrix.insert(m+n+mi,ni) = -cnt_grads_val;
        }
      // update gradient of objective
      qp_gradient << cost_grad, aux_var_grad;

      if(!solver.updateGradient(qp_gradient))
        ROS_ERROR("wrong updating gradient");
      if(!solver.updateHessianMatrix(qp_hessian))
        ROS_ERROR("wrong updating hessian");
      if(!solver.updateLinearConstraintsMatrix(qp_linearMatrix))
        ROS_ERROR("wrong updating linear constraint");
      while(trust_box_size>min_trust_box_size)
      {

        // set bounds on variables based on physical bounds and trust region bounds
        for(int j=0;j<n;j++)
        {
          lb_total(j) = max(lb_onVar(j)-u(j), -trust_box_size);
          ub_total(j) = min(ub_onVar(j)-u(j), trust_box_size);
        }
        if(!solver.updateLowerBound(lb_total))
          ROS_ERROR("wrong updating lower bounds");
        if(!solver.updateUpperBound(ub_total))
          ROS_ERROR("wrong updating upper bounds");
        if(!solver.solve())
        {
          ROS_ERROR("wrong solution");
          break;
        }

        double model_improve = -solver.workspace()->info->obj_val;
        if(model_improve < -1e-4)
        {
          ROS_ERROR("QP got worse solution, exit.");
          std::cout << "model improve: " << model_improve << std::endl;
          ros::shutdown();
        }
        if(model_improve<min_approx_improve)
        {
          ROS_INFO_STREAM("Converged because improve is small,sqp completed, improve: "<< model_improve );
          sub_qp_converged = true;
          break;
        }
        improve_direction = solver.getSolution();
        Eigen::VectorXd u_new = u + improve_direction.head(n);
//        std::vector<double> u_new(n);
//        for(int j=0;j<n;j++)
//          u_new[j] = u[j]+improve_direction(j);

        double objective_old = cost_val + penalty_coe*constraint_vals.cwiseMax(0).sum();
        Eigen::VectorXd constraint_vals_new;
        auto t1 = std::chrono::high_resolution_clock::now();
        double cost_val_new = costValGradCntValGrad(u_new,nullptr,&constraint_vals_new,nullptr);
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
        std::cout << "No grad duration: " << duration1 << std::endl;
        double objective_new = cost_val_new + penalty_coe*constraint_vals_new.cwiseMax(0).sum();
        double true_improve = objective_old - objective_new;
        double improve_ratio = true_improve/model_improve;

        if(true_improve<0||improve_ratio<improve_ratio_threshold)
        {
          ROS_INFO("QP goes too far, shrink the trust region and re opt on origial point");
          trust_box_size *= trust_shrink_ratio;
        }
        else
        {
          u = u_new;
          auto t3 = std::chrono::high_resolution_clock::now();
          cost_val = costValGradCntValGrad(u,&cost_grad,&constraint_vals,&constraint_grads);
          auto t4 = std::chrono::high_resolution_clock::now();
          auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>( t4 - t3 ).count();
          std::cout << "With grad duration: " << duration2 << std::endl;
          trust_box_size *= trust_expand_ratio;
          ROS_INFO_STREAM("Solution accepted, move on to new point opt. Model improve: " << model_improve << ", true improve" << true_improve);
          break;
        }
      }
      if(sub_qp_converged)
        break;
      else if(trust_box_size<min_trust_box_size)
      {
        ROS_INFO("Converged because trust region is small");
        break;
      }
      else if(convex_it == max_iter-1)
      {
        ROS_INFO("Maximum outer iteration limit reached.");
      }
      std::cout << "--------" << std::endl;
    }
    break;
    // in case trust_box_size shrinks below minimum size
    trust_box_size = max(trust_box_size, min_trust_box_size/trust_shrink_ratio*1.5);
  }



  std::vector<double> traj_point_q(7);
  line_strip.color.g = 0.0;
  line_strip.color.r = 1.0;
  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.ns = "trajectory";
  line_strip.points.clear();
  line_strip.points.resize(ph+1);

  for(int i=0;i<7;i++)
    traj_point_q[i] = temp_joint_values(i);
  updateJointState(traj_point_q);
  geometry_msgs::Point traj_point_x;
  Eigen::Vector3d point_x_position = getEEFTransform().translation();
  traj_point_x.x = point_x_position(0);
  traj_point_x.y = point_x_position(1);
  traj_point_x.z = point_x_position(2);
  line_strip.points[0] = traj_point_x;

  for(int i=0;i<ph;i++)
  {
    int index = std::min(i,ch-1);
    for(int j=0;j<7;j++)
      traj_point_q[j] += u[index*7+j]*Ts;
    updateJointState(traj_point_q);
    point_x_position = getEEFTransform().translation();
    traj_point_x.x = point_x_position(0);
    traj_point_x.y = point_x_position(1);
    traj_point_x.z = point_x_position(2);
    line_strip.points[i+1] = traj_point_x;
  }
  marker_publisher.publish(line_strip);

  line_strip.color.g = 1.0;
  line_strip.color.r = 0.0;
  line_strip.id = 2;
  line_strip.type = visualization_msgs::Marker::POINTS;
  line_strip.ns = "trajectory_point";
  marker_publisher.publish(line_strip);
  return;
}


void NmpcSqp::controlLoop()
{
  if(!state_sub_started)
    return;
  // update joint states
  for(int i=0;i<7;i++)
  {
    temp_joint_values(i) = curr_joint_values[i];
    temp_joint_vels(i) = curr_joint_vels[i];
  }


//  auto t1 = std::chrono::high_resolution_clock::now();

  if(track_mode&&goal_sub_started)
  {
    auto t1 = std::chrono::high_resolution_clock::now();
    sqpOptimizer();
    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
    std::cout << "duration1: " << duration << std::endl;
    ros::shutdown();

    // warm start
//    for(int i=0;i<(ch-1);i++)
//     for(int j=0;j<7;j++)
//     {
//       u[i*7+j] = u[(i+1)*7+j];
//     }
//    for(int j=0;j<7;j++)
//    {
//      u[(ch-1)*7+j]=0;
//    }
  }

}
