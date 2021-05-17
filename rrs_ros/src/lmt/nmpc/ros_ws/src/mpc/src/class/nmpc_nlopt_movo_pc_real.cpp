#include <nmpc_nlopt_movo.h>
#include <chrono>
NmpcNlopt::NmpcNlopt(ros::NodeHandle nh):MoveitTool(nh)
{
  std::cout << "Unity version pc_REAL" << " ************ " << std::endl;
  ROS_WARN("Unity Version Loading... ****************************** REAL");
  // load parameters
  string node_name = ros::this_node::getName();
  bool param_ok=true;
  double Q1v,Q2v,Pf1v,Pf2v,Rv;
  if(!nh.getParam("/nlmpc/cost_weight/Q1v",Q1v))
  {
    ROS_ERROR("error loading  Q1v param");
     param_ok = false;
  }
  if(!nh.getParam("/nlmpc/cost_weight/Q2v",Q2v))
  {
    ROS_ERROR("error loading  Q2v param");
     param_ok = false;
  }
  if(!nh.getParam("/nlmpc/cost_weight/Pf1v",Pf1v))
  {
    ROS_ERROR("error loading  Pf1v param");
     param_ok = false;
  }
  if(!nh.getParam("/nlmpc/cost_weight/Pf2v",Pf2v))
  {
    ROS_ERROR("error loading  Pf2v param");
     param_ok = false;
  }
  if(!nh.getParam("/nlmpc/cost_weight/Rv",Rv))
  {
    ROS_ERROR("error loading  Rv param");
     param_ok = false;
  }
  if(!nh.getParam("/nlmpc/ph",ph))
  {
    ROS_ERROR("error loading  prediction horizon param");
     param_ok = false;
  }
  if(!nh.getParam("/nlmpc/ch",ch))
  {
    ROS_ERROR("error loading  control horizon param");
     param_ok = false;
  }

  int maxeval;
  if(!nh.getParam("/nlmpc/max_eval_num",maxeval))
  {
    ROS_ERROR("error loading  max_eval_num");
     param_ok = false;
  }

  if(!nh.getParam(node_name+"/arm_name_space",arm_name_space))
  {
    ROS_ERROR("error loading  arm_name_space");
     param_ok = false;
  }

  if(!param_ok)
  {
    ros::shutdown();
  }
  Vector6d diag1;
  diag1 << Q1v,Q1v,Q1v,Q2v,Q2v,Q2v;
  Q = diag1.asDiagonal();
  diag1 << Pf1v,Pf1v,Pf1v,Pf2v,Pf2v,Pf2v;
  Pf = diag1.asDiagonal();
  Vector7d diag2;
  diag2 << Rv,Rv,Rv,Rv,Rv,Rv,0.1*Rv;
  R = diag2.asDiagonal();


  curr_joint_values.resize(joint_num);
  last_joint_values.resize(joint_num);
  curr_joint_vels.resize(joint_num);
  error_integral = {0,0,0,0,0,0,0};


  joint_state_sub = nh.subscribe("/movo/"+arm_name_space+"_arm_controller/state", 10, &NmpcNlopt::jointStateSubCB_real,this); 
  movo2kinova_offset = {0,3.1416,0,3.1416,0,3.1416,0};

  state_sub_started = false;
  goal_sub_started = false;
  goal_sub = nh.subscribe("/"+arm_name_space+"_ee_control_marker/feedback",10,&NmpcNlopt::goalSubCB,this);

  velocity_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/movo/"+arm_name_space+"_arm_controller/command777",1);
  if(arm_name_space=="left") 
  {
    goal_msg.joint_names={"left_shoulder_pan_joint", "left_shoulder_lift_joint", "left_arm_half_joint", "left_elbow_joint",
                "left_wrist_spherical_1_joint", "left_wrist_spherical_2_joint", "left_wrist_3_joint"};
  }
  else
  {
    goal_msg.joint_names={"right_shoulder_pan_joint", "right_shoulder_lift_joint", "right_arm_half_joint", "right_elbow_joint",
                "right_wrist_spherical_1_joint", "right_wrist_spherical_2_joint", "right_wrist_3_joint"};
  }
  cylinders_sent = false;
  self_cylinders_pub=nh.advertise<perception_msgs::Cylinders>(arm_name_space+"/nmpc_controller/out/cylinders_ps",1);
  octomap_sub = nh.subscribe("/octomap_binary",1,&NmpcNlopt::octomap_sub_cb,this);
 

  // eef_pose_pub = nh.advertise<geometry_msgs::Pose>(arm_name_space+"/nmpc_controller/out/eef_pose",1);


  // server for toggle tracking mode
  track_toggle_server = nh.advertiseService("/j2s7s300/track_toggle",&NmpcNlopt::track_toggle_cb,this);
  track_mode = false;

  Ts = 0.2;
  interval = ros::Duration(Ts);

  joint_velocities_mutex = PTHREAD_MUTEX_INITIALIZER;
  position_goal_mutex = PTHREAD_MUTEX_INITIALIZER;

  // initialize optmizer
  optimizer = nlopt::opt(nlopt::LD_SLSQP, ch*7);


  // add upper and lower bounds;
  lb.resize(ch*7);
  std::fill(lb.begin(),lb.end(),-0.17453);
  optimizer.set_lower_bounds(lb);
  ub.resize(ch*7);
  std::fill(ub.begin(),ub.end(),0.17453);
  optimizer.set_upper_bounds(ub);

  for(int i=0;i<ch;i++)
  {
    lb[7*i+6] = -0.4;
    ub[7*i+6] = 0.4;
  }

  // set objective cost function to minimize
  optimizer.set_min_objective(costFuncMultiThread,this);
  optimizer.set_ftol_rel(1e-6);
  optimizer.set_maxeval(maxeval);

  u.resize(ch*7);
  std::fill(u.begin(),u.end(),0);

  // set joint_velocities[0] to -200 to represent that controller has not generate velocity command and
  // velocity sending thread should not send command to robot.
  joint_velocities.resize(7);
  joint_velocities[0] = -200;

//  std::fill(joint_velocities.begin(),joint_velocities.end(),0);

  // set exitFlag to false first
  exitFlag=false;
  new_goal_got = false;
  position_goal.resize(joint_num);
  // start the thread
  thread1 = std::thread(velocitiesSend_thread_sim, velocity_pub, &exitFlag, &joint_velocities, &goal_msg, &joint_velocities_mutex, &position_goal_mutex,&new_goal_got, &position_goal);
  // start cost val cal thread and initiliaze shared in-output container
  num_threads = 1;
  num_loops = ph/num_threads;
  last_num_loops = num_loops+ph%num_threads;
  u_ptr = new double[ch*7];
  jnt_vals_ptr = new Vector7d[ph];
  costCal_jnt_vals_ptr = new Vector7d[ph];
  partial_costs_ptr = new double[num_threads];
  cost_activated = false;
  activated = false;
  cost_calculate_grad = false;
  calculate_grad = false;
  finished_ptr = new std::atomic_bool[num_threads];
  cost_finished_ptr = new std::atomic_bool[num_threads];

//  mutex_vec = std::vector<std::mutex>(num_threads);
  for(int i=0;i<num_threads;i++)
  {
    finished_ptr[i] = true;
    cost_finished_ptr[i] = true;
  }
  partial_grads_ptr = new double*[num_threads];
  for(int i=0;i<num_threads;i++)
    partial_grads_ptr[i] = new double[ch*7];

  partial_cnt_val.resize(num_threads);
  partial_cnt_grads.resize(num_threads);
  MoveitTools_array.resize(num_threads);
  for(int i=0;i<num_threads;i++)
    MoveitTools_array[i] = new MoveitTool(nh);



 threads_ptr = new std::thread[num_threads];
 cost_threads_ptr = new std::thread[num_threads];
 for(int i=0;i<num_threads;i++)
 {
   cost_threads_ptr[i] = std::thread(costCalThread, i, this, u_ptr);
   ros::Duration(0.1).sleep();
   threads_ptr[i] = std::thread(cntCalThread, i, this);
   ros::Duration(0.1).sleep();
 }

  // initialize marker for visualizing predicted trajectory
  line_strip.scale.x = 5e-3;
  line_strip.scale.y = 5e-3;
  line_strip.scale.z = 5e-3;
  line_strip.header.frame_id = "left_mpc_base_link";

  // init tf for goal from outside
  // transformStamped_goal.header.frame_id = "world";
  // transformStamped_goal.child_frame_id = "goal_from_outside";
}

NmpcNlopt::~NmpcNlopt()
{
  // stop the velocities sending thread
  exitFlag = true;
  thread1.join();
  for(int i=0;i<num_threads;i++)
  {  
    cost_threads_ptr[i].join();
    threads_ptr[i].join();
  }
  delete []threads_ptr;
  delete []cost_threads_ptr;

  // delete arraies
  delete []u_ptr;
  delete []jnt_vals_ptr;
  delete []costCal_jnt_vals_ptr;
  delete []partial_costs_ptr;
  delete []finished_ptr;
  delete []cost_finished_ptr;
  for(int i=0;i<num_threads;i++)
  {
    delete []partial_grads_ptr[i];
    delete MoveitTools_array[i];
  }
  delete []partial_grads_ptr;
}
void NmpcNlopt::jointStateSubCB_real(const sensor_msgs::JointState::ConstPtr& msg)
{
	
	if(!state_sub_started)
		state_sub_started = true;
  const auto& positions = msg->position;
	for(int i=0;i<joint_num;i++)
  {
		curr_joint_values[i] = angles::normalize_angle(positions[i]+movo2kinova_offset[i]);
  }

  ROS_INFO("Get message");
}


void NmpcNlopt::goalSubCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg)
{

  goal_position << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z,
                   msg->pose.orientation.w,msg->pose.orientation.x,
                   msg->pose.orientation.y,msg->pose.orientation.z;

  if(!goal_sub_started)
    goal_sub_started = true;
}

bool NmpcNlopt::track_toggle_cb(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
  track_mode = track_mode?false:true;
  return true;
}


void NmpcNlopt::octomap_sub_cb(const octomap_msgs::OctomapConstPtr octo_msg)
{
  const auto tree_ptr = octomap_msgs::binaryMsgToMap(*octo_msg);
  octree_shape = std::make_shared<shapes::OcTree>(std::shared_ptr<const octomap::OcTree>(
                                                    dynamic_cast<octomap::OcTree*>(tree_ptr)
                                                    ));
  Eigen::Affine3d octree_pose;
  octree_pose.setIdentity();
  addObstacle("octree",octree_shape,octree_pose);
}



void NmpcNlopt::addObstacle(const std::string &obj_name, const shapes::ShapeConstPtr &obj_shape, const Eigen::Affine3d &obj_transform, const bool plot)
{

  // transfer global coordinate of the obstacle to local coordinate
  // Eigen::Affine3d local_obj_transform = origin2base * obj_transform;
  Eigen::Affine3d local_obj_transform = obj_transform;
  // rviz marker doesn't support octree, do not visualize octree objects
  if (obj_shape->type==shapes::OCTREE)
    MoveitTool::addObstacle(obj_name,obj_shape,obj_transform);
  else
    MoveitTool::addObstacle(obj_name,obj_shape,obj_transform,true);
  for(int i=0;i<num_threads;i++)
   MoveitTools_array[i]->addObstacle(obj_name,obj_shape,local_obj_transform);
  
  int m = ph*3*getObstacleNum();
  std::vector<double> tol(m,1e-3);
  optimizer.remove_inequality_constraints();
  optimizer.add_inequality_mconstraint(obstacleConstraintsMultiThread,this,tol);
}

void NmpcNlopt::updateObstacle(const std::string& obj_name, const Eigen::Affine3d& obj_transform, const bool plot)
{
  MoveitTool::updateObstacle(obj_name,obj_transform);
  for(int i=0;i<num_threads;i++)
   MoveitTools_array[i]->updateObstacle(obj_name,obj_transform);
}


void NmpcNlopt::costCalThread(const int thread_index, NmpcNlopt *class_ptr, const double* u_ptr)
{
  std::cout << "Cost cal thread[" << thread_index+1 << "] started" << std::endl;
  while(ros::ok() && !(class_ptr->exitFlag))
  {
    if(class_ptr->cost_activated && !(class_ptr->cost_finished_ptr[thread_index]))
    {
      //cost and grad calculation
      const Matrix6d& Q = class_ptr->Q;
      const Matrix6d& Pf = class_ptr->Pf;
      const Matrix7d& R = class_ptr->R;
      const double& Ts = class_ptr->Ts;
      const int& ph = class_ptr->ph;
      const int& ch = class_ptr->ch;

      // output partial cost value container
      double& cost_running = class_ptr->partial_costs_ptr[thread_index];
      // output partial cost gradient container
      double* grad = class_ptr->partial_grads_ptr[thread_index];

      cost_running = 0;
      if(class_ptr->cost_calculate_grad)
      {
        for(int i=0;i<ch*7;i++)
          grad[i]=0;
      }
      Vector7d eef_position, u_eigen;
      Vector6d diff_running;
      const Vector7d& goal_position = class_ptr->goal_position;
      std::vector<double> curr_x(7);

      int local_num_loops = (thread_index==(class_ptr->num_threads-1))?class_ptr->last_num_loops:class_ptr->num_loops;
      for(int i=0;i<local_num_loops;i++)
      {
        int global_i = i + thread_index*class_ptr->num_loops;
        for(int j =0;j<7;j++)
          curr_x[j] = class_ptr->costCal_jnt_vals_ptr[global_i][j];
        class_ptr->jaco_hess_cal.getPosition(curr_x, eef_position);

        Eigen::Vector3d diff_position = goal_position.head<3>() - eef_position.head<3>();
        double w1=eef_position(3),w2=goal_position(3);
        Eigen::Vector3d q1=eef_position.tail<3>(),q2=goal_position.tail<3>();
        Eigen::Matrix3d skew_q2;
        skew_q2 << 0,-q2(2),q2(1),q2(2),0,-q2(0),-q2(1),q2(0),0;
        Eigen::Vector3d diff_orientation = w1*q2-w2*q1-skew_q2*q1;
        diff_running << diff_position,diff_orientation;


        Eigen::Matrix<double,1,6> diffTQPf;
        Matrix6d QPf;
        if(global_i==ph-1)
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

        int u_index = std::min(global_i,ch-1);
        u_eigen << u_ptr[7*u_index],u_ptr[7*u_index+1],u_ptr[7*u_index+2],
                   u_ptr[7*u_index+3],u_ptr[7*u_index+4],u_ptr[7*u_index+5],
                   u_ptr[7*u_index+6];

        Eigen::Matrix<double,1,7> uTR = u_eigen.transpose()*R;
        double temp = uTR*u_eigen;
        cost_running = cost_running + cost_running_temp + temp;

        if(!class_ptr->cost_calculate_grad)
          continue;
        Eigen::Matrix<double,7,7> Ji_quat;
        class_ptr->jaco_hess_cal.getAnalyticalJacobian(curr_x,Ji_quat);
        Eigen::Matrix<double,1,7> dfidqi_pos = diffTQPf.leftCols<3>()*-2*Ji_quat.topRows<3>();
        Eigen::Matrix<double,3,4> quat_update;
        quat_update.col(0) = q2;
        quat_update.rightCols<3>() = -w2*Eigen::MatrixXd::Identity(3,3)-skew_q2;
        Eigen::Matrix<double,1,7> dfidqi_quat = diffTQPf.rightCols<3>()*2*quat_update*Ji_quat.bottomRows<4>();
        Eigen::Matrix<double,1,7> dfidqi = dfidqi_pos+dfidqi_quat;

        Eigen::Matrix<double,1,7> dfiduj_past = dfidqi*Ts;
        Eigen::Matrix<double,1,7> dfiduj_now = dfiduj_past+2*uTR;

        for(int j=0;j<std::min(ch-1,global_i+1);j++)
        {
          if(j<global_i)
          {
            for(int k=0;k<7;k++)
              grad[j*7+k] += dfiduj_past(k);
          }
          else
          {
            for(int k=0;k<7;k++)
              grad[j*7+k] += dfiduj_now(k);
          }
        }
        if(global_i>=ch-1)
        {
          Eigen::Matrix<double,1,7> dfiduj = 2*uTR + dfidqi*Ts*(global_i-ch+2);
          for(int k=0;k<7;k++)
            grad[(ch-1)*7+k] += dfiduj(k);
        }

      }
      class_ptr->cost_finished_ptr[thread_index] = true;
    }
    else
      std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
  if(thread_index==0)
    std::cout << "Cost cal threads ended." << std::endl;
}


double NmpcNlopt::costFuncMultiThread(unsigned n, const double *u, double *grad, void *data)
{
  NmpcNlopt* class_ptr = reinterpret_cast<NmpcNlopt*>(data);
  const double& Ts = class_ptr->Ts;
  const int& ph = class_ptr->ph;
  const int& ch = class_ptr->ch;

  //multi threading preparation
  if(grad)
    class_ptr->cost_calculate_grad = true;
  else
    class_ptr->cost_calculate_grad = false;

  for(int j=0;j<7;j++)
  {
    class_ptr->u_ptr[j] = u[j];
    class_ptr->costCal_jnt_vals_ptr[0][j] =  class_ptr->temp_joint_values[j]+u[j]*Ts;
  }

  for(int i=1;i<ph;i++)
  {
    int u_index = std::min(i,ch-1);
    for(int j=0;j<7;j++)
    {
      double u_temp = u[u_index*7+j];
      class_ptr->u_ptr[u_index*7+j] = u_temp;
      class_ptr->costCal_jnt_vals_ptr[i][j] = class_ptr->costCal_jnt_vals_ptr[i-1][j]+u_temp*Ts;
    }
  }
  for(int i=0;i<class_ptr->num_threads;i++)
    class_ptr->cost_finished_ptr[i]=false;
  class_ptr->cost_activated = true;


  // wait until all threads calculation finished
  while(ros::ok())
  {
    bool all_threads_done = true;
    for(int i=0;i<class_ptr->num_threads;i++)
    {
      all_threads_done = all_threads_done&&class_ptr->cost_finished_ptr[i];
    }
    if(all_threads_done)
      break;
    else
    {
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
  }
  // deactivate cost cal in threads

  class_ptr->cost_activated = false;

  double multi_cost = 0;
  for(int i=0;i<class_ptr->num_threads;i++)
    multi_cost += class_ptr->partial_costs_ptr[i];
  if(grad)
  {
    for(int j=0;j<ch*7;j++)
    {
      grad[j] = 0;
      for(int i=0;i<class_ptr->num_threads;i++)
        grad[j] += class_ptr->partial_grads_ptr[i][j];
    }
  }
  return multi_cost;
}

void NmpcNlopt::cntCalThread(const int thread_index, NmpcNlopt *class_ptr)
{
  std::cout << "Constraint cal thread[" << thread_index+1 << "] started" << std::endl;
  while(ros::ok() && !(class_ptr->exitFlag))
  {
    if(class_ptr->activated && !(class_ptr->finished_ptr[thread_index]))
    {
      //cost and grad calculation
      const double& Ts = class_ptr->Ts;
      const int& ch = class_ptr->ch;
      const double safe_distance = 0.02;
      const int& num_obstacles = class_ptr->getObstacleNum();
      const int num_links = 3;
      const int num_dists = num_links*num_obstacles;
      const int n = 7*ch;

      // output partial cost value container
      std::vector<double>& cnt_vals = class_ptr->partial_cnt_val[thread_index];
      // output partial cost gradient container
      std::vector<double>& cnt_grads = class_ptr->partial_cnt_grads[thread_index];

      std::vector<double> curr_x(7);

      int local_num_loops = (thread_index==(class_ptr->num_threads-1))?class_ptr->last_num_loops:class_ptr->num_loops;



      for(int i=0;i<local_num_loops;i++)
      {
      	auto t3 = std::chrono::high_resolution_clock::now();
        int global_i = i + thread_index*class_ptr->num_loops;
        for(int j =0;j<7;j++)
          curr_x[j] = class_ptr->jnt_vals_ptr[global_i][j];
        class_ptr->MoveitTools_array[thread_index]->updateJointState(curr_x, true);
        std::vector< std::vector<double> > dists;
        std::vector<std::vector<Eigen::Vector3d> > points1;
        std::vector<std::vector<Eigen::Vector3d> > points2;
        class_ptr->MoveitTools_array[thread_index]->getDistsAndPoints(dists,points1,points2);
        for(int j=0;j<num_obstacles;j++)
          for(int k=0;k<num_links;k++)
          {
          	auto t5 = std::chrono::high_resolution_clock::now();
            cnt_vals[global_i*num_dists+j*num_links+k] = -dists[j][k] + safe_distance;
            if(class_ptr->calculate_grad)
            {
              Eigen::Matrix<double,3,7> J_link;
              class_ptr->jaco_hess_cal.getAnalyJaco4Ref(k,curr_x, J_link, points1[j][k]);
              Eigen::Vector3d normal = points1[j][k] - points2[j][k];
              normal = normal/normal.norm();
              if(dists[j][k]<0)
                normal = -normal;
              Eigen::Matrix<double,1,7> dc_ijk_dq_i = -normal.transpose() * J_link;
              Eigen::Matrix<double,1,7> dc_ijk_du_n = dc_ijk_dq_i*Ts;
              for(int l=0;l<std::min(ch-1,global_i+1);l++)
              {
                for(int m1=0;m1<7;m1++)
                {
                  cnt_grads[global_i*num_dists*n+j*num_links*n+k*n+l*7+m1]=dc_ijk_du_n(m1);
                }
              }
              if(global_i>=ch-1)
              {
                Eigen::Matrix<double,1,7> dc_ijk_du_last = dc_ijk_dq_i*Ts*(global_i-ch+2);
                for(int m1=0;m1<7;m1++)
                {
                 cnt_grads[global_i*num_dists*n+j*num_links*n+k*n+(ch-1)*7+m1]=dc_ijk_du_last(m1);
                }
              }
            }

          }
      }
      class_ptr->finished_ptr[thread_index] = true;
    }
    else
      std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
  if(thread_index==0)
    std::cout << "Constraint cal threads ended." << std::endl;
}

void NmpcNlopt::obstacleConstraintsMultiThread(unsigned m, double *result, unsigned n, const double *u, double *grad, void *data)
{
  // auto t1 = std::chrono::high_resolution_clock::now();
  NmpcNlopt* class_ptr = reinterpret_cast<NmpcNlopt*>(data);
  const double& Ts = class_ptr->Ts;
  const int& ph = class_ptr->ph;
  const int& ch = class_ptr->ch;

  if(grad)
  {
    for(int i=0;i<m*n;i++)
      grad[i]=0;
    class_ptr->calculate_grad = true;
    for(int i=0;i<class_ptr->num_threads;i++)
      class_ptr->partial_cnt_grads[i].resize(m*n);
  }
  else
    class_ptr->calculate_grad = false;
  for(int i=0;i<class_ptr->num_threads;i++)
    class_ptr->partial_cnt_val[i].resize(m);
  for(int j=0;j<7;j++)
    class_ptr->jnt_vals_ptr[0][j] =  class_ptr->temp_joint_values[j]+u[j]*Ts;
  for(int i=1;i<ph;i++)
  {
    int u_index = std::min(i,ch-1);
    for(int j=0;j<7;j++)
    {
      double u_temp = u[u_index*7+j];
      class_ptr->jnt_vals_ptr[i][j] = class_ptr->jnt_vals_ptr[i-1][j]+u_temp*Ts;
    }
  }
  class_ptr->activated = true;
  for(int i=0;i<class_ptr->num_threads;i++)
    class_ptr->finished_ptr[i]=false;
  while(ros::ok())
  {
    bool all_threads_done = true;
    for(int i=0;i<class_ptr->num_threads;i++)
      all_threads_done = all_threads_done&&class_ptr->finished_ptr[i];

    if(all_threads_done)
      break;
    else
      std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
  class_ptr->activated = false;

  for(int i=0;i<m;i++)
  {
    result[i] = 0;
    for(int j=0;j<class_ptr->num_threads;j++)
      result[i]+=class_ptr->partial_cnt_val[j][i];
  }
  if(grad)
  {
    for(int i=0;i<m*n;i++)
    {
      grad[i]=0;
      for(int j=0;j<class_ptr->num_threads;j++)
      {
        grad[i] += class_ptr->partial_cnt_grads[j][i];
      }
    }
  }
  return;
}


void NmpcNlopt::optimize(std::vector<double>& u, double mincost)
{
  auto t1 = std::chrono::high_resolution_clock::now();
  try{
      nlopt::result result = optimizer.optimize(u,mincost);
  }
  catch(std::exception &e){
    std::cout << "nlopt failed: " << e.what() << std::endl;
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
  std::cout << duration << std::endl;

  // discard mvs in [1,ch-1]-th time step
  // only send the mv in the frist time step: 0th time step
  pthread_mutex_lock(&joint_velocities_mutex);
  for(int i=0;i<7;i++)
  {
    joint_velocities[i] = u[i];
  }
  pthread_mutex_unlock(&joint_velocities_mutex);
  new_goal_got = true;

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
}

// velocitySend thread for arm in movo simulation
void NmpcNlopt::velocitiesSend_thread_sim(ros::Publisher publisher, bool* exitFlag,
                                      std::vector<double>* v_ptr, trajectory_msgs::JointTrajectory* msg,
                                      pthread_mutex_t *joint_velocities_mutex, pthread_mutex_t* position_goal_mutex,
                                      bool* new_goal_got, std::vector<double>* position_goal_ptr)
{
  double passsed_time_after_new_goal;
  std::vector<double> temp_v(7);
  ros::Duration d(0.01);
  while(ros::ok() && !(*exitFlag))
  {
    if((*v_ptr)[0]!=-200)
    {

      if(*new_goal_got)
      {
        passsed_time_after_new_goal = 0;
        *new_goal_got = false;
      }
      else
      {
        passsed_time_after_new_goal +=0.01;
        if(passsed_time_after_new_goal > 0.2)
        {
//          ROS_WARN("Joint velocities not updte for a long time, sending 0 velocities to robot.");
          std::fill(temp_v.begin(),temp_v.end(),0);
        }
        else
        {
          pthread_mutex_lock(joint_velocities_mutex);
          temp_v = *v_ptr;
          pthread_mutex_unlock(joint_velocities_mutex);
        }
      }

      msg->header.seq++;
      msg->points.clear();
      trajectory_msgs::JointTrajectoryPoint msg_point;
      msg_point.time_from_start = d;
      msg_point.positions.resize(7);
      double movo2kinova_offset[7] = {0,3.1416,0,3.1416,0,3.1416,0};
      pthread_mutex_lock(position_goal_mutex);
      for(int i=0;i<7;i++)
      {
        (*position_goal_ptr)[i] += (temp_v[i])*0.01;
        msg_point.positions[i] = angles::normalize_angle((*position_goal_ptr)[i]-movo2kinova_offset[i]);
      }
      pthread_mutex_unlock(position_goal_mutex);

      // msg_point.positions = *position_goal_ptr;
      msg->points.push_back(msg_point);
      publisher.publish(*msg);
    }
    d.sleep();
  }
  std::cout << "Velocities sending thread ended. " << std::endl;
}

inline void NmpcNlopt::publish_cylinders()
{
  updateJointState(curr_joint_values,true,true);
  perception_msgs::Cylinders cylinders_msg;
  geometry_msgs::Point point;
  cylinders_msg.header.frame_id="left_mpc_base_link";
  for(int i=0;i<3;i++)
  {
    point.x = centroids[i][0];
    point.y = centroids[i][1];
    point.z = centroids[i][2];
    cylinders_msg.centers.push_back(point);

    point.x = z_axes[i][0];
    point.y = z_axes[i][1];
    point.z = z_axes[i][2];
    cylinders_msg.z_axes.push_back(point);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  self_cylinders_pub.publish(cylinders_msg);
  cylinders_sent = true;
  return;
}


void NmpcNlopt::initialize()
{
  while(!state_sub_started)
    ros::Duration(0.5).sleep();
}

void NmpcNlopt::controlLoop()
{
  if(!state_sub_started)
    return;
  // update joint states
  pthread_mutex_lock(&position_goal_mutex);
  for(int i=0;i<7;i++)
  {
    temp_joint_values(i) = curr_joint_values[i];
    position_goal[i] = curr_joint_values[i];
  }
  pthread_mutex_unlock(&position_goal_mutex);
  if(!cylinders_sent)
    publish_cylinders();

 // if(track_mode&&goal_sub_started)
  if(goal_sub_started)
  {
    optimize(u,mincost);

//     warm start
    for(int i=0;i<(ch-1);i++)
     for(int j=0;j<7;j++)
     {
       u[i*7+j] = u[(i+1)*7+j];
     }
    for(int j=0;j<7;j++)
    {
      u[(ch-1)*7+j]=0;
    }
  }

}