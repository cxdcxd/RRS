#include <nmpc_nlopt.h>
#include <chrono>
NmpcNlopt::NmpcNlopt(ros::NodeHandle nh):MoveitTool(nh)
{
  // load parameters
  bool param_ok=true;
  if(!nh.getParam("nlopt_mpc_main_node/j2s7s300/jaco_inv_pid/p",p))
  {
    ROS_ERROR("error loading  p param");
    param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/j2s7s300/jaco_inv_pid/i",i))
  {
    ROS_ERROR("error loading  i param");
    param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/j2s7s300/jaco_inv_pid/d",d))
  {
    ROS_ERROR("error loading  d param");
     param_ok = false;
  }

  double Q1v,Q2v,Pf1v,Pf2v,Rv;
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/cost_weight/Q1v",Q1v))
  {
    ROS_ERROR("error loading  Q1v param");
     param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/cost_weight/Q2v",Q2v))
  {
    ROS_ERROR("error loading  Q2v param");
     param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/cost_weight/Pf1v",Pf1v))
  {
    ROS_ERROR("error loading  Pf1v param");
     param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/cost_weight/Pf2v",Pf2v))
  {
    ROS_ERROR("error loading  Pf2v param");
     param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/cost_weight/Rv",Rv))
  {
    ROS_ERROR("error loading  Rv param");
     param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/ph",ph))
  {
    ROS_ERROR("error loading  prediction horizon param");
     param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/ch",ch))
  {
    ROS_ERROR("error loading  control horizon param");
     param_ok = false;
  }

  int maxeval;
  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/max_eval_num",maxeval))
  {
    ROS_ERROR("error loading  max_eval_num");
     param_ok = false;
  }

  // outout filename parameter
//  string filename1;
//  if(!nh.getParam("nlopt_mpc_main_node/nlmpc/filename1",filename1))
//  {
//    ROS_ERROR("error loading  filename1");
//     param_ok = false;
//  }


  // parameter: number of threads
  if(!nh.getParam("nlopt_mpc_main_node/multi_threading/num_cost_threads",num_cost_threads))
  {
    ROS_ERROR("error loading  num_cost_threads");
     param_ok = false;
  }
  if(!nh.getParam("nlopt_mpc_main_node/multi_threading/num_cnt_threads",num_cnt_threads))
  {
    ROS_ERROR("error loading  num_cnt_threads");
     param_ok = false;
  }

  // goal
  std::vector<double> goal_vec;
  if(!nh.getParam("nlopt_mpc_main_node/goal",goal_vec))
  {
    ROS_ERROR("error loading  goal_vec");
     param_ok = false;
  }
  if(!param_ok)
  {
    ros::shutdown();
  }
  for(int i=0;i<7;i++)
  {
    goal_position(i)=goal_vec[i];
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

   joint_state_sub = nh.subscribe("/j2s7s300/joint_states", 100, &NmpcNlopt::jointStateSubCB,this);
  state_sub_started = false;
  goal_sub_started = false;
  goal_sub = nh.subscribe("/simple_marker/feedback",100,&NmpcNlopt::goalSubCB,this);

  // this is for using a trajectory controller
  goal_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/j2s7s300/position_joint_trajectory_controller/command",1);
  // eef_pose_pub = nh.advertise<geometry_msgs::Pose>("/nmpc_controller/out/eef_pose",1);

  // feed unchanged content into goal_msg
  goal_msg.joint_names = {"j2s7s300_joint_1","j2s7s300_joint_2","j2s7s300_joint_3","j2s7s300_joint_4",
                         "j2s7s300_joint_5","j2s7s300_joint_6","j2s7s300_joint_7"};

  // server for toggle tracking mode
  track_toggle_server = nh.advertiseService("/j2s7s300/track_toggle",&NmpcNlopt::track_toggle_cb,this);
  track_mode = false;

  Ts = 0.2;
  interval = ros::Duration(Ts);

  joint_velocities_mutex = PTHREAD_MUTEX_INITIALIZER;
  position_goal_mutex = PTHREAD_MUTEX_INITIALIZER;

  // initialize optmizer
  optimizer = nlopt::opt(nlopt::LD_SLSQP, ch*7);
  simple_optimizer = nlopt::opt(nlopt::LD_SLSQP, ch*7);

  // add upper and lower bounds;
  lb.resize(ch*7);
  std::fill(lb.begin(),lb.end(),-0.34);
  ub.resize(ch*7);
  std::fill(ub.begin(),ub.end(),0.34);
  for(int i=0;i<ch;i++)
  {
    lb[7*i+6] = -0.8;
    ub[7*i+6] = 0.8;
  }
  optimizer.set_lower_bounds(lb);
  optimizer.set_upper_bounds(ub);
  // set objective cost function to minimize
  optimizer.set_min_objective(costFuncMultiThread,this);
  optimizer.set_ftol_rel(1e-6);
  optimizer.set_maxeval(maxeval);

  std::vector<double> tol(ph,1e-2);
  optimizer.add_inequality_mconstraint(obstacleConstraintsMultiThread,this,tol);

  simple_optimizer.set_lower_bounds(lb);
  simple_optimizer.set_upper_bounds(ub);
  simple_optimizer.set_min_objective(constantCostFunc,this);
  simple_optimizer.set_xtol_rel(1e-6);
  simple_optimizer.set_maxeval(200);
  simple_optimizer.add_inequality_mconstraint(obstacleConstraintsMultiThread,this,tol);

  u.resize(ch*7);
  std::fill(u.begin(),u.end(),0);

  // set joint_velocities[0] to -200 to represent that controller has not generate velocity command and
  // velocity sending thread should not send command to robot.
  joint_velocities.resize(7);
  joint_velocities[0] = -200;

  position_goal.resize(joint_num);
//  std::fill(joint_velocities.begin(),joint_velocities.end(),0);

  // set exitFlag to false first
  exitFlag=false;
  new_goal_got = false;

  // start the velocity sending thread
  thread1 = std::thread(velocitiesSend_thread, goal_pub, &exitFlag, &joint_velocities, &goal_msg, &joint_velocities_mutex, &position_goal_mutex,&new_goal_got, &position_goal);

  // start cost val cal thread and initiliaze shared in-output container
  num_cost_loops = ph/num_cost_threads;
  last_num_cost_loops = num_cost_loops+ph%num_cost_threads;
  num_cnt_loops = ph/num_cnt_threads;
  last_num_cnt_loops = num_cnt_loops+ph%num_cnt_threads;

  // threads input
  u_ptr = new double[ch*7];
  jnt_vals_ptr = new Vector7d[ph];

  // cost calculation threads preparetion
  costCal_jnt_vals_ptr = new Vector7d[ph];
  partial_costs_ptr = new double[num_cost_threads];
  partial_grads_ptr = new double*[num_cost_threads];
  cost_activated = false;
  cost_finished_ptr = new std::atomic_bool[num_cost_threads];
  cost_calculate_grad = false;
  cost_threads_ptr = new std::thread[num_cost_threads];

  for(int i=0;i<num_cost_threads;i++)
  {
 	partial_grads_ptr[i] = new double[ch*7];
 	cost_finished_ptr[i] = true;
 	cost_threads_ptr[i] = std::thread(costCalThread, i, this, u_ptr);
 	ros::Duration(0.1).sleep();
  }


  // constraint calculation threads preparetion
  activated = false;
  calculate_grad = false;
  finished_ptr = new std::atomic_bool[num_cnt_threads];
  threads_ptr = new std::thread[num_cnt_threads];
  for(int i=0;i<num_cnt_threads;i++)
  {
    finished_ptr[i] = true;
    threads_ptr[i] = std::thread(cntCalThread, i, this);
    ros::Duration(0.1).sleep();
  }
  // shared in-output constainer for constraint cal thread INITIALIZATION
  partial_cnt_val.resize(num_cnt_threads);
  partial_cnt_grads.resize(num_cnt_threads);
  MoveitTools_array.resize(num_cnt_threads);
  for(int i=0;i<num_cnt_threads;i++)
    MoveitTools_array[i] = new MoveitTool(nh);


  cal_cost_count = 0;
  time_avg = 0;
  time_consumed.resize(num_cost_threads);

  total_time = 0;
  total_joint_velocities = Vector7d::Zero();
  total_traj_length = 0;
  iter_num = 0;
  total_diff_position =0;
  total_diff_orientation = 0;

  // simulation 1 for static obstacle
 // goal_position << -0.328791,0.70096,0.512825, 0.70424,-0.241063,0.636445,0.202173;
  // simulation 2
  goal_position << -0.248104,0.760702,0.503285, 0.38258, -0.636583, 0.375553, 0.554396;
 transformStamped.header.stamp = ros::Time::now();
 transformStamped.header.frame_id = "world";
 transformStamped.child_frame_id = "goal";
 transformStamped.transform.translation.x = goal_position(0);
 transformStamped.transform.translation.y = goal_position(1);
 transformStamped.transform.translation.z = goal_position(2);
 transformStamped.transform.rotation.w = goal_position(3);
 transformStamped.transform.rotation.x = goal_position(4);
 transformStamped.transform.rotation.y = goal_position(5);
 transformStamped.transform.rotation.z = goal_position(6);
 br.sendTransform(transformStamped);

  // obstacle motion initialization for simulation2
  obstacle1_y_v=0.01;
  obstacle1_y_p=0.5;
 // // visualize the motion of obstacle1
  visualization_msgs::Marker obst_traj_line;
  obst_traj_line.header.frame_id = "world";
  obst_traj_line.action = visualization_msgs::Marker::ADD;
  obst_traj_line.pose.orientation.w = 1.0;
  obst_traj_line.scale.x = 5e-3;
  obst_traj_line.scale.y = 5e-3;
  obst_traj_line.scale.z = 5e-3;
  obst_traj_line.color.a = 1.0;
  obst_traj_line.color.r = 1.0;
  obst_traj_line.id=5;
  obst_traj_line.type = visualization_msgs::Marker::LINE_STRIP;
  obst_traj_line.ns="obst_trajectory";
  geometry_msgs::Point traj_point_x;
  traj_point_x.x = -0.3;
  traj_point_x.y = 0.5;
  traj_point_x.z = 0.5;
  obst_traj_line.points.push_back(traj_point_x);
  traj_point_x.y = 1.2;
  obst_traj_line.points.push_back(traj_point_x);
  marker_publisher.publish(obst_traj_line);
  obst_traj_line.color.g=0.5;
  obst_traj_line.color.b=0.5;
  obst_traj_line.id=6;
  obst_traj_line.type=visualization_msgs::Marker::POINTS;
  obst_traj_line.ns="obst_end_points";
  marker_publisher.publish(obst_traj_line);



  // initialize marker for visualizing predicted trajectory
  line_strip.scale.x = 5e-3;
  line_strip.scale.y = 5e-3;
  line_strip.scale.z = 5e-3;

  // initialize marker for visualizing actual trajectory
  actual_traj_strip.header.frame_id = "world";
  actual_traj_strip.action = visualization_msgs::Marker::ADD;
  actual_traj_strip.pose.orientation.w = 1.0;
  actual_traj_strip.scale.x = 5e-3;
  actual_traj_strip.scale.y = 5e-3;
  actual_traj_strip.scale.z = 5e-3;
  actual_traj_strip.color.a = 1.0;

  cylinders_pos_pub=nh.advertise<perception_msgs::Cylinders>("left/nmpc_controller/out/cylinders_ps",1);
  cylinders_sent = false;

//  octomap_sub = nh.subscribe("/octomap_binary",1,&NmpcNlopt::octomap_sub_cb,this);

  // initialize the handle to data file
//  const string filename1 = "/home/siqihu/MasterThesis/Thesis/temp_data/actual_mpc_cost2.csv";
//  const string filename2 = "/home/siqihu/MasterThesis/Thesis/temp_data/predict_mpc_cost2.csv";
//  const string filename3 = "/home/siqihu/MasterThesis/Thesis/temp_data/actual_dists2.csv";
//  fout1.open(filename1);
//  fout2.open(filename2);
//  fout3.open(filename3);
  init_prediction = true;
  data_number_counter=0;

  // If there is no other node like gazebo subscribe topic msg published by goal_pub, then wait
   ros::WallDuration sleep_t(0.5);
   while(goal_pub.getNumSubscribers() < 1)
    sleep_t.sleep();
  ROS_INFO("Controller Initialization completed.");
}

NmpcNlopt::~NmpcNlopt()
{
  // stop the velocities sending thread
  exitFlag = true;
  thread1.join();
 for(int i=0;i<num_cost_threads;i++)
 {
 	cost_threads_ptr[i].join();
 	delete []partial_grads_ptr[i];
 }
 for(int i=0;i<num_cnt_threads;i++)
 {
 	threads_ptr[i].join();
 	delete MoveitTools_array[i];
 }
 delete []partial_grads_ptr;
 
 delete []threads_ptr;
 delete []cost_threads_ptr;

  // delete arraies
  delete []u_ptr;
  delete []jnt_vals_ptr;
  delete []costCal_jnt_vals_ptr;
  delete []partial_costs_ptr;
  delete []finished_ptr;
  delete []cost_finished_ptr;
  std::cout << "iteration number: " << iter_num << std::endl;
  std::cout << "avg time: " << total_time/iter_num << std::endl;
  std::cout << "avg joint velocities: " << total_joint_velocities.sum()/(7*iter_num) << std::endl;
  std::cout << "norm trajectory length: " << total_traj_length/straight_length << std::endl;
  std::cout << "avg position squared error: " << total_diff_position/iter_num << std::endl;
  std::cout << "avg orientation squared error: " << total_diff_orientation/iter_num << std::endl;
  // Eigen::Vector3d diff_position = goal_position.head<3>() - prev_point_x_position;
  // double w1=curr_point_quat(0),w2=goal_position(3);
  // Eigen::Vector3d q1=curr_point_quat.tail<3>(),q2=goal_position.tail<3>();
  // Eigen::Matrix3d skew_q2;
  // skew_q2 << 0,-q2(2),q2(1),q2(2),0,-q2(0),-q2(1),q2(0),0;
  // Eigen::Vector3d diff_orientation = w1*q2-w2*q1-skew_q2*q1;
  // std::cout << "squared position error: " << diff_position.squaredNorm() << std::endl;
  // std::cout << "squared quaternion error: " << diff_orientation.squaredNorm() << std::endl;

  // end the data file and close the handle;
 fout1.close();
 fout2.close();
 fout3.close();
}
void NmpcNlopt::jointStateSubCB(const sensor_msgs::JointState::ConstPtr& msg)
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

void NmpcNlopt::goalSubCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg)
{
  // convert quaternion to rpy
//  tf::Matrix3x3 rot(tf::Quaternion(msg->pose.orientation.x,
//                                     msg->pose.orientation.y,
//                                     msg->pose.orientation.z,
//                                     msg->pose.orientation.w));
//  double roll,pitch,yaw;
//  rot.getRPY(roll,pitch,yaw);
//  goal1 << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z,
//                   roll,pitch,yaw;


  goal_position << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z,
                   msg->pose.orientation.w,msg->pose.orientation.x,
                   msg->pose.orientation.y,msg->pose.orientation.z;

  if(!goal_sub_started)
    goal_sub_started = true;


//   interactive marker is used for moving obstacle
//      obstacle_state.pose.position.x = msg->pose.position.x;
//      obstacle_state.pose.position.y = msg->pose.position.y;
//      obstacle_state.pose.position.z = msg->pose.position.z;
//      obstacle_state.pose.orientation.x = msg->pose.orientation.x;
//      obstacle_state.pose.orientation.y = msg->pose.orientation.y;
//      obstacle_state.pose.orientation.z = msg->pose.orientation.z;
//      obstacle_state.pose.orientation.w = msg->pose.orientation.w;
//      gazebo_model_pub.publish(obstacle_state);

}

void NmpcNlopt::octomap_sub_cb(const octomap_msgs::OctomapConstPtr octo_msg)
{
  ROS_INFO("octomap received.");
  const auto tree_ptr = octomap_msgs::binaryMsgToMap(*octo_msg);
  octree_shape = std::make_shared<shapes::OcTree>(std::shared_ptr<const octomap::OcTree>(
                                                    dynamic_cast<octomap::OcTree*>(tree_ptr)
                                                    ));
  Eigen::Affine3d octree_pose;
  octree_pose.setIdentity();
  addObstacle("shelf",octree_shape,octree_pose);
}


bool NmpcNlopt::track_toggle_cb(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
  track_mode = track_mode?false:true;
  return true;
}

void NmpcNlopt::addObstacle(const std::string &obj_name, const shapes::ShapeConstPtr &obj_shape, const Eigen::Affine3d &obj_transform)
{
  // rviz marker doesn't support octree, do not visualize octree objects
  if (obj_shape->type==shapes::OCTREE)
    MoveitTool::addObstacle(obj_name,obj_shape,obj_transform);
  else
    MoveitTool::addObstacle(obj_name,obj_shape,obj_transform,true);
 for(int i=0;i<num_cnt_threads;i++)
   MoveitTools_array[i]->addObstacle(obj_name,obj_shape,obj_transform);

  int m = ph*(3*getObstacleNum()+1);
//  int m = ph*(3*getObstacleNum());
  std::vector<double> tol(m,1e-2);
  optimizer.remove_inequality_constraints();
  optimizer.add_inequality_mconstraint(obstacleConstraintsMultiThread,this,tol);
  simple_optimizer.remove_inequality_constraints();
  simple_optimizer.add_inequality_mconstraint(obstacleConstraintsMultiThread,this,tol);

}

void NmpcNlopt::updateObstacle(const std::string& obj_name, const Eigen::Affine3d& obj_transform)
{
  MoveitTool::updateObstacle(obj_name,obj_transform,true);
  for(int i=0;i<num_cnt_threads;i++)
   MoveitTools_array[i]->updateObstacle(obj_name,obj_transform);
}



void NmpcNlopt::costCalThread(const int thread_index, NmpcNlopt *class_ptr, const double* u_ptr)
{
  std::cout << "Cost cal thread[" << thread_index+1 << "] started" << std::endl;
  while(ros::ok() && !(class_ptr->exitFlag))
  {
    if(class_ptr->cost_activated && !(class_ptr->cost_finished_ptr[thread_index]))
    {
//      std::cout << "thread[" << thread_index << "] cal started." << std::endl;
      // auto t1 = std::chrono::high_resolution_clock::now();
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
      Eigen::Matrix3d eef_rotm;
      Eigen::Vector3d eef_translation;
      Eigen::Vector4d eef_quat;

      int local_num_loops = (thread_index==(class_ptr->num_cost_threads-1))?class_ptr->last_num_cost_loops:class_ptr->num_cost_loops;
      for(int i=0;i<local_num_loops;i++)
      {
        int global_i = i + thread_index*class_ptr->num_cost_loops;
        for(int j =0;j<7;j++)
          curr_x[j] = class_ptr->costCal_jnt_vals_ptr[global_i][j];
//        class_ptr->jaco_hess_cal.getPosition(curr_x, eef_position);
//        Eigen::Vector3d diff_position = goal_position.head<3>() - eef_position.head<3>();
//        double w1=eef_position(3),w2=goal_position(3);
//        Eigen::Vector3d q1=eef_position.tail<3>(),q2=goal_position.tail<3>();
//        Eigen::Matrix3d skew_q2;
//        skew_q2 << 0,-q2(2),q2(1),q2(2),0,-q2(0),-q2(1),q2(0),0;
//        Eigen::Vector3d diff_orientation = w1*q2-w2*q1-skew_q2*q1;
//        diff_running << diff_position,diff_orientation;

        class_ptr->jaco_hess_cal.getEEFTransform(curr_x, eef_rotm, eef_translation);
        class_ptr->jaco_hess_cal.rotm2quat(eef_rotm,eef_quat);
        Eigen::Vector3d diff_position = goal_position.head<3>() - eef_translation;
        double w1=eef_quat(0),w2=goal_position(3);
        Eigen::Vector3d q1=eef_quat.tail<3>(),q2=goal_position.tail<3>();
        Eigen::Matrix3d skew_q2;
        skew_q2 << 0,-q2(2),q2(1),q2(2),0,-q2(0),-q2(1),q2(0),0;
        Eigen::Vector3d diff_orientation = w1*q2-w2*q1-skew_q2*q1;
        //Eigen::Vector3d diff_orientation = w2*q1-w1*q2-skew_q2*q1;  // accoring to paper, this one is correct
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

        // Eigen::Matrix<double,6,7> Ji = local_moveit->getJacobian(false);
        // Eigen::Matrix<double,4,3> quaternion_update_matrix;
        // quaternion_update_matrix << -x,-y,-z,w,z,-y,-z,w,x,y,-x,w;
        // Ji_quat.topRows<3>() = Ji.topRows<3>();
        // Ji_quat.bottomRows<4>() = 0.5*quaternion_update_matrix*Ji.bottomRows<3>();
        // Ji_quat.block<3,1>(0,6) << 0,0,0;
        // Ji_quat(2,0) = 0;

//        class_ptr->jaco_hess_cal.getAnalyticalJacobian(curr_x,Ji_quat);
        Eigen::Matrix<double,6,7> Ji;
        class_ptr->jaco_hess_cal.getAnalyticalJacobianOmega(curr_x, Ji);
        double w =eef_quat(0),x=eef_quat(1),y=eef_quat(2),z=eef_quat(3);
        Eigen::Matrix<double,4,3> quaternion_update_matrix;
        quaternion_update_matrix << -x,-y,-z,w,z,-y,-z,w,x,y,-x,w;
        // quaternion_update_matrix << -x,-y,-z,w,-z,y,z,w,-x,-y,x,w; // this shoud be right one according to paper
        Ji_quat.topRows<3>() = Ji.topRows<3>();
        Ji_quat.bottomRows<4>() = 0.5*quaternion_update_matrix*Ji.bottomRows<3>();

        Eigen::Matrix<double,1,7> dfidqi_pos = diffTQPf.leftCols<3>()*-2*Ji_quat.topRows<3>();
        Eigen::Matrix<double,3,4> quat_update;
        quat_update.col(0) << q2;
        quat_update.rightCols<3>() << -w2*Eigen::MatrixXd::Identity(3,3)-skew_q2;
        // quat_update.col(0) << -q2;
        // quat_update.rightCols<3>() << w2*Eigen::MatrixXd::Identity(3,3)-skew_q2; //this shoud be right one according to paper

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
      // auto t2 = std::chrono::high_resolution_clock::now();
      // class_ptr->time_consumed[thread_index] = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 );
//      std::cout << "thread["<<thread_index<<"] done." << std::endl;
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
// int& cal_cost_count = class_ptr->cal_cost_count;
// double& time_avg = class_ptr->time_avg;
//multi threading preparation
//  class_ptr->time_consumed.resize(class_ptr->num_cost_threads);

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
  for(int i=0;i<class_ptr->num_cost_threads;i++)
    class_ptr->cost_finished_ptr[i]=false;
  class_ptr->cost_activated = true;


  // wait until all threads calculation finished
  while(ros::ok())
  {
    bool all_threads_done = true;
    for(int i=0;i<class_ptr->num_cost_threads;i++)
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
  for(int i=0;i<class_ptr->num_cost_threads;i++)
    multi_cost += class_ptr->partial_costs_ptr[i];
  if(grad)
  {
    for(int j=0;j<ch*7;j++)
    {
      grad[j] = 0;
      for(int i=0;i<class_ptr->num_cost_threads;i++)
        grad[j] += class_ptr->partial_grads_ptr[i][j];
    }
  }
 // average time calculation
 //   time_avg = (duration_total+time_avg*cal_cost_count)/(cal_cost_count+1);
 //   cal_cost_count++;
 //   std::cout << time_avg << " | ";
 // }


//  std::cout << "----" << std::endl;
//  if(class_ptr->cal_cost_count++>1)
//  {
//    ros::shutdown();
//    std::this_thread::sleep_for(std::chrono::milliseconds(50));
//    exit(0);
//  }

  return multi_cost;
}
double NmpcNlopt::constantCostFunc(unsigned n, const double* u, double* grad, void *data)
{
  if(grad)
  {
    for(int i=0;i<n;i++)
      grad[i] = 0;
  }
  return 0;
}

double NmpcNlopt::costFunc(unsigned n, const double* u, double* grad, void *data)
{
//   auto t1 = std::chrono::high_resolution_clock::now();
    NmpcNlopt* class_ptr = reinterpret_cast<NmpcNlopt*>(data);
    Vector7d x_old = class_ptr->temp_joint_values;
    const Matrix6d& Q = class_ptr->Q;
    const Matrix6d& Pf = class_ptr->Pf;
    const Matrix7d& R = class_ptr->R;
    const double& Ts = class_ptr->Ts;
    const int& ph = class_ptr->ph;
    const int& ch = class_ptr->ch;
    // int& cal_cost_count = class_ptr->cal_cost_count;
    // double& time_avg = class_ptr->time_avg;

    // set grad to zero
    if(grad)
    {
      for(int i=0;i<n;i++)
        grad[i] = 0;
    }

    Vector7d x_new,u_eigen,eef_position;
    Vector6d diff_running;
    std::vector<double> x_new_vector(7);
    const Vector7d& goal_position = class_ptr->goal_position;
    double cost_running = 0;
    for(int i=0;i<ph;i++)
    {

      // std vector to eigen vector for u
      int u_index = std::min(i,ch-1);
//      u_eigen << u[6*u_index],u[6*u_index+1],u[6*u_index+2],u[6*u_index+3],u[6*u_index+4],u[6*u_index+5];
      u_eigen << u[7*u_index],u[7*u_index+1],u[7*u_index+2],u[7*u_index+3],u[7*u_index+4],u[7*u_index+5],u[7*u_index+6];
      // predict state in next time step given mvs(u) based on model
      x_new = x_old+u_eigen*Ts;
      x_old = x_new;
      // update joint values to the predicted new joint values
      // eigen vector to std vector for x_new
      for(int ii =0;ii<7;ii++)
        x_new_vector[ii] = x_new(ii);
      class_ptr->jaco_hess_cal.getPosition(x_new_vector, eef_position);

//      class_ptr->updateJointState(x_new_vector);
//      auto taskTemp = class_ptr->getEEFTransform();
//      // covert affine3d linear matrix to quaternion
//      Eigen::Quaterniond curr_quat(taskTemp.linear());
//      double w=curr_quat.w(),x=curr_quat.x(),y=curr_quat.y(),z=curr_quat.z();
//      eef_position << taskTemp.translation(),w,x,y,z;


      Eigen::Vector3d diff_position = goal_position.head<3>() - eef_position.head<3>();
      double w1=eef_position(3),w2=goal_position(3);
      Eigen::Vector3d q1=eef_position.tail<3>(),q2=goal_position.tail<3>();
      Eigen::Matrix3d skew_q2;
      skew_q2 << 0,-q2(2),q2(1),q2(2),0,-q2(0),-q2(1),q2(0),0;
      Eigen::Vector3d diff_orientation = w1*q2-w2*q1-skew_q2*q1;
      diff_running << diff_position,diff_orientation;


      Eigen::Matrix<double,1,6> diffTQPf;
      Matrix6d QPf;


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
      cost_running = cost_running + cost_running_temp + temp;
      // gradient
      if (!grad)
        continue;
      //       get current robot jacobian

      //      rpy representing orientation jacobian
//      Eigen::Matrix<float,6,7> Ji = class_ptr->getJacobian().cast<float>();
//        Eigen::Matrix3f B,B_inv;
//        B << 0, -sin(curr_yaw), cos(curr_yaw)*cos(curr_pitch),
//             0, cos(curr_yaw), sin(curr_yaw)*cos(curr_pitch),
//             1, 0, -sin(curr_pitch);
//        B_inv << (cos(y1)*sin(p1))/cos(p1), (sin(y1)*sin(p1))/cos(p1), 1,
//                  -sin(y1), cos(y1), 0,
//                  cos(y1)/cos(p1), sin(y1)/cos(p1), 0;
//        Eigen::Matrix<float,6,6> IB_inv = Eigen::Matrix<float,6,6>::Identity();
//        IB.bottomRightCorner<3,3>() = B;
//        Eigen::Matrix<float,6,7> Ji_rpy = IB_inv*Ji;

      //       quaternion representing orientation jacobian
      //       attention: calculation of quaternion based jacobian from moveit is wrong!!!!
      //       use moveit to calculate the original euler angle based jacobian, then manually update it to quaternion
      Eigen::Matrix<double,7,7> Ji_quat;
//      Eigen::Matrix<double,6,7> Ji = class_ptr->getJacobian(false);
//      Eigen::Matrix<double,4,3> quaternion_update_matrix;
//      quaternion_update_matrix << -x,-y,-z,w,z,-y,-z,w,x,y,-x,w;
//      Ji_quat.topRows<3>() = Ji.topRows<3>();
//      Ji_quat.bottomRows<4>() = 0.5*quaternion_update_matrix*Ji.bottomRows<3>();
//      Ji_quat.block<3,1>(0,6) << 0,0,0;
//      Ji_quat(2,0) = 0;
      class_ptr->jaco_hess_cal.getAnalyticalJacobian(x_new_vector,Ji_quat);




      //       z axis vector representing orientation jacobian
//      Eigen::Matrix<double,6,6> Ji_z = class_ptr->getJacobian().leftCols<6>();
//      Ji_z(2,0) = 0;
//      Ji_z(3,0) = 0;
//      Ji_z(4,0) = 0;
//      Ji_z(5,0) = -1;
//      Ji_z(5,1) = 0;
//      for(int i=0;i<6;i++)
//      {
//        Eigen::Matrix3d skew;
//        skew << 0,-Ji_z(5,i),Ji_z(4,i),Ji_z(5,i),0,-Ji_z(3,i),-Ji_z(4,i),Ji_z(3,i),0;
//        Ji_z.block<3,1>(3,i) = skew*curr_z;
//      }

      // ------------description for gradient of cast with respect to input u -----------------
      // past mv can only affect current cost by affecting past states
      // current mv can only affect current cost via u'Ru term in cost
      // future mvs dont have affect

      // qi = qi-1 + Ts*ui
      // future mvs can not affect past state and current state is affected by all mvs from past
      // dqiduj = Ts if j<=i
      // dqiduj = 0  if j>i
      // --------------------------------------------------------------------------------------
      Eigen::Matrix<double,1,7> dfidqi_pos = diffTQPf.leftCols<3>()*-2*Ji_quat.topRows<3>();
      Eigen::Matrix<double,3,4> quat_update;
      quat_update.col(0) = q2;
      quat_update.rightCols<3>() = -w2*Eigen::MatrixXd::Identity(3,3)-skew_q2;
      Eigen::Matrix<double,1,7> dfidqi_quat = diffTQPf.rightCols<3>()*2*quat_update*Ji_quat.bottomRows<4>();
      Eigen::Matrix<double,1,7> dfidqi = dfidqi_pos+dfidqi_quat;

       Eigen::Matrix<double,1,7> dfiduj_past = dfidqi*Ts;
       Eigen::Matrix<double,1,7> dfiduj_now = dfiduj_past+2*uTR;

//      Eigen::Matrix<double,1,7> dfidqi = diffTQPf*-2*Ji_quat;
//      Eigen::Matrix<double,1,7> dfiduj_past = dfidqi*Ts;
//      Eigen::Matrix<double,1,7> dfiduj_now = dfiduj_past+2*uTR;

      // gradient of u in the first (ch-1) time steps
      // ui will be only applied in the i-th time step
      for(int j=0;j<std::min(ch-1,i+1);j++)
      {
        if(j<i)
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

      // special processs of the last manipulated varibale(mv)
      if(i>=ch-1)
      {
        Eigen::Matrix<double,1,7> dfiduj = 2*uTR + dfidqi*Ts*(i-ch+2);
        for(int k=0;k<7;k++)
          grad[(ch-1)*7+k] += dfiduj(k);
      }
    }
   // if(grad)
   // {
   //   auto t2 = std::chrono::high_resolution_clock::now();
   //   auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
   //   // std::cout << duration << " | ";
   //   time_avg = (duration+time_avg*cal_cost_count)/(cal_cost_count+1);
   //   cal_cost_count++;
   //   std::cout << time_avg << " | ";
   // }
    return cost_running;
}

void NmpcNlopt::cntCalThread(const int thread_index, NmpcNlopt *class_ptr)
{
  std::cout << "Constraint cal thread[" << thread_index+1 << "] started" << std::endl;
  while(ros::ok() && !(class_ptr->exitFlag))
  {
    if(class_ptr->activated && !(class_ptr->finished_ptr[thread_index]))
    {
      // auto t1 = std::chrono::high_resolution_clock::now();
      //cost and grad calculation
      const double& Ts = class_ptr->Ts;
      const int& ch = class_ptr->ch;
      const double safe_distance = 0.02;
      const int& num_obstacles = class_ptr->getObstacleNum();
      const int num_links = 3;
      const int num_dists = num_links*num_obstacles+1;
      const int n = 7*ch;

      // output partial cost value container
      std::vector<double>& cnt_vals = class_ptr->partial_cnt_val[thread_index];
      // output partial cost gradient container
      std::vector<double>& cnt_grads = class_ptr->partial_cnt_grads[thread_index];

      std::vector<double> curr_x(7);
      MoveitTool* local_moveit = class_ptr->MoveitTools_array[thread_index];

      int local_num_cnt_loops = (thread_index==(class_ptr->num_cnt_threads-1))?class_ptr->last_num_cnt_loops:class_ptr->num_cnt_loops;
      for(int i=0;i<local_num_cnt_loops;i++)
      {
        int global_i = i + thread_index*class_ptr->num_cnt_loops;
        for(int j =0;j<7;j++)
          curr_x[j] = class_ptr->jnt_vals_ptr[global_i][j];
        local_moveit->updateJointState(curr_x, true);
        std::vector< std::vector<double> > dists;
        std::vector<std::vector<Eigen::Vector3d> > points1, points2;
        local_moveit->getDistsAndPoints(dists,points1,points2);
        for(int j=0;j<num_obstacles;j++)
        {
          for(int k=0;k<num_links;k++)
          {
            cnt_vals[global_i*num_dists+j*num_links+k] = -dists[j][k] + safe_distance;
            if(class_ptr->calculate_grad)
            {
              Eigen::Matrix<double,3,7> J_link;
              class_ptr->jaco_hess_cal.getAnalyJaco4Ref(k,curr_x, J_link, points1[j][k]);
              Eigen::Vector3d normal = points1[j][k] - points2[j][k];
              normal = normal/normal.norm();
              if(dists[j][k]<0)
                normal = -normal;
              // normal = normal/dists[j][k] 
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
        // distance between cylinder 1 and 3
        Eigen::Vector3d cylinder1_point, cylinder3_point;
        double dist13 = local_moveit->selfCollisionDistAndPoints(cylinder1_point,cylinder3_point);
        cnt_vals[global_i*num_dists+num_obstacles*num_links] = -dist13 + safe_distance;

        if(class_ptr->calculate_grad)
        {
          Eigen::Matrix<double,3,7> J_link1,J_link3;
          class_ptr->jaco_hess_cal.getAnalyJaco4Ref(0,curr_x, J_link1, cylinder1_point);
          class_ptr->jaco_hess_cal.getAnalyJaco4Ref(2,curr_x, J_link3, cylinder1_point);
          Eigen::Vector3d normal = cylinder1_point - cylinder3_point;
          normal = normal/normal.norm();
          if(dist13<0)
            normal = -normal;
          Eigen::Matrix<double,1,7> dc_ijk_dq_i = normal.transpose() * (J_link3-J_link1);
          Eigen::Matrix<double,1,7> dc_ijk_du_n = dc_ijk_dq_i*Ts;

          for(int l=0;l<std::min(ch-1,global_i+1);l++)
          {
            for(int m1=0;m1<7;m1++)
            {
              cnt_grads[global_i*num_dists*n+num_obstacles*num_links*n+l*7+m1]=dc_ijk_du_n(m1);
            }
          }
          if(global_i>=ch-1)
          {
            Eigen::Matrix<double,1,7> dc_ijk_du_last = dc_ijk_dq_i*Ts*(global_i-ch+2);
            for(int m1=0;m1<7;m1++)
            {
             cnt_grads[global_i*num_dists*n+num_obstacles*num_links*n+(ch-1)*7+m1]=dc_ijk_du_last(m1);
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
  NmpcNlopt* class_ptr = reinterpret_cast<NmpcNlopt*>(data);
  const double& Ts = class_ptr->Ts;
  const int& ph = class_ptr->ph;
  const int& ch = class_ptr->ch;
  int& cal_cost_count = class_ptr->cal_cost_count;
  double& time_avg = class_ptr->time_avg;


  if(grad)
  {
    for(int i=0;i<m*n;i++)
      grad[i]=0;
    class_ptr->calculate_grad = true;
    for(int i=0;i<class_ptr->num_cnt_threads;i++)
      class_ptr->partial_cnt_grads[i].resize(m*n);
  }
  else
    class_ptr->calculate_grad = false;
  for(int i=0;i<class_ptr->num_cnt_threads;i++)
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
  for(int i=0;i<class_ptr->num_cnt_threads;i++)
    class_ptr->finished_ptr[i]=false;
  while(ros::ok())
  {
    bool all_threads_done = true;
    for(int i=0;i<class_ptr->num_cnt_threads;i++)
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
    for(int j=0;j<class_ptr->num_cnt_threads;j++)
      result[i]+=class_ptr->partial_cnt_val[j][i];
  }
  if(grad)
  {
    for(int i=0;i<m*n;i++)
    {
      grad[i]=0;
      for(int j=0;j<class_ptr->num_cnt_threads;j++)
      {
        grad[i] += class_ptr->partial_cnt_grads[j][i];
      }
    }
  }

  // if(grad)
  // {
    // auto t2 = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    // std::cout << duration << std::endl;
    // if(cal_cost_count++ > 1)
      // {
  //       ros::shutdown();
  //       std::this_thread::sleep_for(std::chrono::milliseconds(10));
  //       exit(0);
  //     }
  // }

  // if(grad)
  // {
  // auto t2 = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  // std::cout << duration << "|";
  // time_avg = (duration+time_avg*cal_cost_count)/(cal_cost_count+1);
  // cal_cost_count++;
  // std::cout << time_avg << " | ";
  // }
  return;
}


void NmpcNlopt::obstacleConstraints(unsigned m, double *result, unsigned n, const double *u, double *grad, void *data)
{
  // auto t1 = std::chrono::high_resolution_clock::now();
  NmpcNlopt* class_ptr = reinterpret_cast<NmpcNlopt*>(data);
  Vector7d x_old = class_ptr->temp_joint_values;
  const double& Ts = class_ptr->Ts;
  const int& ph = class_ptr->ph;
  const int& ch = class_ptr->ch;
  const double safe_distance = 0.02;
  const int& num_obstacles = class_ptr->getObstacleNum();
  const int num_links = 3;
  const int num_dists = num_links*num_obstacles;

  // set grad to zero
  if(grad)
  {
    for(int i=0;i<m*n;i++)
      grad[i]=0;
  }

  Vector7d x_new,u_eigen;
  std::vector<double> x_new_vector(7);
  for(int i=0;i<ph;i++)
  {
    int u_index = std::min(i,ch-1);
    u_eigen << u[7*u_index],u[7*u_index+1],u[7*u_index+2],u[7*u_index+3],u[7*u_index+4],u[7*u_index+5],u[7*u_index+6];
    x_new = x_old + u_eigen*Ts;
    x_old = x_new;
    // update joint values to the predicted new joint values
    // eigen vector to std vector for x_new
    for(int ii =0;ii<7;ii++)
      x_new_vector[ii] = x_new(ii);
    class_ptr->updateJointState(x_new_vector,true);

    std::vector< std::vector<double> > dists;
    std::vector<std::vector<Eigen::Vector3d> > points1;
    std::vector<std::vector<Eigen::Vector3d> > points2;
    class_ptr->getDistsAndPoints(dists,points1,points2);
    // calculate distances and jacobians for different nearest points
    for(int j=0;j<num_obstacles;j++)
      for(int k=0;k<num_links;k++)
      {
        result[i*num_dists+j*num_links+k] = -dists[j][k] + safe_distance;
        if(grad)
        {
          // calculate gradients
          // 1.jacobian for the nearest point to j-th obstacle on k-th robot link
          Eigen::Matrix<double,3,7> J_link;
          // Eigen::MatrixXd J_link;
          // auto t3 = std::chrono::high_resolution_clock::now();
          // class_ptr->getjacobian(k,J_link,points1[j][k]);
          class_ptr->jaco_hess_cal.getAnalyJaco4Ref(k,x_new_vector, J_link, points1[j][k]);
          // auto t4 = std::chrono::high_resolution_clock::now();
          // auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>( t4 - t3 ).count();
          // std::cout <<  duration1 <<"|";


          // 2. constraint values derivative wrt current joint values q
          Eigen::Vector3d normal = points1[j][k] - points2[j][k];
          normal = normal/normal.norm();

          if(dists[j][k]<0)
            normal = -normal;
          // Eigen::Matrix<double,1,7> dc_ijk_dq_i = -normal.transpose() * J_link.topRows<3>();
         Eigen::Matrix<double,1,7> dc_ijk_dq_i = -normal.transpose() * J_link;

          // 3. joint values derivative wrt to mvs
          // c_ijk, in i-th time step, constraint for distances between j-th obstacle and k-th robor link
          // u_n, mv in n-th time step
          // u_last, mv in (ch-1)-th time step, will be applied in ch-1,ch,...,ph-1 -th time steps
          Eigen::Matrix<double,1,7> dc_ijk_du_n = dc_ijk_dq_i*Ts;
          for(int l=0;l<std::min(ch-1,i+1);l++)
          {
            for(int m1=0;m1<7;m1++)
            {
              grad[i*num_dists*n+j*num_links*n+k*n+l*7+m1]=dc_ijk_du_n(m1);
            }
          }
          if(i>=ch-1)
          {
            Eigen::Matrix<double,1,7> dc_ijk_du_last = dc_ijk_dq_i*Ts*(i-ch+2);
            for(int m1=0;m1<7;m1++)
            {
             grad[i*num_dists*n+j*num_links*n+k*n+(ch-1)*7+m1]=dc_ijk_du_last(m1);
            }
          }
        }
      }
  }
//  if(grad)
//  {
//    auto t2 = std::chrono::high_resolution_clock::now();
//    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
//    std::cout << duration << " | ";
//  }
}


void NmpcNlopt::optimize(std::vector<double>& u, double mincost)
{
//auto t1 = std::chrono::high_resolution_clock::now();
  try{
      nlopt::result result = optimizer.optimize(u,mincost);
  }
  catch(std::exception &e){
    std::cout << "nlopt failed: " << e.what() << std::endl;
  }
//  auto t2 = std::chrono::high_resolution_clock::now();
//  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( t2 - t1 ).count();
//  iter_num++;
//  total_time += duration;

//  average_time +=duration/iter_num;
//fout1 << duration << ",";
  // discard mvs in [1,ch-1]-th time step
  // only send the mv in the frist time step: 0th time step

//  std::vector<double> goal_q(7);
  Vector7d curr_u;
  pthread_mutex_lock(&joint_velocities_mutex);
  for(int i=0;i<7;i++)
  {
    joint_velocities[i] = u[i];
    curr_u(i)=u[i];
  }
  pthread_mutex_unlock(&joint_velocities_mutex);
  total_joint_velocities += curr_u.cwiseAbs();
  new_goal_got = true;

  std::vector<double> traj_point_q(7);
  for(int i=0;i<7;i++)
    traj_point_q[i] = temp_joint_values(i);
  updateJointState(traj_point_q,true);

  geometry_msgs::Point traj_point_x;
  const Eigen::Affine3d& point_x_tf = getEEFTransform();
  Eigen::Vector3d point_x_position = point_x_tf.translation();
  jaco_hess_cal.rotm2quat(point_x_tf.linear(),curr_point_quat);

  //calculate and  store the actual stage cost
//  const Eigen::Quaterniond point_x_quat(point_x_tf.linear());
//  Eigen::Vector3d diff_position = goal_position.head<3>() - point_x_position;
//  double w1=point_x_quat.w(),w2=goal_position(3);
//  Eigen::Vector3d q1;
//  q1 << point_x_quat.x(),point_x_quat.y(),point_x_quat.z();
//  Eigen::Vector3d q2=goal_position.tail<3>();
//  Eigen::Matrix3d skew_q2;
//  skew_q2 << 0,-q2(2),q2(1),q2(2),0,-q2(0),-q2(1),q2(0),0;
//  Eigen::Vector3d diff_orientation = w1*q2-w2*q1-skew_q2*q1;

//  Vector6d diff_running;
//  diff_running << diff_position,diff_orientation;

//  double cost1 = diff_running.transpose()*Q*diff_running;
//  double cost2 = curr_u.transpose()*R*curr_u;
//  double actual_stage_cost = cost1+cost2;
//  fout1 << actual_stage_cost << std::endl;

//  traj_point_x.x = point_x_position(0);
//  traj_point_x.y = point_x_position(1);
//  traj_point_x.z = point_x_position(2);
//  actual_traj_strip.points.push_back(traj_point_x);
//  actual_traj_strip.color.r = 0.6;
//  actual_traj_strip.color.g = 0.0;
//  actual_traj_strip.color.b = 1.0;
//  actual_traj_strip.id = 3;
//  actual_traj_strip.type = visualization_msgs::Marker::LINE_STRIP;
//  actual_traj_strip.ns = "actual_trajectory";
//  marker_publisher.publish(actual_traj_strip);

//  actual_traj_strip.color.r = 1.0;
//  actual_traj_strip.color.g = 0.0;
//  actual_traj_strip.color.b = 0.0;
//  actual_traj_strip.id = 4;
//  actual_traj_strip.type = visualization_msgs::Marker::POINTS;
//  actual_traj_strip.ns = "actual_trajectory_point";
//  marker_publisher.publish(actual_traj_strip);

////   recored distances to the obstacle
//    std::vector< std::vector<double> > dists;
//    std::vector<std::vector<Eigen::Vector3d> > points1;
//    std::vector<std::vector<Eigen::Vector3d> > points2;
//    getDistsAndPoints(dists,points1,points2);
//    for(auto j:dists)
//      for(auto k:j)
//        fout3 << k << ",";
//    fout3 << std::endl;


  if(init_prediction)
  {
    init_prediction=false;
    line_strip.color.g = 0.0;
    line_strip.color.b = 1.0;
    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.ns = "predict_trajectory";
    line_strip.points.clear();
    line_strip.points.resize(ph+1);


    // Eigen::Vector3d target_position;
    // target_position << goal_position(0),goal_position(1),goal_position(2);
    // straight_length = (target_position-point_x_position).norm();
    traj_point_x.x = point_x_position(0);
    traj_point_x.y = point_x_position(1);
    traj_point_x.z = point_x_position(2);
    line_strip.points[0] = traj_point_x;

    for(int i=0;i<ph;i++)
    {
      int index = std::min(i,ch-1);
      for(int j=0;j<7;j++)
      {
        traj_point_q[j] += u[index*7+j]*Ts;
        curr_u(j)=u[(std::min(i+1,ch-1))*7+j];
      }
      updateJointState(traj_point_q);
      const Eigen::Affine3d& point_x_tf1 = getEEFTransform();
      point_x_position = point_x_tf1.translation();
      Eigen::Vector3d point_x_position_new = point_x_tf1.translation();
//      length += (point_x_position_new-point_x_position).norm();
      point_x_position = point_x_position_new;

      //calculate and  store the predicted stage cost
//      const Eigen::Quaterniond point_x_quat1(point_x_tf1.linear());
//      diff_position = goal_position.head<3>() - point_x_position;
//      w1=point_x_quat1.w();
//      w2=goal_position(3);
//      q1<<point_x_quat1.x(),point_x_quat1.y(),point_x_quat1.z();
//      q2=goal_position.tail<3>();
//      skew_q2 << 0,-q2(2),q2(1),q2(2),0,-q2(0),-q2(1),q2(0),0;
//      diff_orientation = w1*q2-w2*q1-skew_q2*q1;
//      diff_running << diff_position,diff_orientation;
//      cost1= diff_running.transpose()*Q*diff_running;
//      cost2= curr_u.transpose()*R*curr_u;
//      double predict_stage_cost = cost1+cost2;
//      fout2 << predict_stage_cost << std::endl;
      traj_point_x.x = point_x_position(0);
      traj_point_x.y = point_x_position(1);
      traj_point_x.z = point_x_position(2);
      line_strip.points[i+1] = traj_point_x;
    }
   // double norm_length = length/straight_length;
  //  average_norm_length += norm_length/iter_num;
    marker_publisher.publish(line_strip);

    line_strip.color.g = 1.0;
    line_strip.color.b = 0.0;
    line_strip.id = 2;
    line_strip.type = visualization_msgs::Marker::POINTS;
    line_strip.ns = "predict_trajectory_point";
    marker_publisher.publish(line_strip);
   }
  // else
  //   total_traj_length += (point_x_position-prev_point_x_position).norm();
  // prev_point_x_position = point_x_position;
}

void NmpcNlopt::velocitiesSend_thread(ros::Publisher publisher, bool* exitFlag,
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

      pthread_mutex_lock(position_goal_mutex);
      for(int i=0;i<7;i++)
        (*position_goal_ptr)[i] += (temp_v[i])*0.01;
      pthread_mutex_unlock(position_goal_mutex);

      msg_point.positions = *position_goal_ptr;
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
  cylinders_msg.header.frame_id="world";
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
  cylinders_pos_pub.publish(cylinders_msg);
  cylinders_sent = true;
  ROS_INFO("Cylinders sent out");
  return;
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
  updateJointState(curr_joint_values,true);
  pthread_mutex_unlock(&position_goal_mutex);

  // update the  dynamic obstacle
   obstacle1_y_p += obstacle1_y_v;
   if(obstacle1_y_p>1.2||obstacle1_y_p<0.5)
     obstacle1_y_v = -obstacle1_y_v;
   Eigen::Affine3d obj_pose = Eigen::Translation3d(-0.3,obstacle1_y_p,0.5)*Eigen::Quaterniond(1,0,0,0);
   updateObstacle("obstacle1",obj_pose);


  if(!cylinders_sent)
   publish_cylinders();


//  if(goal_sub_started)
 if(track_mode)
  { 
    optimize(u,mincost);
//     warm start
    for(int i=0;i<(ch-1);i++)
     for(int j=0;j<7;j++)
     {
       u[i*7+j] = u[(i+1)*7+j];
     }
    for(int j=0;j<7;j++)
      u[(ch-1)*7+j]=0;
  }
}
