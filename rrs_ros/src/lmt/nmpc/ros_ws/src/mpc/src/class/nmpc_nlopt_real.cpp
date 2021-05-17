#include <nmpc_nlopt_real.h>
#include <chrono>
NmpcNlopt::NmpcNlopt(ros::NodeHandle nh):MoveitTool(nh)
{
  ROS_WARN("Unity Version Loading...");
  // load parameters
  bool param_ok=true;
  double Q1v,Q2v,Pf1v,Pf2v,Rv;
  if(!nh.getParam("main_real_node/nlmpc/cost_weight/Q1v",Q1v))
  {
    ROS_ERROR("error loading  Q1v param");
     param_ok = false;
  }
  if(!nh.getParam("main_real_node/nlmpc/cost_weight/Q2v",Q2v))
  {
    ROS_ERROR("error loading  Q2v param");
     param_ok = false;
  }
  if(!nh.getParam("main_real_node/nlmpc/cost_weight/Pf1v",Pf1v))
  {
    ROS_ERROR("error loading  Pf1v param");
     param_ok = false;
  }
  if(!nh.getParam("main_real_node/nlmpc/cost_weight/Pf2v",Pf2v))
  {
    ROS_ERROR("error loading  Pf2v param");
     param_ok = false;
  }
  if(!nh.getParam("main_real_node/nlmpc/cost_weight/Rv",Rv))
  {
    ROS_ERROR("error loading  Rv param");
     param_ok = false;
  }
  if(!nh.getParam("main_real_node/nlmpc/ph",ph))
  {
    ROS_ERROR("error loading  prediction horizon param");
     param_ok = false;
  }
  if(!nh.getParam("main_real_node/nlmpc/ch",ch))
  {
    ROS_ERROR("error loading  control horizon param");
     param_ok = false;
  }

  int maxeval;
  if(!nh.getParam("main_real_node/nlmpc/max_eval_num",maxeval))
  {
    ROS_ERROR("error loading  max_eval_num");
     param_ok = false;
  }
  if(!nh.getParam("main_real_node/box_size",box_size))
  {
    ROS_ERROR("error loading  box_size");
     param_ok = false;
  }

  if(!nh.getParam("main_real_node/robot_v",robot_v))
  {
    ROS_ERROR("error loading  robot_v");
     param_ok = false;
  }

  if(!nh.getParam("main_real_node/obstacle_v",obstacle_v))
  {
    ROS_ERROR("error loading  obstacle_v");
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

  joint_state_sub = nh.subscribe("/j2s7s300_driver/out/joint_state", 100, &NmpcNlopt::jointStateSubCB,this);
  obstacle_sub1 = nh.subscribe("/nmpc_controller/in/obstacle1", 100, &NmpcNlopt::obstacle1_sub_cb,this);
  // obstacle_sub2 = nh.subscribe("/nmpc_controller/in/obstacle2", 100, &NmpcNlopt::obstacle2_sub_cb,this);

  // initialize the obstacle shape
  obstacle1_shape = std::make_shared<shapes::Box>(0.1,0.1,0.1);
  obstacle2_shape = std::make_shared<shapes::Sphere>(0.1);

  base2vive_offset << -0.478224396706, 1.01738905907, -0.773618030548; 


  state_sub_started = false;
  goal_sub_started = false;
  // goal_sub = nh.subscribe("/simple_marker/feedback",100,&NmpcNlopt::goalSubCB,this);
  goal_sub_from_outside = nh.subscribe("/nmpc_controller/in/goal",100,&NmpcNlopt::goalSubFromOutsideCB,this);



  // publisher to velocity controller on robot
  velocity_pub = nh.advertise<kinova_msgs::JointVelocity>("/j2s7s300_driver/in/joint_velocity",1);
  eef_pose_pub = nh.advertise<geometry_msgs::Pose>("/nmpc_controller/out/eef_pose",1);


  // server for toggle tracking mode
  track_toggle_server = nh.advertiseService("/j2s7s300/track_toggle",&NmpcNlopt::track_toggle_cb,this);
  track_mode = false;

  Ts = 0.2;
  interval = ros::Duration(Ts);

  joint_velocities_mutex = PTHREAD_MUTEX_INITIALIZER;

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
  // start the thread
  thread1 = std::thread(velocitiesSend_thread, velocity_pub, &exitFlag, &joint_velocities, &joint_velocities_mutex,&new_goal_got);
  // start cost val cal thread and initiliaze shared in-output container
  num_threads = 4;
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

 cal_cost_count = 0;
 time_avg = 0;

  // initialize marker for visualizing predicted trajectory
  line_strip.scale.x = 5e-3;
  line_strip.scale.y = 5e-3;
  line_strip.scale.z = 5e-3;

  // init tf for goal from outside
  transformStamped_goal.header.frame_id = "world";
  transformStamped_goal.child_frame_id = "goal_from_outside";


  // initialize tf listener
  tfListener =  std::make_shared<tf2_ros::TransformListener>(tfBuffer);
  // update tf between arm base frame and global frame first
  ROS_INFO("Wait 1 second for tf coming in");
  ros::Duration(1).sleep();
  geometry_msgs::TransformStamped transformStamped;
  try{
  transformStamped = tfBuffer.lookupTransform("world", "origin",
                           ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
  ROS_WARN("%s",ex.what());
  return;
  }
  tf::transformMsgToEigen(transformStamped.transform, origin2base);
  try{
  transformStamped = tfBuffer.lookupTransform("world", "j2s7s300_end_effector",
                           ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
  ROS_WARN("%s",ex.what());
  return;
  }
  
  goal_position << transformStamped.transform.translation.x,
                   transformStamped.transform.translation.y,
                   transformStamped.transform.translation.z,
                   transformStamped.transform.rotation.w,
                   transformStamped.transform.rotation.x,
                   transformStamped.transform.rotation.y,
                   transformStamped.transform.rotation.z;
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

void NmpcNlopt::obstacle1_sub_cb(const geometry_msgs::Pose::ConstPtr& msg)
{
	Eigen::Vector3d position2vive(msg->position.x,msg->position.y,msg->position.z);
	Eigen::Vector3d position2base = position2vive - base2vive_offset;
	Eigen::Translation3d translation(position2base(0),-position2base(1),position2base(2));

	Eigen::Quaterniond quater(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);
	Eigen::Affine3d obj_pose = translation*quater.toRotationMatrix();
	if(!co_ptrs.count("obstacle1"))
      addObstacle("obstacle1", obstacle1_shape, obj_pose);
	else if((obj_pose.matrix()-co_ptrs["obstacle1"]->getTransform().matrix()).norm() > 1e-4)
    updateObstacle("obstacle1",obj_pose);
}

// void NmpcNlopt::obstacle2_sub_cb(const geometry_msgs::Pose::ConstPtr& msg)
// {
// 	Eigen::Vector3d position2vive(msg->position.x,msg->position.y,msg->position.z);
// 	Eigen::Vector3d position2base = position2vive - base2vive_offset;
// 	Eigen::Translation3d translation(position2base(0),-position2base(1),position2base(2));

// 	Eigen::Quaterniond quater(msg->orientation.w,msg->orientation.x,msg->orientation.y,msg->orientation.z);
// 	Eigen::Affine3d obj_pose = translation*quater.toRotationMatrix();
// 	if(!co_ptrs.count("obstacle2"))
//       addObstacle("obstacle2", obstacle2_shape, obj_pose);
// 	else if((obj_pose.matrix()-co_ptrs["obstacle2"]->getTransform().matrix()).norm() > 1e-4)
// 	  updateObstacle("obstacle2",obj_pose);
// }


void NmpcNlopt::goalSubCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg)
{

   // Eigen::Affine3d  goal2origin, goal2base;
   // goal2origin = Eigen::Translation3d(msg->pose.position.x,
   //                                    msg->pose.position.y,
   //                                    msg->pose.position.z)*
   //               Eigen::Quaterniond(msg->pose.orientation.w,
   //                                  msg->pose.orientation.x,
   //                                  msg->pose.orientation.y,
   //                                  msg->pose.orientation.z);

   // goal2base = origin2base*goal2origin;
   // Eigen::Quaterniond goal2base_quat(goal2base.rotation());
   // goal_position << goal2base.translation()[0],goal2base.translation()[1],goal2base.translation()[2],
   // 					goal2base_quat.w(),goal2base_quat.x(),goal2base_quat.y(),goal2base_quat.z();
  goal_position << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z,
                   msg->pose.orientation.w,msg->pose.orientation.x,
                   msg->pose.orientation.y,msg->pose.orientation.z;

  if(!goal_sub_started)
    goal_sub_started = true;
}

void NmpcNlopt::goalSubFromOutsideCB(const geometry_msgs::PoseConstPtr &msg)
{
	goal_position << msg->position.x,msg->position.y,msg->position.z,
                   msg->orientation.w,msg->orientation.x,
                   msg->orientation.y,msg->orientation.z;
	// std::cout << msg->position.x << "," << msg->position.y << "," << msg->position.z << ",";
	// std::cout << msg->orientation.x << ", " << msg->orientation.y << ", " <<msg->orientation.z << ", " <<msg->orientation.w << std::endl;
    transformStamped_goal.header.stamp = ros::Time::now();
    transformStamped_goal.transform.translation.x = msg->position.x;
    transformStamped_goal.transform.translation.y = msg->position.y;
    transformStamped_goal.transform.translation.z = msg->position.z;
    transformStamped_goal.transform.rotation.x = msg->orientation.x;
    transformStamped_goal.transform.rotation.y = msg->orientation.y;
    transformStamped_goal.transform.rotation.z = msg->orientation.z;
    transformStamped_goal.transform.rotation.w = msg->orientation.w;
    br.sendTransform(transformStamped_goal);

    if(!goal_sub_started)
    	goal_sub_started = true;
}



bool NmpcNlopt::track_toggle_cb(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
  track_mode = track_mode?false:true;
  return true;
}


void NmpcNlopt::addObstacle(const std::string &obj_name, const shapes::ShapeConstPtr &obj_shape, const Eigen::Affine3d &obj_transform)
{

  // transfer global coordinate of the obstacle to local coordinate
  // Eigen::Affine3d local_obj_transform = origin2base * obj_transform;
  Eigen::Affine3d local_obj_transform = obj_transform;
  MoveitTool::addObstacle(obj_name,obj_shape,local_obj_transform,true);
  for(int i=0;i<num_threads;i++)
   MoveitTools_array[i]->addObstacle(obj_name,obj_shape,local_obj_transform);
  
  int m = ph*3*getObstacleNum();
  std::vector<double> tol(m,1e-3);
  optimizer.remove_inequality_constraints();
  optimizer.add_inequality_mconstraint(obstacleConstraintsMultiThread,this,tol);
}

void NmpcNlopt::updateObstacle(const std::string& obj_name, const Eigen::Affine3d& obj_transform)
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


double NmpcNlopt::costFunc(unsigned n, const double* u, double* grad, void *data)
{
    double cost_running = 0;

    // set grad to zero
    if(grad)
    {
      for(int i=0;i<n;i++)
        grad[i] = 0;
    }

    NmpcNlopt* class_ptr = reinterpret_cast<NmpcNlopt*>(data);
    Vector7d x_old = class_ptr->temp_joint_values;
    const Matrix6d& Q = class_ptr->Q;
    const Matrix6d& Pf = class_ptr->Pf;
    const Matrix7d& R = class_ptr->R;
    const double Ts = class_ptr->Ts;
    const int ph = class_ptr->ph;
    const int ch = class_ptr->ch;
    const Vector7d& goal_position = class_ptr->goal_position;

    Vector7d x_new,u_eigen,eef_position;
    Vector6d diff_running;
    std::vector<double> x_new_vector(7);

    for(int i=0;i<ph;i++)
    {
      // std vector to eigen vector for u
      int u_index = std::min(i,ch-1);
      u_eigen << u[7*u_index],u[7*u_index+1],u[7*u_index+2],u[7*u_index+3],u[7*u_index+4],u[7*u_index+5],u[7*u_index+6];
      // predict state in next time step given mvs(u) based on model
      x_new = x_old+u_eigen*Ts;

      x_old = x_new;
      // update joint values to the predicted new joint values
      // eigen vector to std vector for x_new
      for(int ii =0;ii<7;ii++)
        x_new_vector[ii] = x_new(ii);
      class_ptr->jaco_hess_cal.getPosition(x_new_vector, eef_position);

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
      Eigen::Matrix<double,7,7> Ji_quat;
      class_ptr->jaco_hess_cal.getAnalyticalJacobian(x_new_vector,Ji_quat);

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
    return cost_running;
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
//  class_ptr->time_consumed.resize(class_ptr->num_threads);
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
      // auto t1 = std::chrono::high_resolution_clock::now();
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

      // auto t2 = std::chrono::high_resolution_clock::now();
      // auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
      // std::cout << "preparation time: " << duration1 << std::endl;


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
        // auto t4 = std::chrono::high_resolution_clock::now();
        // auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>( t4 - t3 ).count();
        // std::cout << "outer loop: " << duration2 << std::endl;
        for(int j=0;j<num_obstacles;j++)
          for(int k=0;k<num_links;k++)
          {
          	auto t5 = std::chrono::high_resolution_clock::now();
            cnt_vals[global_i*num_dists+j*num_links+k] = -dists[j][k] + safe_distance;
            // std::cout << cnt_vals[global_i*num_dists+j*num_links+k] << std::endl;
            // std::cout << class_ptr->partial_cnt_val[thread_index][global_i*num_dists+j*num_links+k]  << std::endl;
            // std::cout << "----" << std::endl;
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
           //    auto t6 = std::chrono::high_resolution_clock::now();
	          // auto duration3 = std::chrono::duration_cast<std::chrono::microseconds>( t6 - t5 ).count();
	          // std::cout << "inner loop: " << duration3 << std::endl;

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
  int& cal_cost_count = class_ptr->cal_cost_count;
  double& time_avg = class_ptr->time_avg;


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


  // if(grad)
  // {
	  // auto t2 = std::chrono::high_resolution_clock::now();
	  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
	  // // std::cout << duration << "|";
	  // time_avg = (duration+time_avg*cal_cost_count)/(cal_cost_count+1);
   //    cal_cost_count++;
   //    std::cout << time_avg << " | ";
  // }
  return;
}

void NmpcNlopt::obstacleConstraints(unsigned m, double *result, unsigned n, const double *u, double *grad, void *data)
{
  using namespace std;
  // set grad to zero
  if(grad)
  {
    for(int i=0;i<m*n;i++)
      grad[i]=0;
  }

  NmpcNlopt* class_ptr = reinterpret_cast<NmpcNlopt*>(data);
  Vector7d x_old = class_ptr->temp_joint_values;
  double Ts = class_ptr->Ts;
  int ph = class_ptr->ph;
  int ch = class_ptr->ch;
  double safe_distance = 0.02;
  int num_obstacles = class_ptr->getObstacleNum();
  int num_links = 3;
  int num_dists = num_links*num_obstacles;

  Vector7d x_new,u_eigen;
  std::vector<double> x_new_vector(7);
//  x_new_vector[6] = class_ptr->temp_joint_values

  for(int i=0;i<ph;i++)
  {
    int u_index = std::min(i,ch-1);
//    u_eigen << u[6*u_index],u[6*u_index+1],u[6*u_index+2],u[6*u_index+3],u[6*u_index+4],u[6*u_index+5];
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
//          Eigen::Matrix<double,3,7> J_link;
//          Eigen::MatrixXd J_link;
//          auto t3 = std::chrono::high_resolution_clock::now();
//          class_ptr->getjacobian(k,J_link,points1[j][k]);
//          class_ptr->jaco_hess_cal.getAnalyJaco4Ref(k,x_new_vector, J_link, points1[j][k]);
//          auto t4 = std::chrono::high_resolution_clock::now();
//          auto duration1 = std::chrono::duration_cast<std::chrono::microseconds>( t4 - t3 ).count();
//          std::cout <<  duration1 <<"|";

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
            for(int m=0;m<7;m++)
            {
              grad[i*num_dists*n+j*num_links*n+k*n+l*7+m]=dc_ijk_du_n(m);
            }
          }
          if(i>=ch-1)
          {
            Eigen::Matrix<double,1,7> dc_ijk_du_last = dc_ijk_dq_i*Ts*(i-ch+2);
            for(int m=0;m<7;m++)
            {
             grad[i*num_dists*n+j*num_links*n+k*n+(ch-1)*7+m]=dc_ijk_du_last(m);
            }
          }
        }
      }
  }
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
  // std::cout << std::endl;
//  average_time +=duration/iter_num;



  // discard mvs in [1,ch-1]-th time step
  // only send the mv in the frist time step: 0th time step

//  std::vector<double> goal_q(7);
  pthread_mutex_lock(&joint_velocities_mutex);
  for(int i=0;i<7;i++)
  {
//    goal_q[i] = temp_joint_values(i)+ u[i]*interval.toSec();
    joint_velocities[i] = u[i];
  }
  pthread_mutex_unlock(&joint_velocities_mutex);
  new_goal_got = true;
// sendCommand(goal_q,interval);

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

//  Eigen::Vector3d target_position;
//  target_position << goal_pos[0],goal_pos[1],goal_pos[2];
//  straight_length = (target_position-point_x_position).norm();


  traj_point_x.x = point_x_position(0);
  traj_point_x.y = point_x_position(1);
  traj_point_x.z = point_x_position(2);
  line_strip.points[0] = traj_point_x;
//  double length = 0;
  for(int i=0;i<ph;i++)
  {
    int index = std::min(i,ch-1);
    for(int j=0;j<7;j++)
      traj_point_q[j] += u[index*7+j]*Ts;
    updateJointState(traj_point_q);
    point_x_position = getEEFTransform().translation();
//    Eigen::Vector3d point_x_position_new = getEEFTransform().translation();
//    length += (point_x_position_new-point_x_position).norm();
//    point_x_position = point_x_position_new;
    traj_point_x.x = point_x_position(0);
    traj_point_x.y = point_x_position(1);
    traj_point_x.z = point_x_position(2);
    line_strip.points[i+1] = traj_point_x;
  }

//  double norm_length = length/straight_length;
//  average_norm_length += norm_length/iter_num;

  marker_publisher.publish(line_strip);

  line_strip.color.g = 1.0;
  line_strip.color.r = 0.0;
  line_strip.id = 2;
  line_strip.type = visualization_msgs::Marker::POINTS;
  line_strip.ns = "trajectory_point";
  marker_publisher.publish(line_strip);
}

void NmpcNlopt::global_optimize(std::vector<double>& u, double mincost)
{
  try{
      nlopt::result result = global_optimizer.optimize(u,mincost);
      std::cout << result << std::endl;
  }
  catch(std::exception &e){
    std::cout << "nlopt failed: " << e.what() << std::endl;
  }
  new_goal = false;

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


// velocitySend thread for kinova ros api in real robot
void NmpcNlopt::velocitiesSend_thread(ros::Publisher publisher2real, bool* exitFlag,
                                      std::vector<double>* v_ptr,
                                      pthread_mutex_t *joint_velocities_mutex,
                                      bool* new_goal_got)
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
          // ROS_WARN("Joint velocities not updte for a long time, sending 0 velocities to robot.");
          std::fill(temp_v.begin(),temp_v.end(),0);
        }
        else
        {
          pthread_mutex_lock(joint_velocities_mutex);
          temp_v = *v_ptr;
          pthread_mutex_unlock(joint_velocities_mutex);
        }
      }

      kinova_msgs::JointVelocity velocity_msg;
      velocity_msg.joint1 = temp_v[0]*57.2958;
      velocity_msg.joint2 = temp_v[1]*57.2958;
      velocity_msg.joint3 = temp_v[2]*57.2958;
      velocity_msg.joint4 = temp_v[3]*57.2958;
      velocity_msg.joint5 = temp_v[4]*57.2958;
      velocity_msg.joint6 = temp_v[5]*57.2958;
      velocity_msg.joint7 = temp_v[6]*57.2958;
      publisher2real.publish(velocity_msg);
    }
    d.sleep();
  }
  std::cout << "Velocities sending thread ended. " << std::endl;
}


void NmpcNlopt::controlLoop()
{
  if(!state_sub_started)
    return;
  // update joint states
  for(int i=0;i<7;i++)
  {
    temp_joint_values(i) = curr_joint_values[i];
  }
  // update tf between arm base frame and global frame first
  geometry_msgs::TransformStamped transformStamped;
  try{
  transformStamped = tfBuffer.lookupTransform("world", "origin",
                           ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
  ROS_WARN("%s",ex.what());
  return;
  }
  tf::transformMsgToEigen(transformStamped.transform, origin2base);

  // send current eef pose out to teleoperation device
  Vector7d curr_eef;
  jaco_hess_cal.getPosition(curr_joint_values,curr_eef);
  curr_eef_pose.position.x = curr_eef(0);
  curr_eef_pose.position.y = curr_eef(1);
  curr_eef_pose.position.z = curr_eef(2);
  curr_eef_pose.orientation.w = curr_eef(3);
  curr_eef_pose.orientation.x = curr_eef(4);
  curr_eef_pose.orientation.y = curr_eef(5);
  curr_eef_pose.orientation.z = curr_eef(6);

  eef_pose_pub.publish(curr_eef_pose);


 // if(track_mode&&goal_sub_started)
  if(track_mode)
  {

// performance evaluation
//    average_time = 0;
//    average_norm_length = 0;
//    iter_num = 20;
//    for(int it=0;it<iter_num;it++)
//    {
//      // set random u
//      struct timespec ts;
//      for(int i = 0;i<ch*7;i++)
//      {
//        clock_gettime(CLOCK_MONOTONIC, &ts);
//        srand((time_t)ts.tv_nsec);
//        u[i] = (double)rand()/(double)RAND_MAX;
//        u[i] = u[i]*1.6-0.8;
//      }
//      optimize(u,mincost);
//    }
//    std::cout << "average time: " << average_time << std::endl;
//    std::cout << "average norm length: " << average_norm_length << std::endl;
    // move obstacle
//    obstacle_state.pose.position.z = std::max(0.1,obstacle_state.pose.position.z-obstacle_v*Ts);
//    gazebo_model_pub.publish(obstacle_state);
//    goal_pos[0] = std::min(goal_pos[0] + robot_v*Ts, 0.8);
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


void NmpcNlopt::test_tf_listener()
{
  ros::Rate loop_rate_sleep(0.5);
  loop_rate_sleep.sleep();
//  auto t3 = std::chrono::high_resolution_clock::now();
  geometry_msgs::TransformStamped transformStamped1;
//  geometry_msgs::TransformStamped transformStamped2;
//  geometry_msgs::TransformStamped transformStamped3;
  try{
  transformStamped1 = tfBuffer.lookupTransform("origin", "j2s7s300_end_effector",
                           ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
  ROS_WARN("%s",ex.what());
  return;
  }
//  try{
//  transformStamped2 = tfBuffer.lookupTransform("j2s7s300_link_base", "j2s7s300_link_7",
//                           ros::Time(0));
//  }
//  catch (tf2::TransformException &ex) {
//  ROS_WARN("%s",ex.what());
//  return;
//  }

//  try{
//  transformStamped3 = tfBuffer.lookupTransform("j2s7s300_link_6", "j2s7s300_link_7",
//                           ros::Time(0));
//  }
//  catch (tf2::TransformException &ex) {
//  ROS_WARN("%s",ex.what());
//  return;
//  }

  Eigen::Isometry3d transform_eigen1;
  tf::transformMsgToEigen(transformStamped1.transform, transform_eigen1);

//  Eigen::Isometry3d transform_eigen1, transform_eigen2, transform_eigen3;
//  auto t3 = std::chrono::high_resolution_clock::now();
//  tf::transformMsgToEigen(transformStamped1.transform, transform_eigen1);
//  tf::transformMsgToEigen(transformStamped2.transform, transform_eigen2);
//  transform_eigen3 = transform_eigen1*transform_eigen2;

//  auto t4 = std::chrono::high_resolution_clock::now();
//  auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>( t4 - t3 ).count();
//  std::cout << "transform production takes time: "<<duration2 << std::endl;


  Eigen::Quaterniond quat_eigen1(transform_eigen1.rotation());
  std::cout << "translation1:\n " << transform_eigen1.translation().transpose() << std::endl;
  std::cout << "rotation1: \n" << quat_eigen1.coeffs().transpose() << std::endl;

  return;
}
