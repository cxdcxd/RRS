#include <nmpc_nlopt_movo.h>
#include <chrono>
NmpcNlopt::NmpcNlopt(ros::NodeHandle nh):MoveitTool(nh)
{
  
  ROS_WARN("Unity Sim SIM ");
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
  // parameter: number of threads
  if(!nh.getParam("/multi_threading/num_cost_threads",num_cost_threads))
  {
    ROS_ERROR("error loading  num_cost_threads");
     param_ok = false;
  }
  if(!nh.getParam("/multi_threading/num_cnt_threads",num_cnt_threads))
  {
    ROS_ERROR("error loading  num_cnt_threads");
     param_ok = false;
  }




  if(!nh.getParam(node_name+"/arm_name_space",arm_name_space))
  {
    ROS_ERROR("error loading  arm_name_space");
     param_ok = false;
  }

  if(!nh.getParam(node_name+"/plot_obstacle",plot_obstacle))
  {
    ROS_ERROR("error loading  plot_obstacle");
     param_ok = false;
  }

  if(!param_ok)
  {
    ROS_ERROR("Param not ok , ROS Shutdown ***********");
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

  ROS_WARN("Unity Sim Param Done");
  ROS_WARN_STREAM("Path " << "/movo/"+arm_name_space+"_arm_controller/state");

  joint_state_sub = nh.subscribe("/movo/"+arm_name_space+"_arm_controller/state", 10, &NmpcNlopt::jointStateSubCB_real,this); 
  movo2kinova_offset = {0,3.1416,0,3.1416,0,3.1416,0};

  state_sub_started = false;
  goal_sub_started = false;
 
  obstacleTable_shape = std::make_shared<shapes::Box>(0.375,0.375,1);

  goal_sub = nh.subscribe(arm_name_space+"/nmpc_controller/in/goal",1,&NmpcNlopt::goalSubCBFromOut,this);

  velocity_pub = nh.advertise<movo_msgs::JacoAngularVelocityCmd7DOF>("/movo/"+arm_name_space+"_arm/angular_vel_cmd",1);

  string this_arm_base, other_arm_base;
  if(arm_name_space=="left") 
  {
    goal_msg.joint_names={"left_shoulder_pan_joint", "left_shoulder_lift_joint", "left_arm_half_joint", "left_elbow_joint",
                "left_wrist_spherical_1_joint", "left_wrist_spherical_2_joint", "left_wrist_3_joint"};
    other_cylinders_sub = nh.subscribe("right/nmpc_controller/out/cylinder_poses",1,&NmpcNlopt::otherCylindersSubCB,this);
    this_arm_base = "left_mpc_base_link";
    other_arm_base= "right_mpc_base_link";
    othter_arm_name_space = "right";
  }
  else
  {
    goal_msg.joint_names={"right_shoulder_pan_joint", "right_shoulder_lift_joint", "right_arm_half_joint", "right_elbow_joint",
                "right_wrist_spherical_1_joint", "right_wrist_spherical_2_joint", "right_wrist_3_joint"};
    other_cylinders_sub = nh.subscribe("left/nmpc_controller/out/cylinder_poses",1,&NmpcNlopt::otherCylindersSubCB,this);
    this_arm_base = "right_mpc_base_link";
    other_arm_base= "left_mpc_base_link";
    othter_arm_name_space = "left";
  }
  cylinders_added = false;
  obstacle_marker.header.frame_id = this_arm_base;
  self_cylinders_pub = nh.advertise<perception_msgs::Cylinders>(arm_name_space+"/nmpc_controller/out/cylinder_poses",1);

  eef_pose_pub = nh.advertise<geometry_msgs::Pose>(arm_name_space+"/nmpc_controller/out/eef_pose",1);

  Ts = 0.2;
  interval = ros::Duration(Ts);

  joint_velocities_mutex = PTHREAD_MUTEX_INITIALIZER;
  position_goal_mutex = PTHREAD_MUTEX_INITIALIZER;

  // initialize optmizer
  optimizer = nlopt::opt(nlopt::LD_SLSQP, ch*7);

  // add upper and lower bounds;
  lb.resize(ch*7);
  std::fill(lb.begin(),lb.end(),-2*0.17453);
  optimizer.set_lower_bounds(lb);
  ub.resize(ch*7);
  std::fill(ub.begin(),ub.end(),2*0.17453);
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
  std::vector<double> tol(ph,1e-2);
  optimizer.add_inequality_mconstraint(obstacleConstraintsMultiThread,this,tol);

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
  thread1 = std::thread(velocitiesSend_thread_sim,&curr_joint_values, velocity_pub, &exitFlag, &joint_velocities, &goal_msg, &joint_velocities_mutex, &position_goal_mutex,&new_goal_got, &position_goal);
  //thread1 = std::thread(velocitiesSend_thread_real, velocity_pub, &exitFlag, &joint_velocities, &joint_velocities_mutex,&new_goal_got);

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

  // initialize marker for visualizing predicted trajectory
  line_strip.scale.x = 5e-3;
  line_strip.scale.y = 5e-3;
  line_strip.scale.z = 5e-3;
  line_strip.header.frame_id = this_arm_base;

  // init tf for goal from outside
  transformStamped_goal.header.frame_id = this_arm_base;
  transformStamped_goal.child_frame_id = arm_name_space+"goal_from_outside";

  // initialize tf listener
  tfListener =  std::make_shared<tf2_ros::TransformListener>(tfBuffer);
  // update tf between arm base frame and global frame first

  ROS_INFO("Wait 3 seconds for tf coming in");
  ros::Duration(3).sleep();

  geometry_msgs::TransformStamped transformStamped;
  try{
  transformStamped = tfBuffer.lookupTransform(this_arm_base, other_arm_base,
                           ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
  ROS_WARN("%s",ex.what());
  return;
  }

  tf::transformMsgToEigen(transformStamped.transform, other_arm_base2this_arm_base);

  try{
  transformStamped = tfBuffer.lookupTransform(this_arm_base, "base_link",
                           ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
  ROS_WARN("%s",ex.what());
  return;
  }
  tf::transformMsgToEigen(transformStamped.transform, base2this_arm_base);

  try{
  transformStamped = tfBuffer.lookupTransform("base_link", "upper_body_link",
                           ros::Time(0));
  }
  catch (tf2::TransformException &ex) {
  ROS_WARN("%s",ex.what());
  return;
  }
  body_height = transformStamped.transform.translation.z;
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
}

void NmpcNlopt::jointStateSubCB_real(const sensor_msgs::JointState::ConstPtr& msg)
{
  //ROS_INFO("Get message");

  if(!state_sub_started)
    state_sub_started = true;
  const auto& positions = msg->position;
  for(int i=0;i<joint_num;i++)
  {
    curr_joint_values[i] = angles::normalize_angle(positions[i]+movo2kinova_offset[i]);
  }
}

void NmpcNlopt::goalSubCB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg)
{
  goal_position << msg->pose.position.x,msg->pose.position.y,msg->pose.position.z,
                   msg->pose.orientation.w,msg->pose.orientation.x,
                   msg->pose.orientation.y,msg->pose.orientation.z;

  transformStamped_goal.header.stamp = ros::Time::now();
  transformStamped_goal.transform.translation.x = msg->pose.position.x;
  transformStamped_goal.transform.translation.y = msg->pose.position.y;
  transformStamped_goal.transform.translation.z = msg->pose.position.z;

  transformStamped_goal.transform.rotation.x = msg->pose.orientation.x;
  transformStamped_goal.transform.rotation.y = msg->pose.orientation.y;
  transformStamped_goal.transform.rotation.z = msg->pose.orientation.z;
  transformStamped_goal.transform.rotation.w = msg->pose.orientation.w;
  br.sendTransform(transformStamped_goal);
  if(!goal_sub_started)
    goal_sub_started = true;
}

void NmpcNlopt::goalSubCBFromOut(const geometry_msgs::PoseConstPtr &msg)
{
  goal_position << msg->position.x,msg->position.y,msg->position.z,
                   msg->orientation.w,msg->orientation.x,
                   msg->orientation.y,msg->orientation.z;

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

void NmpcNlopt::obstacleTable()
{
  Eigen::Translation3d translation(0,0,0);
	Eigen::Quaterniond quater(1,0,0,0);
	Eigen::Affine3d obj_pose = translation*quater.toRotationMatrix();

	if(!co_ptrs.count("obstacleTable"))
    addObstacle("obstacleTable", obstacleTable_shape, obj_pose);
	else if((obj_pose.matrix()-co_ptrs["obstacleTable"]->getTransform().matrix()).norm() > 1e-4)
    updateObstacle("obstacleTable",obj_pose);
}

void NmpcNlopt::otherCylindersSubCB(const perception_msgs::CylindersConstPtr& msg)
{
  const auto& other_centers = msg->centers;
  const auto& other_x_axes = msg->x_axes;
  const auto& other_y_axes = msg->y_axes;
  const auto& other_z_axes = msg->z_axes;
  if(!cylinders_added)
  {
    for(int i=0;i<3;i++)
    {
        Eigen::Affine3d obj_pose;
        obj_pose.setIdentity();
        const auto& center = other_centers[i];
        obj_pose.translation() << center.x,center.y,center.z;
        const auto& x_axis=other_x_axes[i], y_axis=other_y_axes[i], z_axis=other_z_axes[i];
        obj_pose.linear() << x_axis.x,y_axis.x,z_axis.x,
        					   x_axis.y,y_axis.y,z_axis.y,
        					   x_axis.z,y_axis.z,z_axis.z;
        obj_pose = other_arm_base2this_arm_base*obj_pose;
        std::shared_ptr<shapes::Cylinder> other_cylinder_shape;
        if(i<2)
          other_cylinder_shape = std::make_shared<shapes::Cylinder>(0.05,0.3);
        else
          other_cylinder_shape = std::make_shared<shapes::Cylinder>(0.05,0.5);
        addObstacle(arm_name_space+"_otherCylinder"+std::to_string(i),other_cylinder_shape,obj_pose,true);
    }
    cylinders_added=true;
  }
  else
  {
    for(int i=0;i<3;i++)
    {
      Eigen::Affine3d obj_pose;
			obj_pose.setIdentity();
			const auto& center = other_centers[i];
			obj_pose.translation() << center.x,center.y,center.z;
			const auto& x_axis=other_x_axes[i], y_axis=other_y_axes[i], z_axis=other_z_axes[i];
			obj_pose.linear() << x_axis.x,y_axis.x,z_axis.x,
								   x_axis.y,y_axis.y,z_axis.y,
								   x_axis.z,y_axis.z,z_axis.z;
			obj_pose = other_arm_base2this_arm_base*obj_pose;     
      updateObstacle(arm_name_space+"_otherCylinder"+std::to_string(i),obj_pose,true);
    }

  }
}

bool NmpcNlopt::track_toggle_cb(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
  track_mode = track_mode?false:true;
  return true;
}


void NmpcNlopt::addObstacle(const std::string &obj_name, const shapes::ShapeConstPtr &obj_shape, const Eigen::Affine3d &obj_transform, const bool plot)
{

  // transfer global coordinate of the obstacle to local coordinate
  // Eigen::Affine3d local_obj_transform = origin2base * obj_transform;
  Eigen::Affine3d local_obj_transform = obj_transform;
  MoveitTool::addObstacle(obj_name,obj_shape,local_obj_transform,plot);
  for(int i=0;i<num_cnt_threads;i++)
   MoveitTools_array[i]->addObstacle(obj_name,obj_shape,local_obj_transform);
  
  int m = ph*(3*getObstacleNum()+1);
  std::vector<double> tol(m,1e-3);
  optimizer.remove_inequality_constraints();
  optimizer.add_inequality_mconstraint(obstacleConstraintsMultiThread,this,tol);
}

void NmpcNlopt::updateObstacle(const std::string& obj_name, const Eigen::Affine3d& obj_transform, const bool plot)
{
  MoveitTool::updateObstacle(obj_name,obj_transform,plot);
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
        class_ptr->jaco_hess_cal.getPosition(curr_x, eef_position);

        // Eigen::Vector3d diff_position = goal_position.head<3>() - eef_position.head<3>();
        // double w1=eef_position(3),w2=goal_position(3);
        // Eigen::Vector3d q1=eef_position.tail<3>(),q2=goal_position.tail<3>();
        // Eigen::Matrix3d skew_q2;
        // skew_q2 << 0,-q2(2),q2(1),q2(2),0,-q2(0),-q2(1),q2(0),0;
        // Eigen::Vector3d diff_orientation = w1*q2-w2*q1-skew_q2*q1;
        // diff_running << diff_position,diff_orientation;

        class_ptr->jaco_hess_cal.getEEFTransform(curr_x, eef_rotm, eef_translation);
        class_ptr->jaco_hess_cal.rotm2quat(eef_rotm,eef_quat);
        Eigen::Vector3d diff_position = goal_position.head<3>() - eef_translation;
        double w1=eef_quat(0),w2=goal_position(3);
        Eigen::Vector3d q1=eef_quat.tail<3>(),q2=goal_position.tail<3>();
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

        // class_ptr->jaco_hess_cal.getAnalyticalJacobian(curr_x,Ji_quat);
        Eigen::Matrix<double,6,7> Ji;
        class_ptr->jaco_hess_cal.getAnalyticalJacobianOmega(curr_x, Ji);
        double w =eef_quat(0),x=eef_quat(1),y=eef_quat(2),z=eef_quat(3);
        Eigen::Matrix<double,4,3> quaternion_update_matrix;
        quaternion_update_matrix << -x,-y,-z,w,z,-y,-z,w,x,y,-x,w;
        Ji_quat.topRows<3>() = Ji.topRows<3>();
        Ji_quat.bottomRows<4>() = 0.5*quaternion_update_matrix*Ji.bottomRows<3>();



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
      const int num_dists = num_links*num_obstacles+1;
      const int n = 7*ch;

      // output partial cost value container
      std::vector<double>& cnt_vals = class_ptr->partial_cnt_val[thread_index];
      // output partial cost gradient container
      std::vector<double>& cnt_grads = class_ptr->partial_cnt_grads[thread_index];

      std::vector<double> curr_x(7);
      MoveitTool* local_moveit = class_ptr->MoveitTools_array[thread_index];

      int local_num_loops = (thread_index==(class_ptr->num_cnt_threads-1))?class_ptr->last_num_cnt_loops:class_ptr->num_cnt_loops;



      for(int i=0;i<local_num_loops;i++)
      {
      	auto t3 = std::chrono::high_resolution_clock::now();
        int global_i = i + thread_index*class_ptr->num_cnt_loops;
        for(int j =0;j<7;j++)
          curr_x[j] = class_ptr->jnt_vals_ptr[global_i][j];
        local_moveit->updateJointState(curr_x, true);
        std::vector< std::vector<double> > dists;
        std::vector<std::vector<Eigen::Vector3d> > points1;
        std::vector<std::vector<Eigen::Vector3d> > points2;
        local_moveit->getDistsAndPoints(dists,points1,points2);
        for(int j=0;j<num_obstacles;j++)
        {
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

  int id_offset;
  if(arm_name_space=="left")
    id_offset = 1;
  else
    id_offset = 10;

  std::vector<double> traj_point_q(7);
  line_strip.color.g = 0.0;
  line_strip.color.r = 1.0;
  line_strip.id = 1+id_offset;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.ns = arm_name_space+"_trajectory";
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
  line_strip.id = 2+id_offset;
  line_strip.type = visualization_msgs::Marker::POINTS;
  line_strip.ns = arm_name_space+"_trajectory_point";
  marker_publisher.publish(line_strip);
}
// velocitySend thread for arm in movo simulation
void NmpcNlopt::velocitiesSend_thread_sim(std::vector<double>* c_ptr,ros::Publisher publisher, bool* exitFlag,
                                      std::vector<double>* v_ptr, trajectory_msgs::JointTrajectory* msg,
                                      pthread_mutex_t *joint_velocities_mutex, pthread_mutex_t* position_goal_mutex,
                                      bool* new_goal_got, std::vector<double>* position_goal_ptr)
{
  double passsed_time_after_new_goal;
  std::vector<double> temp_v(7);
  std::vector<double> temp_c(7);
  std::vector<double> temp_error(7);
  float delta = 0.002;

  ros::Duration d(delta);
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
        passsed_time_after_new_goal +=delta;
        if(passsed_time_after_new_goal > 0.2)
        {
//          ROS_WARN("Joint velocities not updte for a long time, sending 0 velocities to robot.");
          std::fill(temp_v.begin(),temp_v.end(),0);
        }
        else
        {
          pthread_mutex_lock(joint_velocities_mutex);
          temp_v = *v_ptr;
          temp_c = *c_ptr;
          pthread_mutex_unlock(joint_velocities_mutex);
        }
      }

      std::string line = "";
      std::string line2 = "";
      std::string line3 = "";
      std::string line4 = "";

      msg->header.seq++;
      msg->points.clear();
      trajectory_msgs::JointTrajectoryPoint msg_point;
      msg_point.time_from_start = d;
      msg_point.positions.resize(7);
      double movo2kinova_offset[7] = {0,3.1416,0,3.1416,0,3.1416,0};
      pthread_mutex_lock(position_goal_mutex);

      for(int i=0;i<7;i++)
      {
        (*position_goal_ptr)[i] += (temp_v[i])*delta;
        msg_point.positions[i] = angles::normalize_angle((*position_goal_ptr)[i]-movo2kinova_offset[i]);
        temp_error[i] = msg_point.positions[i] - temp_c[i];
        line += std::to_string(temp_v[i]) + " ";
        line2 += std::to_string(msg_point.positions[i]) + " ";
        line3 += std::to_string(temp_c[i]) + " ";
        line4 += std::to_string(temp_error[i]) + " ";
      }

      pthread_mutex_unlock(position_goal_mutex);

      // msg_point.positions = *position_goal_ptr;
      msg->points.push_back(msg_point);
      publisher.publish(*msg);

      ROS_INFO_STREAM("Velocity : " + line);
      ROS_INFO_STREAM("Goal : " + line2);
      ROS_INFO_STREAM("Current : " + line3);
      ROS_INFO_STREAM("Error : " + line4);
      
      ROS_INFO("====");

    }
    d.sleep();
  }
  std::cout << "Velocities sending thread ended. " << std::endl;
}


//velocitySend thread for arm in real robot
void NmpcNlopt::velocitiesSend_thread_real(ros::Publisher publisher2real, bool* exitFlag,
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
      std::vector<double> temp_v(7);
      pthread_mutex_lock(joint_velocities_mutex);
      temp_v = *v_ptr;
      pthread_mutex_unlock(joint_velocities_mutex);

      movo_msgs::JacoAngularVelocityCmd7DOF velocity_msg;
      velocity_msg.theta_shoulder_pan_joint = temp_v[0]*57.2958;
      velocity_msg.theta_shoulder_lift_joint = temp_v[1]*57.2958;
      velocity_msg.theta_arm_half_joint = temp_v[2]*57.2958;
      velocity_msg.theta_elbow_joint = temp_v[3]*57.2958;
      velocity_msg.theta_wrist_spherical_1_joint = temp_v[4]*57.2958;
      velocity_msg.theta_wrist_spherical_2_joint = temp_v[5]*57.2958;
      velocity_msg.theta_wrist_3_joint = temp_v[6]*57.2958;

      publisher2real.publish(velocity_msg);
    }
    d.sleep();
  }
  std::cout << "Velocities sending thread ended. " << std::endl;
}


void NmpcNlopt::initialize()
{
	// add a box obstacle representing robot body
	shapes::ShapeConstPtr obj_shape1(new shapes::Box(0.52,0.4,0.25));
	Eigen::Quaterniond quater(0,0,0,1);
  quater.normalize ();

	Eigen::Translation3d translation(0.05,0,body_height+0.5);
	Eigen::Affine3d obj_pose = translation*quater;
	obj_pose = base2this_arm_base*obj_pose;
	addObstacle("head",obj_shape1,obj_pose,plot_obstacle);

	shapes::ShapeConstPtr obj_shape2(new shapes::Box(0.4,0.4,0.75));
	translation.x()=-0.01;translation.y()=0;translation.z()=body_height;
	obj_pose = translation*quater;
	obj_pose = base2this_arm_base*obj_pose;
	addObstacle("body",obj_shape2,obj_pose,plot_obstacle);

	shapes::ShapeConstPtr obj_shape3(new shapes::Box(0.77,0.5,0.316));
	translation.x()=0;translation.y()=0;translation.z()=0.158;
	obj_pose = translation*quater;
	obj_pose = base2this_arm_base*obj_pose;
	addObstacle("base",obj_shape3,obj_pose,plot_obstacle);

	shapes::ShapeConstPtr obj_shape4(new shapes::Box(2,2,0.01));
	translation.x()=0;translation.y()=0;translation.z()=0;
	obj_pose = translation*quater;
	obj_pose = base2this_arm_base*obj_pose;
	addObstacle("ground",obj_shape4,obj_pose,plot_obstacle);

  shapes::ShapeConstPtr obj_shape5(new shapes::Box(0.4,0.1,0.75));
	translation.x()=0.6;translation.y()=0;translation.z()=0.45;
	obj_pose = translation*quater;
	obj_pose = base2this_arm_base*obj_pose;
	addObstacle("Table",obj_shape5,obj_pose,plot_obstacle);
}


void NmpcNlopt::controlLoop()
{
  //ROS_INFO("Control Loop ");

	if(!state_sub_started)
		return;
  // update joint states
  pthread_mutex_lock(&position_goal_mutex);
  for(int i=0;i<7;i++)
  {
    temp_joint_values(i) = curr_joint_values[i];
    position_goal[i] = curr_joint_values[i];
  }
  updateJointState(curr_joint_values,true,true);
  pthread_mutex_unlock(&position_goal_mutex);
  perception_msgs::Cylinders cylinders;
  cylinders.centers.resize(3);
  cylinders.x_axes.resize(3);
  cylinders.y_axes.resize(3);
  cylinders.z_axes.resize(3);
  for(int i=0;i<3;i++)
  {
  	cylinders.centers[i].x = centroids[i][0];cylinders.centers[i].y = centroids[i][1];cylinders.centers[i].z = centroids[i][2];
  	cylinders.x_axes[i].x = x_axes[i][0];cylinders.x_axes[i].y = x_axes[i][1];cylinders.x_axes[i].z = x_axes[i][2];
  	cylinders.y_axes[i].x = y_axes[i][0];cylinders.y_axes[i].y = y_axes[i][1];cylinders.y_axes[i].z = y_axes[i][2];
  	cylinders.z_axes[i].x = z_axes[i][0];cylinders.z_axes[i].y = z_axes[i][1];cylinders.z_axes[i].z = z_axes[i][2];
  }
  self_cylinders_pub.publish(cylinders);


  // only need right arm eef position
  const auto& eef_transform = getEEFTransform();
  const auto& translation = eef_transform.translation();
  const auto& rotation = eef_transform.linear();
  Eigen::Quaterniond quat(rotation);
  geometry_msgs::Pose eef_pose_msg;
  eef_pose_msg.position.x = translation.x();
  eef_pose_msg.position.y = translation.y();
  eef_pose_msg.position.z = translation.z();
  eef_pose_msg.orientation.w = quat.w();
  eef_pose_msg.orientation.x = quat.x();
  eef_pose_msg.orientation.y = quat.y();
  eef_pose_msg.orientation.z = quat.z();
  eef_pose_pub.publish(eef_pose_msg);

//  if(track_mode&&goal_sub_started)
  if(goal_sub_started)
  {
    optimize(u,mincost);

		// warm start
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