#include <lmt_movebase.h>

// #define VIRTUALMODE

LMTMoveBase::LMTMoveBase() : 
App_exit(false),
is_sim(false),
_thread_PathFwr(&LMTMoveBase::PathFwr,this),
_thread_Logic(&LMTMoveBase::logic_thread,this),
_thread_Vis(&LMTMoveBase::vis_thread,this),
_thread_Localization(&LMTMoveBase::Localization_thread,this)
{
    init();
}

LMTMoveBase::~LMTMoveBase()
{
	kill();
}

double LMTMoveBase::Quat2Rad(double orientation[])
{
    tf::Quaternion q(orientation[0], orientation[1], orientation[2], orientation[3]);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

void LMTMoveBase::publish_isrobotmove()
{
    std_msgs::Bool _msg;
    _msg.data = getrobotmove();
    pub_move.publish(_msg);
}

void LMTMoveBase::setsystemstate(int value,bool forced = false)
{
   if ( getstatemutex() || forced)
   		system_state = value;
}

void LMTMoveBase::setlogicstate(int value,bool forced = false)
{
	if ( getstatemutex() || forced)
   		logic_state = value;
}

int LMTMoveBase::getsystemstate()
{
   return system_state;
}

int LMTMoveBase::getlogicstate()
{
  return logic_state;
}

bool LMTMoveBase::getstatemutex()
{
   return statemutex;
}

void LMTMoveBase::setstatemutex(bool value)
{
    statemutex = value;
}

void LMTMoveBase::say_message(string data)
{
    if ( say_enable == false ) return;
    playVoice(data);
}

void LMTMoveBase::send_omni(double x,double y ,double w)
{
        x = x;
        y = y;
        w = w * -1;

        //cout<<"publish : "<<x<<" "<<y<<" "<<w<<endl;

        geometry_msgs::Twist myTwist;

        myTwist.linear.x = x;
        myTwist.linear.y = -y;
        myTwist.angular.z = -w;
        
        if ( myTwist.linear.x > 0.2 ) myTwist.linear.x = 0.2;
        if ( myTwist.linear.x < -0.2 ) myTwist.linear.x = -0.2;

        if ( myTwist.linear.y > 0.2 ) myTwist.linear.y = 0.2;
        if ( myTwist.linear.y < -0.2 ) myTwist.linear.y = -0.2;

        if ( myTwist.angular.z > 0.3 ) myTwist.angular.z = 0.3;
        if ( myTwist.angular.z < -0.3 ) myTwist.angular.z = -0.3;

        cout<<"Final publish : "<<myTwist.linear.x<<" "<<myTwist.linear.y<<" "<<myTwist.angular.z<<endl;
        mycmd_vel_pub.publish(myTwist); 
}

void LMTMoveBase::force_stop()
{
    cout<<"force stop"<<endl;
    send_omni(0,0,0);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}

double LMTMoveBase::GetDistance(double x1, double y1, double x2, double y2)
{
    double x = x2-x1;
    double y = y2-y1;
    return sqrt(x*x + y*y);
}

int LMTMoveBase::GetCurrentStep()
{
    for(int i=0;i<globalPathSize-1;i++)
    {
        if(GetDistance(position[0],position[1],globalPath.poses[i].pose.position.x, globalPath.poses[i].pose.position.y) < GetDistance(position[0],position[1],globalPath.poses[i+1].pose.position.x, globalPath.poses[i+1].pose.position.y))
            return i;
    }
    return globalPathSize-1;
}

void LMTMoveBase::LMTmapengine_savemap()
{
   std_srvs::Empty _s;
   client_map_save.call(_s);
}

void LMTMoveBase::LMTmapengine_loadmap()
{
   std_srvs::Empty _s;
   client_map_load.call(_s);
}

void LMTMoveBase::clean_costmaps()
{
   std_srvs::Empty _s;
   client_resetcostmap.call(_s);
}

//cm cm degree
void LMTMoveBase::update_hector_origin(float x,float y,float yaw)
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    pub_slam_origin.publish(msg);
}

void LMTMoveBase::reset_hector_slam()
{
	std_msgs::String _msg;
	_msg.data = "reset";
	pub_slam_reset.publish(_msg);
}

void LMTMoveBase::read_config()
{
        std::string path_points =  ros::package::getPath("managment") + "/maps/moveconfig.txt";
        cout<<path_points<<endl;
        std::string line;
        std::ifstream text;

        goal_list.clear();
        text.open(path_points.c_str(), ios_base::in);
 
        if (text.is_open())
        {
            getline(text,line);
            _normal_max_linear_speedX = atof(line.c_str());
            
            getline(text,line);
            _normal_max_angular_speed = atof(line.c_str());
            
            getline(text,line);
            _normal_kp_linearX = atof(line.c_str());
            
            getline(text,line);
            _norma_kp_angular = atof(line.c_str());
           
        }

    _goal_max_linear_speedX = _normal_max_linear_speedX;
    _goal_max_angular_speed = _normal_max_angular_speed;
    _goal_kp_linearX = _normal_kp_linearX;
    _goal_kp_angular = _norma_kp_angular;
            
}

void LMTMoveBase::read_file()
{

        std::string path_points =  ros::package::getPath("managment") + "/maps/points.txt";
        cout<<path_points<<endl;
        std::string line;
        std::ifstream text;

        goal_list.clear();
        text.open(path_points.c_str(), ios_base::in);

        if (text.is_open())
        {
        	
            getline(text,line);

            while (text.good())
            {
                vector <string> fields;

                boost::split( fields, line, boost::is_any_of( "," ) );
                cout<<line<<endl;
                goal_data gdata;

                 gdata.id = fields[0].c_str();
                 gdata.x = atoi(fields[1].c_str());
                 gdata.y = atoi(fields[2].c_str());
                 gdata.yaw = atoi(fields[3].c_str());
                 gdata.height = atoi(fields[4].c_str());

                goal_list.push_back(gdata);
               
                getline(text,line);

            }
            text.close();

        }
        else
        {
            std::cout << coutcolor_blue << "[Unable to open file]" << coutcolor0 <<std::endl << std::endl;
        }

        std::cout << coutcolor_blue << "read done : " << coutcolor0 << goal_list.size()<<std::endl << std::endl;
}

void LMTMoveBase::SaveLastPosition()
{
    // std::string lastPositionPath =  ros::package::getPath("LMTMovebase") + "/files/lastPosition.txt";
    // std::string str;
    // std::ofstream text("/home/LMT/catkin_ws/src/LMT3/Navigation/LMTMoveBase/files/lastPosition.txt", std::ios::out);
    // if (text.is_open())
    // {
    //     str = boost::lexical_cast<std::string>(position[0]);
    //     text << str + "\n";
    //     str = boost::lexical_cast<std::string>(position[1]);
    //     text << str + "\n";
    //     str = boost::lexical_cast<std::string>(position[1]);
    //     text << str;
    //     text.close();
    // }
    // else cout << "Unable to open lastPosition file";
}

int LMTMoveBase::find_goal_byname(string name)
{
    for ( int i = 0 ; i < goal_list.size() ; i++ )
    {
        if ( name == goal_list.at(i).id )
        {
           return i;
        }
    }

    return -1;
}

nav_msgs::Path LMTMoveBase::call_make_plan()
{
    nav_msgs::GetPlan srv;

    srv.request.start.header.frame_id = "map";
    srv.request.start.pose.position.x = position[0];
    srv.request.start.pose.position.y = position[1];
    srv.request.start.pose.orientation = tf::createQuaternionMsgFromYaw(tetha);

    srv.request.goal.header.frame_id = "map";
    srv.request.goal.pose.position.x = goalPos[0];
    srv.request.goal.pose.position.y = goalPos[1];
    srv.request.goal.pose.orientation = tf::createQuaternionMsgFromYaw(goalTetha);

    srv.request.tolerance = 0.1;
    client_makeplan.call(srv);

    return srv.response.plan;
}

void LMTMoveBase::logic_thread()
{
     boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    std::cout<<"logic thread started"<<endl;
    nav_msgs::Path result;

    while(ros::ok() && !App_exit)
    {
         boost::this_thread::sleep(boost::posix_time::milliseconds(500));
         //cout<<"isLMTmove : "<< isrobotmove << endl;

         if ( getlogicstate() == -1)
         {
            cout<<"Get Error With Hector Status"<<endl;
            say_message("There is a problem with my sensors");
           
            reset_hector_slam();
            update_hector_origin(position[0],position[1],tetha);
            boost::this_thread::sleep(boost::posix_time::milliseconds(2000));   
            clean_costmaps();     
            say_message("My laser recovered successfuly");
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            setsystemstate(tempSystemState);
            setlogicstate(tempLogicState);
       

         }
         if ( getlogicstate() == 0 )
         {
            IsGoalReached = false;
            if ( idle_flag == false )
            {
                idle_flag = true;
                cout<<"wait for exe , idle"<<endl;
            }
            
         }

         else if ( getlogicstate() == 1 )
         {
               idle_flag = false;
               //operation loop
               cout<<coutcolor_green<<"Planning... " <<coutcolor0<<endl;
               result = call_make_plan();
               setlogicstate(2);
              
         }

        else if ( getlogicstate() == 2 )
        {
            //check the plan
            cout<<"Check the plan from global planner"<<endl;
           
            if( result.poses.size() == 0 )
            {
                    cout<<coutcolor_red<<"Error in PATH ! "<<coutcolor0<<endl;
                    
                  
                    setlogicstate(3);
                    setsystemstate(-1); //wait
                    force_stop();
                    if(!IsRecoveryState)
                        say_message("Error in path generation");
            }
            else
            {
                globalPath = result;
                globalPathSize = globalPath.poses.size();
                cout<<coutcolor_green<<"get a new PATH from GPLANNER Points : "<< globalPathSize <<coutcolor0<<endl;
                setsystemstate(1);
                setlogicstate(4);
                force_stop();   

            }
        }

        else if ( getlogicstate() == 3 )
        {
        	say_message("Let me think");
            cout<<coutcolor_red<<" Recovery state " <<coutcolor0<<endl;
            //path error handler 
            //revocery state
           
            if(IsRecoveryState)
            {
            	if(IsHectorReset)
            	{
            		IsRecoveryState = false;
	            	IsHectorReset = false;
	            	setlastnavigationresult("GOAL IS UNREACHABLE");
	            	say_message("Goal is unreachable");
	            	setsystemstate(0);
	            	setlogicstate(0);
            	}
            	else
            	{
            		IsHectorReset = true;
	            	say_message("reseting SLAM");
            		reset_hector_slam();
            		update_hector_origin(position[0],position[1],tetha);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
                    clean_costmaps();   
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            		setlogicstate(1);
	            }

            }
            else
            {
            	IsRecoveryState = true;
            	say_message("reseting  Map");
            	clean_costmaps();
                boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
            	setlogicstate(1);
         	}
             
        }

        else if ( getlogicstate() == 4 ) //controlling mode
        {
        	IsRecoveryState = false;
        	IsHectorReset = false;
           // cout<<coutcolor_magenta<<" getlogicstate() == 4 " <<coutcolor0<<endl;

          

           if ( IsGoalReached == true)
           {
               cout<<coutcolor_red<<" Goal reached " <<coutcolor0<<endl;
               setlogicstate(0);
             
               continue;
           }
            
            result = call_make_plan();
            if( result.poses.size() != 0 )
             {
                int currentStep = GetCurrentStep();
                for(int i=0;i<result.poses.size()-1 && i+currentStep<globalPathSize-1;i++)
                {
                    if(GetDistance(result.poses[i].pose.position.x, result.poses[i].pose.position.y,globalPath.poses[i+currentStep].pose.position.x, globalPath.poses[i+currentStep].pose.position.y)>0.2)
                    {
                        globalPath = result;
                        globalPathSize = result.poses.size();
                         setsystemstate(1);
                        cout<<coutcolor_red<<"PATH changed : "<< globalPathSize <<coutcolor0<<endl;
                        break;
                    }
                }

             }
             else
             {
                    cout<<coutcolor_red<<"Error in PATH ! "<<coutcolor0<<endl;
                    setlogicstate(2);
             }
        

        }
     
    }
}

void LMTMoveBase::exe_slam(goal_data g)
{
    bool valid_point = false;
    //this function get goal and move robot to there
    if ( g.id == "")
    {
       ROS_INFO_STREAM(" MANUAL POINTx " <<  g.x);
       ROS_INFO_STREAM(" MANUAL POINTy " <<  g.y);
       ROS_INFO_STREAM(" MANUAL POINTtetha " <<  g.yaw);

       goalPos[0] = (float)(g.x) / 100; //cm => m
       goalPos[1] = (float)(g.y) / 100; //cm => m
       goalTetha = Deg2Rad(g.yaw); //deg => rad

       valid_point = true;

       ROS_INFO("EXECUTE MANUAL POINT");


    }
    else
    {
       int index = find_goal_byname(g.id);
       if ( index != -1)
       {
           goal_data data = (goal_data)goal_list.at(index);
           goalPos[0] = (float)(data.x)/ 100;
           goalPos[1] = (float)(data.y)/ 100;
           goalTetha = Deg2Rad(data.yaw);

           valid_point = true;
           ROS_INFO("EXECUTE LOCAL POINT");
       }
       else
       {
           valid_point = false;
           ROS_ERROR("LOCAL POINT NOT FOUND");
       }
    }

    if ( valid_point && getlogicstate() == 0)
    {
       
        // say_message("Got a new goal");
       setlogicstate(1);
    }  
}

void LMTMoveBase::exe_cancel()
{
	 setstatemutex(false);
	 setlogicstate(0,true);
     setsystemstate(0,true);
     
     cout<<"Cancel requested"<<endl;
     // say_message("cancel requested , operation canceled!");
     setlastnavigationresult("CANCEL REQUEST");

      force_stop();
    
}

int LMTMoveBase::sign(double data)
{
    if(data > 0) return 1;
    else if(data < 0) return -1;
    else return 0;
}

int LMTMoveBase::roundData(double data)
{
    if(data>=0)
        return ceil(data);
    else
        return floor(data);
}

double LMTMoveBase::GetToPointsAngle(double x1, double y1, double x2, double y2)
{
    return atan2(y2-y1,x2-x1);
}

void LMTMoveBase::ResetLimits()
{
    desireErrorX = normal_desire_errorX;
    desireErrorY = normal_desire_errorY;
    desireErrorTetha = normal_desire_errorTetha;

    LKpX = _normal_kp_linearX;
    WKp = _norma_kp_angular;

    maxLinSpeedX = _normal_max_linear_speedX;
    maxTethaSpeed = _normal_max_angular_speed;
}

void LMTMoveBase::ReduceLimits()
{
    desireErrorX = goal_desire_errorX;
    desireErrorY = goal_desire_errorY;
    desireErrorTetha = goal_desire_errorTetha;

    maxLinSpeedX = _goal_max_linear_speedX;
    maxTethaSpeed = _goal_max_angular_speed;
    LKpX = _goal_kp_linearX;
    WKp = _goal_kp_angular;

}

//0 => wait for goal
//2 => turn to target
//4 => go on path
//6 => turn to goal
//8 => reached

void LMTMoveBase::setrobotmove(bool value)
{
	isrobotmove = value;
}

bool LMTMoveBase::getrobotmove()
{
	return isrobotmove;
}

void LMTMoveBase::setlastnavigationresult(string value)
{
	last_navigation_result = value;
}

string LMTMoveBase::getlastnavigationresult()
{
	return last_navigation_result;
}

int LMTMoveBase::calc_next_point()
{
            bool isgoalnext = false;
            if ( step == globalPathSize-1)
            {
            	tempGoalTetha = goalTetha;
                on_the_goal = true;
               
            }
                
            if(step+step_size >= globalPathSize)
            {
                step = globalPathSize - 1;
                //we are very near to goal so next is the goalstep = globalPathSize-1;
                tempGoalPos[0] = goalPos[0];
                tempGoalPos[1] = goalPos[1];
                //tempGoalTetha = goalTetha;
                tempGoalTetha = GetToPointsAngle(position[0], position[1], tempGoalPos[0], tempGoalPos[1]);
               
                if (tempGoalTetha < 0) tempGoalTetha += 2*M_PI;

                isgoalnext = true;
                //on_the_goal = true;

                cout<<coutcolor_magenta<<"goal calc"<<coutcolor0<<endl;

               
            }
            else
            {
                //select the point in next 20 step and calc all errors
                step +=step_size;

                tempGoalPos[0] = globalPath.poses[step].pose.position.x;
                tempGoalPos[1] = globalPath.poses[step].pose.position.y;
                tempGoalTetha = GetToPointsAngle(position[0], position[1], globalPath.poses[step].pose.position.x, globalPath.poses[step].pose.position.y);
                if (tempGoalTetha < 0) tempGoalTetha += 2*M_PI;


                 cout<<coutcolor_magenta<<"step calc"<<coutcolor0<<endl;
                 
            }

           

            return isgoalnext;
}

void LMTMoveBase::errors_update()
{
            //calc errorX , errorY , errorTetha 
            errorX = tempGoalPos[0]-position[0];
            errorY = tempGoalPos[1]-position[1];
            errorTetha = tempGoalTetha-tetha;

            if (errorTetha >= M_PI) errorTetha =  errorTetha - 2*M_PI;
            if (errorTetha < -M_PI) errorTetha =  errorTetha + 2*M_PI;
            //?
            if (errorTetha > 0.833*M_PI) errorTetha = 0.833*M_PI;
            if (errorTetha < -0.833*M_PI) errorTetha = -0.833*M_PI;

            errorX_R = cos(tetha)*errorX+sin(tetha)*errorY;
            errorY_R = -sin(tetha)*errorX+cos(tetha)*errorY;

            float x1 = position[0];
            float y1 = position[1];
            float x2 = goalPos[0];
            float y2 = goalPos[1];
            float d_1 = (x2 - x1); 
            float d_2 = (y2 - y1);
            distacne_to_goal = d_1 * d_1 + d_2 * d_2;
            distacne_to_goal = sqrt(distacne_to_goal);
}

void LMTMoveBase::publish_info()
{
        info_counter++;
        if ( info_counter>50)
        {
            info_counter= 0;

            cout << "Speed: " << xSpeed << " - " << ySpeed << " - " << tethaSpeed << endl;
            cout << "Step: " << step << endl;
            cout << "TError: " << errorX << " - " << errorY << " - " << errorTetha << endl;
            cout << "Goal: " << fabs(goalPos[0]-position[0]) << " - " << fabs(goalPos[1]-position[1]) << " - " << fabs(goalTetha-tetha)<< endl; 
        }
}

void LMTMoveBase::controller_update(int x,bool y,bool theta)
{
    if ( x == 1)
    xSpeed = (fabs(errorX_R*LKpX)<=maxLinSpeedX)?(errorX_R*LKpX):sign(errorX_R)*maxLinSpeedX;
    else if ( x == 0)
    xSpeed = 0;
    else if ( x == 2)
    xSpeed = maxLinSpeedX / 2;

  

    if ( theta )
    tethaSpeed = (fabs(errorTetha*WKp)<=maxTethaSpeed)?(errorTetha*WKp):sign(errorTetha)*maxTethaSpeed;
    else
    tethaSpeed = 0;

    //cout<<"X : "<<xSpeed<<"T : "<<tethaSpeed<<endl;
    send_omni(xSpeed,0,tethaSpeed); 
}

void LMTMoveBase::PathFwr()
{
	
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    force_stop();
   
    #ifndef VIRTUALMODE
    say_message("LMT Move Base Started");
    #else
    say_message("LMT Move Base Started in virtual mode");
    #endif

    while (ros::ok() && !App_exit)
    {
        errors_update();

        if ( getsystemstate() == 0 )
        {
        	if ( getstatemutex() == false )
        	{
        		boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        		setstatemutex(true);
        	}
        	setrobotmove(false);
        }
        else
        {
        	setrobotmove(true);
        }

        if ( getsystemstate() == -1) //wait state
        {

           boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }
        if ( getsystemstate() == 0)
        {

            if ( wait_flag == false)
            {
               wait_flag = true;
               cout<< coutcolor_green <<"Wait for goal ! ... "<< coutcolor0 <<endl;
               say_message("I am waiting for new goal");
            }
          
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        }
        else
        if ( getsystemstate() == 1)
        {
           force_stop();
           IsGoalReached = false;
           on_the_goal = false;
           ResetLimits();
           step = 0;
           wait_flag = false;
           cout<<"State = 1 -turn to target-"<<endl;

            bool resutl = calc_next_point();
            if ( resutl)
            {
                //next is the goal !
                cout<<"Next is goal =>3"<<endl;
              
                setsystemstate(3);
            }
            else
            {
                cout<<"Next is step =>2"<<endl;
                setsystemstate(2);
            }

        }
        else
        if ( getsystemstate() == 2)
        {
            //turn to goal <loop>
            
            if(fabs(errorTetha)<=desireErrorTetha)
            {
                
                cout<<"DONE ! "<<tetha<<" "<<tempGoalTetha<<" "<<errorTetha<<endl;
                setsystemstate(3);
                force_stop();
                
            }
            else
            {
                controller_update(0,false,true);
            }

        }
        else
        if ( getsystemstate() == 3)
        {
           cout<<"State = 3 -go on path- Step = "<<step<<endl;
           setsystemstate(4);
        }
        else
        if ( getsystemstate() == 4)
        {
           bool next = false;

              double dist_error = sqrt(errorX_R*errorX_R + errorY_R*errorY_R);
              //cout<<dist_error<<" "<<desireErrorX<<" "<<errorTetha<<" "<<desireErrorTetha<<endl;

              bool okk = false;
              // if ( step < 40 || (globalPathSize - step) < 40 )	
              // {
              //      if ( fabs(dist_error)<=desireErrorX  )
              //      {
              //      	  okk = true;
              //      }
              // }
              // else
              // {
	              	if ( fabs(dist_error)<=desireErrorX && fabs(errorTetha)<=desireErrorTetha )
	              	{
	              		okk = true;
	              	}
              //}
              //Middle Way
              if(okk) //distance & angle is important
              {
                bool resutl = calc_next_point();
                if ( resutl )
                {
                    //next is the goal
                    ReduceLimits();

                    if ( on_the_goal )
                    {
                       
                       setsystemstate(5);
                    }
                    else
                    {
                       setsystemstate(3);
                    }
                    
                }
                else
                {
                    cout<<"Temp point reached"<<endl;
                    setsystemstate(3);
                }
              
           }
           else
           {

                
                	if ( fabs(errorTetha)<=desireErrorTetha / 2 )
                	{
                		if ( fabs(errorTetha)<=desireErrorTetha / 4 )
                		{
                		   controller_update(2,false,false); //fixed (F,Y,T)
                	    }
                	    else
                	    {
                	       controller_update(1,false,false); //fixed (F,Y,T)
                	    }
                	}
                	else
                	{
                		//if we are going away ! 
                		controller_update(0,false,true); //p  fixed (X,Y,T)
                	}

  
            }
        }
        else
        if ( getsystemstate() == 5)
        {
        	tempGoalTetha = goalTetha;
        	errors_update();
        	cout<<"Check ! "<<tetha<<" "<<tempGoalTetha<<" "<<errorTetha<<endl;
            cout<<"State = 5 -turn to goal-"<<endl;
            setsystemstate(6);
        }
        else
        if ( getsystemstate() == 6)
        {
            if(fabs(errorTetha)<=desireErrorTetha)
            {
                cout<<"DONE ! "<<tetha<<" "<<tempGoalTetha<<" "<<errorTetha<<endl;
                setsystemstate(7);
                force_stop();
            }
            else
            {
                controller_update(0,false,true);
            }
        }
        else
        if ( getsystemstate() == 7)
        {
            cout<<"State = 7 -goal reached-"<<endl;
            setsystemstate(8);
        }
        else
        if ( getsystemstate() == 8)
        {
            cout<<"Finished !"<<endl;
            
            IsGoalReached = true;

            on_the_goal = false;
            wait_flag = false;
            force_stop();
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
            setsystemstate(0);
            setlastnavigationresult("GOAL REACHED");
            say_message("Goal reached");
        }

        //publish_info();
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
    }
}

void LMTMoveBase::GetCostmap(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
	if(!IsCmValid)
	{
		costmap = *msg;
		IsCmValid = true;
	}
}

bool LMTMoveBase::calc_error(double x1,double y1,double t1,double x2,double y2,double t2,double delta_t)
{
   bool valid = true;
   double v1 = fabs(x2-x1) / delta_t;
   double v2 = fabs(y2-y1) / delta_t;

   double dt = t2-t1;
   if (dt >= M_PI) dt =  dt - 2*M_PI;
   if (dt < -M_PI) dt =  dt + 2*M_PI;

   double v3 = fabs(dt) / delta_t;

   if ( v1 > maxLinSpeedX + 0.1 ) valid = false;
   else if ( v3 > maxTethaSpeed + 0.1 ) valid = false;

   // cout<<v1<<" "<<v2<<" "<<v3<<" "<<delta_t<<endl;

   return valid;
}

void LMTMoveBase::hector_problem_detected()
{
    if ( getlogicstate() != -1)
    {
        
            tempLogicState = getlogicstate();
            setlogicstate(-1);

            tempSystemState = getsystemstate();
            setsystemstate(-1);

            force_stop();
        }
}

void LMTMoveBase::chatterCallbackPoints(const lmt_movebase::PointList::ConstPtr &msg)
{
    if ( is_sim == false ) return;

    if ( goal_list.size() == 0)
    {
        for ( int i = 0 ; i < msg->poses.size() ; i++)
        {
                 goal_data gdata;

                 gdata.id = std::to_string(i+1);
                 gdata.x = msg->poses[i].position.x;
                 gdata.y = msg->poses[i].position.z;
                 gdata.yaw = 0;
                 gdata.height = msg->poses[i].position.y;

                 goal_list.push_back(gdata);
        }
    }
               
}

void LMTMoveBase::chatterCallbackTags(const lmt_movebase::PointList::ConstPtr &msg)
{
        std::lock_guard<std::mutex> guard(tag_mutex);
        tag_list.clear();

        if ( tag_list.size() == 0)
        {
            for ( int i = 0 ; i < msg->poses.size() ; i++)
            {
                     goal_data gdata;

                     gdata.id = std::to_string(i+1);
                     gdata.x = msg->poses[i].position.x;
                     gdata.y = msg->poses[i].position.z;
                     gdata.yaw = 0;
                     gdata.height = msg->poses[i].position.y;

                     tag_list.push_back(gdata);
            }

            ROS_INFO_STREAM("Updated TAG SIZE " << tag_list.size());
        }
}

void LMTMoveBase::GetAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
	amclCovariance = msg->pose.covariance;
	amclPosition[0] = msg->pose.pose.position.x;
	amclPosition[1] = msg->pose.pose.position.y;
    amclOrientation[0] = msg->pose.pose.orientation.x;
    amclOrientation[1] = msg->pose.pose.orientation.y;
    amclOrientation[2] = msg->pose.pose.orientation.z;
    amclOrientation[3] = msg->pose.pose.orientation.w;
    amclTetha = Quat2Rad(amclOrientation);
}

void LMTMoveBase::GetPos(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    position[0] = msg->pose.position.x;
    position[1] = msg->pose.position.y;
    orientation[0] = msg->pose.orientation.x;
    orientation[1] = msg->pose.orientation.y;
    orientation[2] = msg->pose.orientation.z;
    orientation[3] = msg->pose.orientation.w;
    tetha = Quat2Rad(orientation);
    
    if ( tetha < 0)  tetha += 2*M_PI;


    //cout<<"position : "<<position[0]<<" "<<position[1]<<" "<<tetha<<endl;
}

void LMTMoveBase::CheckHectorStatus(const std_msgs::Bool::ConstPtr &msg)
{ 
    if(msg->data == false)
    {
        hector_problem_detected(); 
    }
}

void LMTMoveBase::chatterCallback_ttsfb(const std_msgs::String::ConstPtr &msg)
{
    if(!isttsready && msg->data == sayMessageId)
    {
        cout<<coutcolor_brown<<"text to speech is ready!"<<coutcolor0<<endl;
        isttsready = true;
    }
}

bool LMTMoveBase::checkcommand(lmt_msgs::command::Request  &req,lmt_msgs::command::Response &res)
{

	ROS_INFO("Service Request....");

    std::string _cmd = req.command;
    std::string _id = req.id;
    int _value1 = req.value1;
    int _value2 = req.value2;
    int _value3 = req.value3;

    if ( _cmd == "reload_points" && is_sim == false)
    {
    	read_file();
    }

    if ( _cmd == "save_map")
    {
    	LMTmapengine_savemap();
    }

    if ( _cmd == "load_map")
    {
    	LMTmapengine_loadmap();
    }

    if ( _cmd == "exe")
    {

        goal_data g;
        g.id = _id;
        g.x = _value1;
        g.y = _value2;
        g.yaw = _value3;

        exe_slam(g);
    }

    if ( _cmd == "cancel")
    {

        exe_cancel();
    }

    if ( _cmd == "reset_hector")
    {
        reset_hector_slam();
    }

    if ( _cmd == "update_hector_origin")
    {

        update_hector_origin(0,0,0);
    }

    res.result = "done";
    return true;
}

void LMTMoveBase::test_vis()
{

    visualization_msgs::Marker points , points2 , points3 , points4 ;

    points.header.frame_id =  "map";
    points.header.stamp = ros::Time::now();
    points.ns = "points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.color.g = 1.0f;
    points.color.a = 1.0;

    points2.header.frame_id =  "map";
    points2.header.stamp = ros::Time::now();
    points2.action = visualization_msgs::Marker::ADD;
    points2.id = 1;
    points2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    points2.scale.x = 1;
    points2.scale.y = 1;
    points2.scale.z = 0.4;
    points2.color.b = 0.5;
    points2.color.r = 1;
    points2.color.a = 1;

    points3.header.frame_id =  "map";
    points3.header.stamp = ros::Time::now();
    points3.action = visualization_msgs::Marker::ADD;
    points3.id = 2;
    points3.type = visualization_msgs::Marker::ARROW;
    points3.scale.x = 0.5;
    points3.scale.y = 0.05;
    points3.scale.z = 0.05;
    points3.color.b = 0.5;
    points3.color.r = 1;
    points3.color.a = 1;

    points4.header.frame_id =  "map";
    points4.header.stamp = ros::Time::now();
    points4.ns = "tags";
    points4.action = visualization_msgs::Marker::ADD;
    points4.pose.orientation.w = 1.0;
    points4.id = 3;
    points4.type = visualization_msgs::Marker::POINTS;
    points4.scale.x = 0.1;
    points4.scale.y = 0.1;
    points4.color.r = 0.0f;
    points4.color.b = 1.0f;
    points4.color.a = 1.0;


     // Create the vertices for the points and lines
    for (int i = 0; i < globalPath.poses.size(); i += step_size)
    {
      
      geometry_msgs::Point p;
      p.x = globalPath.poses[i].pose.position.x;
      p.y = globalPath.poses[i].pose.position.y;
      p.z = 0;

      points.points.push_back(p);
      
    }

    for ( int i = 0 ; i < goal_list.size() ; i++)
    {
     
     points2.pose.position.x =  (float)goal_list[i].x / 100;
     points2.pose.position.y =  (float)goal_list[i].y / 100;
     points2.pose.position.z = 0;
     points2.text = goal_list[i].id;
     points2.ns = "text " + goal_list[i].id;
     marker_pub2.publish(points2);


     points3.pose.position.x =  (float)goal_list[i].x / 100;
     points3.pose.position.y =  (float)goal_list[i].y / 100;
     points3.pose.position.z = 0; 
     points3.pose.orientation = tf::createQuaternionMsgFromYaw(Deg2Rad(goal_list[i].yaw));
     points3.ns = "arrow " + goal_list[i].id;

     marker_pub3.publish(points3);
     
    }

    std::lock_guard<std::mutex> guard(tag_mutex);
    
    for ( int i = 0 ; i < tag_list.size() ; i++)
    {
          geometry_msgs::Point p;
          p.x = (float)tag_list[i].x / 100;
          p.y = (float)tag_list[i].y / 100;
          p.z = (float)tag_list[i].height / 100;

          points4.points.push_back(p);        
    }

    marker_pub4.publish(points4);
    marker_pub.publish(points);
}

void LMTMoveBase::vis_thread()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    while (ros::ok() && App_exit == false)
    {
    	test_vis();
        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
    }
}

void LMTMoveBase::Localization_thread()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    int counter = 0;
    bool isFirstTime = true;

    double hector_offset[3] = {0};
    int mode = 0;
    int loc_mode = 0;

    // while (ros::ok() && App_exit == false)
    // {
        

    //         position[0] = tempPosition[0];
    //         position[1] = tempPosition[1];
    //         tetha = tempTetha;

        


    //     }


    //     boost::this_thread::sleep(boost::posix_time::milliseconds(50)); //20hz
    // }
}

void LMTMoveBase::init()
{

ROS_INFO("LMTMoveBase Version 2.2 :*");
cout<<"1"<<endl;
coutcolor0 = "\033[0;0m";
coutcolor_red = "\033[0;31m";
coutcolor_green = "\033[0;32m";
coutcolor_blue = "\033[0;34m";
coutcolor_magenta = "\033[0;35m";
coutcolor_brown = "\033[0;33m";
say_enable = true;
App_exit = false;
IsCmValid = false;
IsGoalReached = false;
IsRecoveryState = false;
IsHectorReset = false;
isttsready = true;
statemutex = true;

globalPathSize = 0;
temp_path_size = 0;
xSpeed=0;
ySpeed=0;
tethaSpeed=0;
desireErrorX = normal_desire_errorX;
desireErrorY = normal_desire_errorY;
desireErrorTetha = normal_desire_errorTetha;
errorX = 0;
errorY = 0;
errorTetha = 0;
errorX_R = 0;
errorY_R = 0;

LKiX = 0;
LKiY = 0;
WKi = 0;
step = 0;
position[2] = {0};
hectorPosition[2] = {0};
tempPosition[2] = {0};
orientation[4] = {0};
amclPosition[2] = {0};
amclOrientation[4] = {0};
tetha = 0;
hectorTetha=0;
amclTetha=0;
tempTetha = 0;
tempGoalPos[2] = {0};
tempGoalTetha = 0;
goalPos[2] = {0};
goalOri[4] = {0};
goalTetha = 0;
distacne_to_goal = 0;
maxErrorX = 0;
maxErrorY = 0;
maxErrorTetha = 0;
info_counter = 0;
system_state = 0;
logic_state = 0;
on_the_goal = false;
step_size  = 40;
wait_flag = false;
idle_flag = false;
isrobotmove = false;
last_navigation_result = "";
f = 0.0;

read_config();

maxLinSpeedX = _normal_max_linear_speedX;
maxTethaSpeed = _normal_max_angular_speed;
LKpX = _normal_kp_linearX;
WKp = _norma_kp_angular;

std::cout<<"READ CONFIG"<<std::endl;

std::cout<<"maxLinSpeedX : "<<maxLinSpeedX<<std::endl;
std::cout<<"maxTethaSpeed : "<<maxTethaSpeed<<std::endl;
std::cout<<"LKpX : "<<LKpX<<std::endl;
std::cout<<"WKp : "<<WKp<<std::endl;

//=============================================

std::cout<<"Goal maxLinSpeedX : "<<_goal_max_linear_speedX<<std::endl;
std::cout<<"Goal maxTethaSpeed : "<<_goal_max_angular_speed<<std::endl;
std::cout<<"Goal LKpX : "<<_goal_kp_linearX<<std::endl;
std::cout<<"Goal WKp : "<<_goal_kp_angular<<std::endl;

cout<<"DONE"<<endl;
    //============================================================================================
    sub_handles[0] = node_handle.subscribe("/slam_out_pose", 10, &LMTMoveBase::GetPos,this);
    //============================================================================================
    sub_handles[1] = node_handle.subscribe("/move_base/global_costmap/costmap", 10, &LMTMoveBase::GetCostmap,this);
    //============================================================================================
    sub_handles[2] = node_handle.subscribe("/HectorStatus", 10, &LMTMoveBase::CheckHectorStatus,this);
    //============================================================================================
    sub_handles[4] = node_handle.subscribe("/amcl_pose", 10, &LMTMoveBase::GetAmclPose,this);

    sub_map_points = node_handle.subscribe("points",1, &LMTMoveBase::chatterCallbackPoints,this);
    sub_map_tags = node_handle.subscribe("tags",1, &LMTMoveBase::chatterCallbackTags,this);

    //============================================================================================
    mycmd_vel_pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    //mycmd_vel_pub = node_handle.advertise<geometry_msgs::Twist>("movo/teleop/cmd_vel", 10);
    pub_slam_origin = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("/slam_origin", 1);
    pub_slam_reset = node_handle.advertise<std_msgs::String>("syscommand", 1);
    //============================================================================================
    ros::ServiceServer service_command = n_service.advertiseService("LMTMovebase/command", &LMTMoveBase::checkcommand,this);
    //============================================================================================
    pub_tts = node_handle.advertise<std_msgs::String>("/texttospeech/message", 10);
    //============================================================================================
    pub_current_goal = node_handle.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );
    pub_alarm = node_handle.advertise<std_msgs::Int32>("lowerbodycore/alarm",10);
    pub_move = node_handle.advertise<std_msgs::Bool>("lowerbodycore/isrobotmove", 10);
    //============================================================================================
    marker_pub =  node_handle.advertise<visualization_msgs::Marker>("visualization_marker_steps", 10);
    marker_pub2 =  node_handle.advertise<visualization_msgs::Marker>("visualization_marker_goals", 10);
    marker_pub3 =  node_handle.advertise<visualization_msgs::Marker>("visualization_marker_goals_arrow", 10);
    marker_pub4 =  node_handle.advertise<visualization_msgs::Marker>("visualization_marker_tags", 10);
    //============================================================================================
    client_makeplan = node_handle.serviceClient<nav_msgs::GetPlanRequest>("move_base/make_plan");
 	client_resetcostmap = node_handle.serviceClient<std_srvs::EmptyRequest>("move_base/clear_costmaps");
 	client_map_save = node_handle.serviceClient<std_srvs::EmptyRequest>("LMTmapengenine/save");
    client_map_load = node_handle.serviceClient<std_srvs::EmptyRequest>("LMTmapengenine/load");
    //============================================================================================
    sub_handles[4] = node_handle.subscribe("/texttospeech/queue", 10, &LMTMoveBase::chatterCallback_ttsfb,this);
    say_service = node_handle.serviceClient<lmt_msgs::command>("texttospeech/say");
    pub_voice_cmd = node_handle.advertise<std_msgs::String>("/movo/voice/text",1);
    //============================================================================================
    
    if ( is_sim == false)
    read_file();
  
    ROS_INFO("Init done");
}

void LMTMoveBase::playVoice(std::string text)
{
    ROS_WARN_STREAM("Play voice : " << text);
    std_msgs::String msg;
    msg.data =  text;
    pub_voice_cmd.publish(msg);
}

void LMTMoveBase::kill()
{
	_thread_PathFwr.interrupt();
    _thread_PathFwr.join();

    _thread_Logic.interrupt();
    _thread_Logic.join();

    _thread_Vis.interrupt();
    _thread_Vis.join();

    _thread_Localization.interrupt();
    _thread_Localization.join();
}
