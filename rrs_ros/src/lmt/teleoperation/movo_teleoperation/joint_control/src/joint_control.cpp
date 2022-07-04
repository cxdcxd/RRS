
#include "joint_control/joint_control.hh"

using namespace std;

namespace roboland
{

JointControl::JointControl() :
    app_exit(false),
    thread_main(&JointControl::thrMain,this)
{
   
}

JointControl::~JointControl()
{
   kill();
}


void JointControl::thrMain()
{
   wait(1000);

   while ( app_exit == false )
   {
       wait(1000);
   }
}

 

void JointControl::wait(int ms)
{
    //wait
    boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

void JointControl::kill()
{
   this->thread_main.interrupt();
   this->thread_main.~thread();
}


}
