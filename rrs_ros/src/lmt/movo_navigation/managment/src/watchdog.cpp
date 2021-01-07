
#include "rosspawn/watchdog.h"
#include "std_msgs/String.h"



void watchdog::wd()
{

    while(appexit == false)
    {
        //std::cout<<"thread ..."<<std::endl;
        boost::this_thread::sleep(boost::posix_time::milliseconds(timeout));
        if ( signal == false )
        {
            wd_counter++;
        }
        else
        {
            wd_counter = 0;
        }

        if ( wd_counter > wd_max )
        {
            ack = false;
            wd_counter = 0;
        }

        signal = false;
    }
}

void watchdog::callbackRawImage_kinectrgb(const sensor_msgs::Image::ConstPtr& msg)
{
    //std::cout<<"get"<<std::endl;
    ack = true;
    signal = true;
}

void watchdog::callbackRaw_ack(const std_msgs::String::ConstPtr& msg)
{
    //std::cout<<"get"<<std::endl;
    ack = true;
    signal = true;
}

void watchdog::callbackRaw_motor(const dynamixel_msgs::MotorStateList::ConstPtr &msg)
{
    //std::cout<<"get"<<std::endl;
    ack = true;
    signal = true;
}

void watchdog::callbackRaw_marker(const visualization_msgs::Marker::ConstPtr& msg)
{
    //std::cout<<"get"<<std::endl;
    ack = true;
    signal = true;
}

void watchdog::init()
{

    boost::thread _thread_wd(&watchdog::wd,this);
    global_wd = &_thread_wd;
    ros::NodeHandle node_handles[10];
    //=================================
    if ( ack_mode == "image")
    sub_kinect = node_handles[0].subscribe(callback_name, 1, &watchdog::callbackRawImage_kinectrgb , this);

    if ( ack_mode == "string")
    sub_ack = node_handles[0].subscribe(callback_name, 1, &watchdog::callbackRaw_ack , this);

    if ( ack_mode == "motor")
    sub_ack = node_handles[0].subscribe(callback_name, 1, &watchdog::callbackRaw_motor , this);

    if ( ack_mode == "marker")
    sub_ack = node_handles[0].subscribe(callback_name, 1, &watchdog::callbackRaw_marker , this);
}



void watchdog::kill()
{
     appexit = true;
     global_wd->interrupt();
     global_wd->join();
}
