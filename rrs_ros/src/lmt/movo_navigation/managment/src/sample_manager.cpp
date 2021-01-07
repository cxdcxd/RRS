#include "client_manager.h"

bool appExit = false;
client_manager *gClinet;

using namespace std;

void TEST1()
{
    cout<<"Test 1 , run upperbody "<<endl;

    int x = gClinet->task_start("upperbody",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    x = gClinet->task_stop("upperbody",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    cout<<"Test 2 , run face "<<endl;

    x = gClinet->task_start("face",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    x = gClinet->task_stop("face",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    cout<<"Test 3 , run downerbody "<<endl;

    x = gClinet->task_start("downerbody",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    x = gClinet->task_stop("downerbody",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    cout<<"Test 4 , run speech "<<endl;

    x = gClinet->task_start("speech",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    x = gClinet->task_stop("speech",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    cout<<"Test 5 , run sound "<<endl;

    x = gClinet->task_start("sound",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    x = gClinet->task_stop("sound",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    cout<<"Test 6 , run skeleton "<<endl;

    x = gClinet->task_start("skeleton",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    x = gClinet->task_stop("skeleton",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    cout<<"Test 7 , run object "<<endl;

    x = gClinet->task_start("object",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    x = gClinet->task_stop("object",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    cout<<"Test 8 , run stair "<<endl;

    x = gClinet->task_start("stair",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    x = gClinet->task_stop("stair",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

}


void TEST2()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    cout<<"Test 2 , run face "<<endl;

    int x = gClinet->task_start("face",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    cout<<"Test 6 , run skeleton "<<endl;

    x = gClinet->task_start("skeleton",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    x = gClinet->task_stop("skeleton",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    x = gClinet->task_stop("face",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

}

void TEST3()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    cout<<"Test 2 , run skeleton "<<endl;

    int x = gClinet->task_start("skeleton",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    cout<<"Test 6 , run face "<<endl;

    x = gClinet->task_start("face",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    x = gClinet->task_stop("face",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    x = gClinet->task_stop("skeleton",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

}

void TEST4()
{
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    cout<<"Test 2 , run face "<<endl;

    int x = gClinet->task_start("face",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    cout<<"Test 6 , run object "<<endl;

    x = gClinet->task_start("object",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    cout<<"Test 6 , run stair "<<endl;

    x = gClinet->task_start("stair",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));


    x = gClinet->task_stop("stair",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    x = gClinet->task_stop("face",3,10);
    if ( x == 0 )  cout<<"done"<<endl;  else  cout<<"failed"<<" "<<x<<endl; cout<<"============"<<endl;

    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    x = gClinet->task_stop("object",3,10);



}

void logic(int select_ramp_stair)
{
    while( !appExit )
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
        if ( gClinet == NULL ) continue;

        //TEST1();
        // TEST2();
        //TEST3();
        TEST4();

        break;
    }
}

int main(int argc, char **argv)
{
    boost::thread _thread_logic(&logic);

    ros::init(argc, argv, "client_manager");
    std::cout<<"client manager started done"<<std::endl;

    ros::Time::init();
    ros::Rate loop_rate(20);
    
    client_manager Client;
    gClinet = &Client;

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    appExit = true;
    _thread_logic.interrupt();
    _thread_logic.join();

    return 0;
}
