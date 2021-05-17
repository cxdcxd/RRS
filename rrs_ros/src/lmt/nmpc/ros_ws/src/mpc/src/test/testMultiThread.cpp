#include <iostream>
#include <thread>
#include <chrono>
#include <ros/ros.h>

using namespace std;

thread::id main_thread_id = this_thread::get_id();

void hello()
{
    std::cout << "Hello Concurrent World\n";
    if (main_thread_id == std::this_thread::get_id())
        std::cout << "This is the main thread.\n";
    else
        std::cout << "This is not the main thread.\n";
}
void pause_thread(int n,int m, int* l)
{
//  std::this_thread::sleep_for (std::chrono::seconds(n));
//  std::cout << "pause of " << n << " seconds ended in thead " << m << endl;
  (*l) = n+m;
}

void loop_thread(vector<double>* v)
{
  while(ros::ok())
  {
    for(int i=0;i<3;i++)
      cout << (*v)[i] << " ";
    cout << endl;
    this_thread::sleep_for(chrono::seconds(1));
  }
  cout << "loop thread ended." << endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "testMultiThread");
//  ros::NodeHandle n;
//  double v[3] = {1,2,3};
  vector<double> v = {1,2,3};
//  thread t1(loop_thread, &v);
  shared_ptr<thread> t1;
  t1 = make_shared<thread>(loop_thread, &v);
  this_thread::sleep_for(chrono::seconds(4));
  v[0] = 4;
  v[1] = 5;
  v[2] = 6;
  this_thread::sleep_for(chrono::seconds(4));
  ros::shutdown();
  t1->join();
  cout << "Main thread ended" << endl;
  return 0;
}
