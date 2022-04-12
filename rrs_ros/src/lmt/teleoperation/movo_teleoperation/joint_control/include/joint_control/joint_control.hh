//2019

#ifndef _JOINT_CONTROL_HH_
#define _JOINT_CONTROL_HH_

//STD
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <functional>

//BOOST
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <functional>

using namespace std;

namespace roboland
{

class JointControl
{

public:
  JointControl ();
  ~JointControl ();

  bool app_exit;
  boost::thread thread_main;

  void kill();
  void thrMain();
  void wait(int ms);

  std::function<void (int value)> callBackCMD;
};

} 
#endif

