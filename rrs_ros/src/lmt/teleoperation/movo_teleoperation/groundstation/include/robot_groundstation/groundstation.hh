//2016

#ifndef CAR_GROUNDSTATION_HH_
#define CAR_GROUNDSTATION_HH_

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

#include <robot_groundstation/network.hh>

#include <Main.pb.h>
#include <functional>

using namespace std;

namespace roboland
{

class Groundstation
{

public:
  Groundstation (string groundstation_remote_ip, int argc, char *argv[]);
  ~Groundstation ();

  Network *net_interface;                //Get MovoCommand and Send MovoStatus
  Network *net_interface_camera;         //Get <nothing>  Send Camera
  Network *net_interface_haptic_right;   //Get haptic device position and Send force for rendering
  Network *net_interface_haptic_left;    //Get haptic device position and Send force for rendering
  Network *net_interface_ml;             //Machine Learning Interface

  bool is_connected_main;
  int time_out_main;

  bool is_connected_haptic_right;
  int time_out_haptic_right;

  bool is_connected_haptic_left;
  int time_out_haptic_left;

  bool is_connected_ml;
  int time_out_ml;
  
  bool app_exit;
  boost::thread thread_main;

  void kill();
  void receiveCallback(char * data, int size, int index);
  void receiveCallbackCamera(char * data, int size, int index);
  void receiveCallbackHapticRight(char * data, int size, int index);
  void receiveCallbackHapticLeft(char * data, int size, int index);
  void receiveCallbackML(char * data, int size, int index);

  void sendStatus(MovoStatus msg);
  void sendCamera(char * buffer,int size);
  void sendForceRight(HapticRender msg);
  void sendForceLeft(HapticRender msg);
  void sendML(HapticCommands msg);

  void thrMain();
  void wait(int ms);
  void sendAlive();

  std::function<void (MovoCommand cmd)> callBackCMD;
  std::function<void (HapticCommand cmd)> callBackHapticPositionRight;
  std::function<void (HapticCommand cmd)> callBackHapticPositionLeft;
  std::function<void (HapticCommand cmd)> callBackML;

  


};

} 
#endif

