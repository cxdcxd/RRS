
#include "robot_groundstation/groundstation.hh"

using namespace std;

namespace roboland
{

Groundstation::Groundstation(std::string groundstation_remote_ip,int argc, char *argv[]) :
    app_exit(false),
    thread_main(&Groundstation::thrMain,this)
{
    is_connected_main = false;
    time_out_main = 5;

    is_connected_haptic_right = false;
    time_out_haptic_right = 5;

    is_connected_haptic_left = false;
    time_out_haptic_left = 5;

    net_interface = new Network(8001,8000,8002,"main",groundstation_remote_ip,false,5);
    net_interface->dataCallBackFunction = std::bind(&Groundstation::receiveCallback, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3);

    net_interface_camera = new Network(9001,9000,9002,"camera",groundstation_remote_ip,false,5);
    net_interface_camera->dataCallBackFunction = std::bind(&Groundstation::receiveCallbackCamera, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3);

    net_interface_haptic_right = new Network(7050,7051,7052,"haptics_right",groundstation_remote_ip,false,5);
    net_interface_haptic_right->dataCallBackFunction = std::bind(&Groundstation::receiveCallbackHapticRight, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3);

    net_interface_haptic_left = new Network(8050,8051,8052,"haptics_left",groundstation_remote_ip,false,5);
    net_interface_haptic_left->dataCallBackFunction = std::bind(&Groundstation::receiveCallbackHapticLeft, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3);

    net_interface_ml = new Network(4050,4051,4052,"ml",groundstation_remote_ip,false,5);
    net_interface_ml->dataCallBackFunction = std::bind(&Groundstation::receiveCallbackML, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3);
   
}

Groundstation::~Groundstation()
{
   kill();
}

void Groundstation::receiveCallback(char * data,int size,int index)
{
  MovoCommand msg;
  msg.ParseFromArray(data,size);

  time_out_main = 5;
  is_connected_main = true;

  if ( callBackCMD )
        callBackCMD(msg);
}

void Groundstation::receiveCallbackML(char * data,int size,int index)
{
  HapticCommand msg;
  msg.ParseFromArray(data,size);

  time_out_ml = 5;
  is_connected_ml = true;

  if ( callBackML )
        callBackML(msg);
}

void Groundstation::receiveCallbackCamera(char * data,int size,int index)
{
  //dummy
}

void Groundstation::receiveCallbackHapticRight(char * data,int size,int index)
{
  HapticCommand msg;
  msg.ParseFromArray(data,size);

  time_out_haptic_right = 5;
  is_connected_haptic_right = true;

  if ( callBackHapticPositionRight )
  	    callBackHapticPositionRight(msg);

}

void Groundstation::receiveCallbackHapticLeft(char * data,int size,int index)
{
  HapticCommand msg;
  msg.ParseFromArray(data,size);

  time_out_haptic_left = 5;
  is_connected_haptic_left = true;

  if ( callBackHapticPositionLeft )
        callBackHapticPositionLeft(msg);
}

void Groundstation::thrMain()
{
   wait(1000);

   while ( app_exit == false )
   {
      sendAlive();
      wait(1000);
   }
}

void Groundstation::sendAlive()
{
   MovoStatus msg;
   msg.set_version(100);

   sendStatus(msg);

   time_out_main--;
   if ( time_out_main <= 0)
   {
       time_out_main = 5;
       is_connected_main = false;
   }
}

void Groundstation::wait(int ms)
{
    //wait
    boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

void Groundstation::sendStatus(MovoStatus msg)
{
   int size = msg.ByteSize();
   char buffer[size];
   msg.SerializeToArray(buffer,size);
   net_interface->tcpWrite(buffer,size);
}

void Groundstation::sendCamera(char * buffer,int size)
{
   net_interface_camera->tcpWrite(buffer,size);
}

void Groundstation::sendForceRight(HapticRender msg)
{
	int size = msg.ByteSize();
  char buffer[size];
  msg.SerializeToArray(buffer,size);
	net_interface_haptic_right->tcpWrite(buffer,size);
}

void Groundstation::sendForceLeft(HapticRender msg)
{
  int size = msg.ByteSize();
  char buffer[size];
  msg.SerializeToArray(buffer,size);
  net_interface_haptic_left->tcpWrite(buffer,size);
}

void Groundstation::sendML(HapticCommands msg)
{
  int size = msg.ByteSize();
  char buffer[size];
  msg.SerializeToArray(buffer,size);
  net_interface_ml->tcpWrite(buffer,size);
}

void Groundstation::kill()
{
   net_interface->kill();
   net_interface_camera->kill();
   net_interface_haptic_left->kill();
   net_interface_haptic_right->kill();
   
   delete net_interface;
   delete net_interface_camera;
   delete net_interface_haptic_right;
   delete net_interface_haptic_left;

   this->thread_main.interrupt();
   this->thread_main.~thread();
}


}
