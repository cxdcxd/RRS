#ifndef ROBOLAND_NETWORK_HH_
#define ROBOLAND_NETWORK_HH_

//STD
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <functional>
#include <ctime>

//Roboland
#include <Main.pb.h>
#include "zmq.hpp"

//BOOST
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

namespace roboland
{

/**
 * @brief The Network class
 * This class is responsible for create a safe and reusable functionality to interface with other applications
 * such as Unity3d engine or Xamarin Mono via TCP/IP protocol over ZMQ library.
 *
 * It served the minimum functions to read and write data from ZMQ sockets.
 */
class Network
{

public:
  Network(int port, int subsribe_port, int responser_port, std::string net_name, std::string subscribe_ip, bool is_req, int net_timeout);
  ~Network();

  /**
   * @brief Network::wait Call boost thread wait
   * @param ms The time to wait (ms)
   */
  void wait(int ms);

  /**
   * @brief Network::tcpWrite The main network write to first connected socket
   * @param buffer The pointer of write buffer
   * @param size The size of data to send in bytes
   */
  void tcpWrite(char * buffer,int size);

  /**
   * @brief Network::kill Destroy all obkects related to this class
   */
  void kill();
  void killConnection();
  void killSetup();

  /**
   * @brief Main network callback for receive envets
   * @param data Data bytes
   * @param size Data bytes size
   */
  void receiveCallback(char * data,int size,int index);

  bool app_exit_sub;
  bool app_exit_req;
  bool app_exit;
  uint64_t last_read_time;

  int tcp_port;
  int index;
  std::string name;
  std::string subscribe_tcp_ip;
  int responser_tcp_port;
  int subscribe_tcp_port;

  boost::thread *thread_Req;
  boost::thread *thread_ZMQ;
  boost::thread thread_main;

  void thrZmqSubscriber();
  void thrZmqReq();
  void thrMain();
  void start();
  void connectionWrite(RobolandConnection msg);

  std::string connectionp;
  std::string connections;

  zmq::socket_t *publisher;
  zmq::socket_t *subscriber;
  zmq::socket_t *responser;
  zmq::socket_t *request;
  zmq::context_t *zmqcontext;

  bool is_audio_mode;
  bool is_req_mode;
  bool is_connected;
  bool is_inited;
  bool is_restart;
  int time_out_val;

  int time_out;

  void setup();

  std::function<void (char * data,int size,int index)> dataCallBackFunction;
  std::function<void (bool status)> statusCallBackFunction;
};

} //roboland namespace
#endif
