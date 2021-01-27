#include "Net2/NTPClient.hh"

namespace lmt
{
namespace Tools
{
namespace Network
{

NTPClient::NTPClient()
{
   this->time_start = std::chrono::steady_clock::now();
}

void NTPClient::error( char* msg )
{
  perror( msg ); // Print the error message to stderr.
  exit( 0 ); // Quit the process.
}

long NTPClient::get()
{
  long result = 0;

  this->time_end = std::chrono::steady_clock::now();

  std::chrono::duration<double> diff = this->time_end-this->time_start;

  int diff2 = diff.count() * 1000;

  result = (long)(diff2) + (long)time_offset;

  return result;
}

long NTPClient::sync(std::string host_name, int port)
{
  ROS_INFO_STREAM("NTP Client calling " << host_name.c_str());

  int sockfd, n;  // Socket file descriptor and the n return result from writing/reading from the socket.

  ntp_packet packet = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  memset( &packet, 0, sizeof( ntp_packet ) );

  // Set the first byte's bits to 00,011,011 for li = 0, vn = 3, and mode = 3. The rest will be left set to zero.

  *( ( char * ) &packet + 0 ) = 0x1b; // Represents 27 in base 10 or 00011011 in base 2.

  // Create a UDP socket, convert the host-name to an IP address, set the port number,
  // connect to the server, send the packet, and then read in the return packet.

  struct sockaddr_in serv_addr; // Server address data structure.
  struct hostent *server;      // Server data structure.

  sockfd = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP ); // Create a UDP socket.

  if ( sockfd < 0 )
    error( "ERROR opening socket" );

  server = gethostbyname( host_name.c_str() ); // Convert URL to IP.

  std::cout << server << std::endl;

  if ( server == NULL )
    error( "ERROR, no such host" );

  // Zero out the server address structure.

  bzero( ( char* ) &serv_addr, sizeof( serv_addr ) );

  serv_addr.sin_family = AF_INET;

  // Copy the server's IP address to the server address structure.

  bcopy( ( char* )server->h_addr, ( char* ) &serv_addr.sin_addr.s_addr, server->h_length );

  // Convert the port number integer to network big-endian style and save it to the server address structure.

  serv_addr.sin_port = htons( port );

  // Call up the server using its IP address and port number.

  if ( connect( sockfd, ( struct sockaddr * ) &serv_addr, sizeof( serv_addr) ) < 0 )
    error( "ERROR connecting" );

  // Send it the NTP packet it wants. If n == -1, it failed.

  n = write( sockfd, ( char* ) &packet, sizeof( ntp_packet ) );

  if ( n < 0 )
    error( "ERROR writing to socket" );

  // Wait and receive the packet back from the server. If n == -1, it failed.

  n = read( sockfd, ( char* ) &packet, sizeof( ntp_packet ) );

  if ( n < 0 )
    error( "ERROR reading from socket" );

  // These two fields contain the time-stamp seconds as the packet left the NTP server.
  // The number of seconds correspond to the seconds passed since 1900.
  // ntohl() converts the bit/byte order from the network's to host's "endianness".

  packet.txTm_s = ntohl( packet.txTm_s ); // Time-stamp seconds.
  packet.txTm_f = ntohl( packet.txTm_f ); // Time-stamp fraction of a second.

  // Extract the 32 bits that represent the time-stamp seconds (since NTP epoch) from when the packet left the server.
  // Subtract 70 years worth of seconds from the seconds since 1900.
  // This leaves the seconds since the UNIX epoch of 1970.
  // (1900)------------------(1970)**************************************(Time Packet Left the Server)

  //TODO We have 200 ms difference in our time and sync time
  time_t txTm = ( time_t ) ( packet.txTm_s - 2208988800 );

  int f = packet.txTm_f / 10000000 ;
  long final = ((long)txTm) * 1000 + f;

  this->time_end = std::chrono::steady_clock::now();

  std::chrono::duration<double> diff = this->time_end-this->time_start;

  double diff2 = diff.count() * 1000;

  long result = (long)(diff2);

  this->time_offset = final - result;

  //std::cout << final << std::endl;

  // Print the time we got from the server, accounting for local timezone and conversion from UTC time.
  printf( "Time: %s", ctime( ( const time_t* ) &txTm ) );

  return final;
}
}
}
}
