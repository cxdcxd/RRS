#ifndef NET2_NTP_CLIENT_HH_
#define NET2_NTP_CLIENT_HH_

#include <string>
#include <vector>
#include <iostream>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include "ProcessResult.h"

#include <numeric>
#include <chrono>

#include "ros/ros.h"

#include <chrono>
#include <sys/time.h>
#include <ratio>

using namespace std::chrono;

#define NTP_TIMESTAMP_DELTA 2208988800ull

#define LI(packet)   (uint8_t) ((packet.li_vn_mode & 0xC0) >> 6) // (li   & 11 000 000) >> 6
#define VN(packet)   (uint8_t) ((packet.li_vn_mode & 0x38) >> 3) // (vn   & 00 111 000) >> 3
#define MODE(packet) (uint8_t) ((packet.li_vn_mode & 0x07) >> 0) // (mode & 00 000 111) >> 0

namespace Roboland
{
namespace Tools
{
namespace Network
{
typedef struct
{
  uint8_t li_vn_mode;      // Eight bits. li, vn, and mode.
  // li.   Two bits.   Leap indicator.
  // vn.   Three bits. Version number of the protocol.
  // mode. Three bits. Client will pick mode 3 for client.

  uint8_t stratum;         // Eight bits. Stratum level of the local clock.
  uint8_t poll;            // Eight bits. Maximum interval between successive messages.
  uint8_t precision;       // Eight bits. Precision of the local clock.

  uint32_t rootDelay;      // 32 bits. Total round trip delay time.
  uint32_t rootDispersion; // 32 bits. Max error aloud from primary clock source.
  uint32_t refId;          // 32 bits. Reference clock identifier.

  uint32_t refTm_s;        // 32 bits. Reference time-stamp seconds.
  uint32_t refTm_f;        // 32 bits. Reference time-stamp fraction of a second.

  uint32_t origTm_s;       // 32 bits. Originate time-stamp seconds.
  uint32_t origTm_f;       // 32 bits. Originate time-stamp fraction of a second.

  uint32_t rxTm_s;         // 32 bits. Received time-stamp seconds.
  uint32_t rxTm_f;         // 32 bits. Received time-stamp fraction of a second.

  uint32_t txTm_s;         // 32 bits and the most important field the client cares about. Transmit time-stamp seconds.
  uint32_t txTm_f;         // 32 bits. Transmit time-stamp fraction of a second.

} ntp_packet; // Total: 384 bits or 48 bytes.

class NTPClient
{
public:
  NTPClient();
  time_t time_offset;
  std::chrono::time_point<std::chrono::steady_clock> time_start;
  std::chrono::time_point<std::chrono::steady_clock> time_end;
  void error( char* msg );
  long sync(std::string ntp_host_name, int port);
  long get();
};
}
}
}

#endif /* NET2_NTP_CLIENT_HH_ */
