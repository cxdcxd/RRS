#ifndef NET2_TRANSMITED_DATA_
#define NET2_TRANSMITED_DATA_

#include "Enum.h"

#include <string>
#include <unordered_map>
#include <vector>
#include <stdexcept>
#include <memory>
#include <cmath>
#include <functional>

namespace lmt
{
namespace Tools
{
namespace Network
{
class TransmitedData
{
private:
  unsigned long long bytes = 0;
  unsigned long long old_bytes = 0;

public:
  std::function<void ()> delegateTransmitedChanged;

  TransmitedData();
  TransmitedData(unsigned long long bytes);

  unsigned long long getBytes() const;
  void setBytes(unsigned long long value);

  std::string toString();
};
}
}
}

#endif /* NET2_TRANSMITED_DATA_ */
