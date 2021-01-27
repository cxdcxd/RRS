#include "Net2/TransmitedData.h"


namespace lmt
{
namespace Tools
{
namespace Network
{

TransmitedData::TransmitedData()
{
  bytes = 0;
  old_bytes = 0;
}

TransmitedData::TransmitedData(unsigned long long bytes)
{
  this->bytes = bytes;
}

unsigned long long TransmitedData::getBytes() const
{
  return bytes;
}

void TransmitedData::setBytes(unsigned long long value)
{
  bytes = (value > 0) ? value : 0;

  if (old_bytes != bytes)
  {
    old_bytes = bytes;

    if (delegateTransmitedChanged )
      delegateTransmitedChanged();
  }
}

std::string TransmitedData::toString()
{
  std::string final_name = "";

  Net2DataRateUnit unit = Net2DataRateUnit::B;
  unsigned long long amount = bytes;
  unsigned long long residual = 0;

  int uintt = (int)unit;

  while (amount >= 1024 && uintt != 6)
  {
    uintt++;
    residual = amount % 1024;
    amount /= 1024;
  }

  int fraction = static_cast<int>(std::round(((residual / 1024.0) * 100)));

  final_name = std::to_string(amount);

  if ( fraction > 0 )
  {
    final_name += ".";

    std::string temp = std::to_string(fraction);

    if ( temp.size() == 1)
    {
      temp = "0" + temp;
    }

    final_name += temp;

  }

  if ( uintt == 1)
    final_name += "B";
  if ( uintt == 2)
    final_name += "KB";
  if ( uintt == 3)
    final_name += "MB";
  if ( uintt == 4)
    final_name += "GB";
  if ( uintt == 5)
    final_name += "TB";
  if ( uintt == 6)
    final_name += "PB";

  return final_name;
}
}
}
}
