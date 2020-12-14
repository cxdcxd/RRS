#ifndef NET2_PROCESS_RESULT_
#define NET2_PROCESS_RESULT_

#include <string>
#include <memory>

template <class T>
class ProcessResult
{
public:
  bool success = false;
  T result;
  std::string message;
};

#endif /* NET2_PROCESS_RESULT */
