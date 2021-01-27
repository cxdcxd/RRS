#ifndef NET2_ASYNC_RUNNER_
#define NET2_ASYNC_RUNNER_

#include <string>
#include <vector>
#include <stdexcept>
#include <typeinfo>
#include <memory>

namespace lmt
{
namespace Tools
{
namespace Network
{
class AsyncRunner : public std::enable_shared_from_this<AsyncRunner>
{
public:
  ///%static std::shared_ptr<dynamic> objCast(std::shared_ptr<void> source, std::type_info dest);

  ///%static std::vector<std::shared_ptr<dynamic>> getDynamics(std::vector<std::shared_ptr<void>> &args, std::vector<std::type_info> &types);

  ///%static bool isValidArgs(std::vector<std::shared_ptr<void>> &args, std::vector<std::type_info> &types);

  ///%static ObjAtr getObjs(std::vector<Object> &args);

public:
  class ObjAtr
  {
  public:
    std::vector<std::shared_ptr<void>> objs;
    std::vector<std::type_info> types;
  };

  ///%using DelegateAsyncResult = std::function<void (std::shared_ptr<ProcessResult> result, const std::string &key)>;
  ///%using asyncFunction = std::function<std::shared_ptr<ProcessResult> (std::vector<std::shared_ptr<void>> obj, std::vector<std::type_info> types)>;

public:
  AsyncRunner();

  ///%void processAsyncRun(asyncFunction func, ObjAtr oa, DelegateAsyncResult callback, const std::string &key, int time_out = 5);
};
}
}
}

#endif /* NET2_ASYNC_RUNNER */
