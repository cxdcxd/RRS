#include "AsyncRunner.h"

namespace Roboland
{
namespace Tools
{
namespace Network
{

///%//			std::shared_ptr<dynamic> AsyncRunner::objCast(std::shared_ptr<void> source, std::type_info dest)
//			{
//				return Convert::ChangeType(source, dest);
///%//			}

///%//			std::vector<std::shared_ptr<dynamic>> AsyncRunner::getDynamics(std::vector<std::shared_ptr<void>> &args, std::vector<std::type_info> &types)
//			{
////C# TO C++ CONVERTER TODO TASK: There is no C++ equivalent to the C# 'dynamic' keyword:
//				std::vector<dynamic> arguments = std::vector<std::shared_ptr<dynamic>>(args.size());
//				for (int i = 0; i < arguments.size(); i++)
//				{
//					arguments[i] = objCast(args[i], types[i]);
//				}
//				return arguments;
///%//			}

//			bool AsyncRunner::isValidArgs(std::vector<std::shared_ptr<void>> &args, std::vector<std::type_info> &types)
//			{
//				if (args.size() != types.size())
//				{
//					return false;
//				}

//				int i = 0;
//				for (auto item : args)
//				{
//					if (item->GetType() != types[i])
//					{
//						return false;
//					}
//					i++;
//				}

//				return true;
//			}

//			AsyncRunner::ObjAtr AsyncRunner::getObjs(std::vector<Object> &args)
//			{
//				ObjAtr result;
//				std::vector<std::type_info> ts(args->Length);

//				int i = 0;
//				for (auto item : args)
//				{
//					ts[i] = item->GetType();
//					i++;
//				}

//				result.objs = args;
//				result.types = ts;
//				return result;
//			}

//			AsyncRunner::AsyncRunner()
//			{

//			}

//			void AsyncRunner::processAsyncRun(asyncFunction func, ObjAtr oa, DelegateAsyncResult callback, const std::string &key, int time_out)
//			{
//				auto timeout = TimeSpan::FromSeconds(time_out);

//				auto actualTask = std::make_shared<Task>([&] ()
//				{
//					std::shared_ptr<ProcessResult> result = std::make_shared<ProcessResult>();
//					result->Success = false;

//					auto longRunningTask = std::make_shared<Task>([&] ()
//					{
//							try
//							{
//								result = func(oa.objs, oa.types);
//							}
//							catch (const std::runtime_error &ee)
//							{
//								result->Message = ee.Message;
//							}

//					}, TaskCreationOptions::LongRunning);

//					longRunningTask->Start();

//					if (!longRunningTask->Wait(timeout))
//					{
//						result->Message = "Timeout";
//					}

//					callback == nullptr ? nullptr : callback->Invoke(result, key);
//				});

//				actualTask->Start();
//			}
}
}
}
