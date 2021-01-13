#include "lmt_msgs/NodeAction.h"
#include "lmt_msgs/ListAvailable.h"
#include "lmt_msgs/ListLoaded.h"
#include "lmt_msgs/NodeEvent.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <map>
#include <string>
#include <utility>
#include <fstream>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/algorithm/string.hpp>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <dirent.h>
#include <signal.h>
#include <unistd.h>
#include <regex.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <ros/package.h>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <sensor_msgs/Image.h>

#include <lmt_msgs/nodestatus.h>
#include <lmt_msgs/nodestatuslist.h>
#include <lmt_msgs/kill_marker.h>
#include <fstream>
#include "ros/ros.h"
#include <math.h>
#include <sstream>
#include <cstdio>
#include <unistd.h>
#include <cmath>
#include <tbb/atomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <boost/lexical_cast.hpp>
#include <lmt_msgs/log.h>

using namespace std;
using namespace boost;

std::string coutcolor0 = "\033[0;0m";
std::string coutcolor_red = "\033[0;31m";
std::string coutcolor_green = "\033[0;32m";
std::string coutcolor_blue = "\033[0;34m";
std::string coutcolor_magenta = "\033[0;35m";
std::string coutcolor_brown = "\033[0;33m";

const char *test_paths[] = { "%s/bin/%s" };

std::vector<std::string > models_fpath;
std::vector<std::string > models_ename;
std::vector<std::string > shell_fpath;
std::vector<std::string > shell_ename;

struct main_node
{
public :

    std::string name ;
    std::string status ;

    std::vector<std::string> logs;
};

bool App_exit = false;

std::vector<main_node> node_list;
ros::Publisher pub_status;
ros::Subscriber sub_log;

void std_out(std::string msg,std::string color)
{
	if ( color == "")
	std::cout<<msg<<std::endl;

    if ( color == "red" )
    std::cout<<coutcolor_red<<msg<<coutcolor0<<std::endl;

    if ( color == "blue" )
    std::cout<<coutcolor_blue<<msg<<coutcolor0<<std::endl;

    if ( color == "green" )
    std::cout<<coutcolor_green<<msg<<coutcolor0<<std::endl;

    if ( color == "magenta" )
    std::cout<<coutcolor_magenta<<msg<<coutcolor0<<std::endl;

    if ( color == "brown" )
    std::cout<<coutcolor_brown<<msg<<coutcolor0<<std::endl;
}

//===========================================================================

class lmt_msgsMain
{

public:
    lmt_msgsMain(ros::NodeHandle &n)
        : __n(n)
    {
        __children_wait_thread = boost::thread(boost::bind(&lmt_msgsMain::wait_thread, this));
        if (regcomp(&__re_alnum, "^[[:alnum:]]+$", REG_EXTENDED) != 0) {
            throw ros::Exception("Failed to compile regex");
        }

        __use_acceptable_modules_file = n.getParam("/lmt_msgs/acceptable_modules_file",
                                                   __acceptable_modules_file);

        if (__use_acceptable_modules_file) {
            ROS_INFO("Using acceptable modules file %s", __acceptable_modules_file.c_str());
            std::ifstream f(__acceptable_modules_file.c_str());
            while (! (f.fail() || f.eof()) ) {
                std::string mod;
                f >> mod;
              
            }
        } else {
            std::string ros_root_bin = getenv("ROS_ROOT");
            if (ros_root_bin == "") {
                throw ros::Exception("Failed to read ROS_ROOT environment variable");
            }
            ros_root_bin += "/bin";
            __search_paths.push_back(ros_root_bin);
       
        }

        ros::NodeHandle nn;
        __srv_startall = n.advertiseService("/manager/startall", &lmt_msgsMain::startall_node, this);
        __srv_stopall = n.advertiseService("/manager/stopall", &lmt_msgsMain::stopall_node, this);
        __srv_start = n.advertiseService("/manager/start", &lmt_msgsMain::start_node, this);
        __srv_stop = n.advertiseService("/manager/stop", &lmt_msgsMain::stop_node, this);
        __srv_pause = n.advertiseService("/manager/pause", &lmt_msgsMain::pause_node, this);
        __srv_continue = n.advertiseService("/manager/continue", &lmt_msgsMain::continue_node, this);
        __srv_list_loaded = n.advertiseService("/manager/list_loaded",&lmt_msgsMain::list_loaded, this);
        __srv_list_avail = n.advertiseService("/manager/list_available", &lmt_msgsMain::list_available, this);

        __pub_node_events = n.advertise<lmt_msgs::NodeEvent>("manager/node_events", 10);
        service_markerkill = nn.serviceClient<lmt_msgs::kill_marker>("pgitic_kill_marker");

    }

    ~lmt_msgsMain()
    {
        void *dont_care;
        pthread_cancel(__children_wait_thread.native_handle());
        pthread_join(__children_wait_thread.native_handle(), &dont_care);
    }

    std::string find_valid_devel(std::string &progname)
    {

        for (ChildrenMap::iterator i = __children.begin(); i != __children.end(); ++i) {
            if (i->second.first == progname) {
                throw ros::Exception("Program is already running");
            }
        }

        for (int i = 0 ; i < models_ename.size() ; i++)
        {
            if ( models_ename.at(i) == progname.c_str())
            {
                return models_fpath.at(i);
            }
        }

        throw ros::Exception("No program with the requested name found");
    }

    std::string find_valid_shell(std::string &progname)
    {
        for (ChildrenMap::iterator i = __children.begin(); i != __children.end(); ++i) {
            if (i->second.first == progname) {
                throw ros::Exception("Program is already running");
            }
        }

        for (int i = 0 ; i < shell_ename.size() ; i++)
        {
            if ( shell_ename.at(i) == progname.c_str())
            {
                return shell_fpath.at(i);
            }
        }

        throw ros::Exception("No program with the requested name found");
    }

    void update_status(std::string name,std::string status)
    {
    	std::cout<<"updated : "<<name<<" "<<status<<std::endl;
        for ( int i = 0 ; i < node_list.size() ; i++ )
        {
            if ( node_list.at(i).name == name )
            {
                node_list.at(i).status = status; break;
            }
        }
    }

    
    bool fork_and_exec(std::string progname)
    {
        std_out("GET START :" + progname,"magenta");

        std::string p;
        bool isshell = false;
        isshell = boost::algorithm::contains(progname, ".sh");

        if (  isshell )
            p = find_valid_shell(progname);
        else
            p = find_valid_devel(progname);

        pid_t pid = fork();
        if (pid == -1) {
            return false;
        } else if (pid == 0) {
            // child
            setsid();
            signal(SIGINT, SIG_IGN);
            ROS_INFO("Running %s from path %s", progname.c_str(), p.c_str());
            fclose(stdout);
            fclose(stdin);
            fclose(stderr);
            execl(p.c_str(), p.c_str(), NULL);
        } else {
            ROS_DEBUG("Child PID %i", pid);
            boost::mutex::scoped_lock lock(__children_mutex);
            __children[pid] = make_pair(progname, p);
            if (__children.size() == 1) {
                __children_cond.notify_all();
            }
            lmt_msgs::NodeEvent msg;
            msg.header.stamp.setNow(ros::Time::now());
            msg.event_type = lmt_msgs::NodeEvent::NODE_STARTED;
            msg.node_name = progname;
            __pub_node_events.publish(msg);

            update_status(progname,"start");
        }

        return true;
    }

    void wait_thread()
    {
        while (ros::ok()) {
            boost::unique_lock<boost::mutex> lock(__children_mutex);
            while (__children.empty()) {
                __children_cond.wait(lock);
            }

            int status = 0;
            lock.unlock();
            pid_t pid = waitpid(-1, &status, WUNTRACED | WCONTINUED);
            if (pid == -1)  continue;
            lock.lock();

            lmt_msgs::NodeEvent msg;
            msg.event_type = lmt_msgs::NodeEvent::NODE_DIED;
            msg.node_name = __children[pid].first;
            update_status(msg.node_name,"stop");

            // Debug output
            if (WIFEXITED(status)) {
                ROS_INFO("%i/%s exited, status=%d", pid,
                         __children[pid].first.c_str(), WEXITSTATUS(status));
                char *tmp;
                if (asprintf(&tmp, "%s (PID %i) exited, status=%d",
                             __children[pid].first.c_str(), pid, WEXITSTATUS(status)) != -1) {
                    msg.message = tmp;
                    free(tmp);
                }
            } else if (WIFSIGNALED(status)) {
                ROS_INFO("%i/%s killed by signal %d", pid,
                         __children[pid].first.c_str(), WTERMSIG(status));
                char *tmp;
                if (asprintf(&tmp, "%s (PID %i) killed by signal %d",
                             __children[pid].first.c_str(), pid, WTERMSIG(status)) != -1) {
                    msg.message = tmp;
                    free(tmp);
                }
            } else if (WIFSTOPPED(status)) {
                ROS_INFO("%i/%s stopped by signal %d", pid,
                         __children[pid].first.c_str(), WSTOPSIG(status));
                char *tmp;
                msg.event_type = lmt_msgs::NodeEvent::NODE_PAUSED;
                update_status(msg.node_name,"pause");
                if (asprintf(&tmp, "%s (PID %i) stopped by signal %d",
                             __children[pid].first.c_str(), pid, WSTOPSIG(status)) != -1) {
                    msg.message = tmp;
                    free(tmp);
                }
            } else if (WIFCONTINUED(status)) {
                ROS_INFO("%i/%s continued", pid, __children[pid].first.c_str());
                char *tmp;
                msg.event_type = lmt_msgs::NodeEvent::NODE_CONTINUED;
                update_status(msg.node_name,"resume");
                if (asprintf(&tmp, "%s (PID %i) continued",
                             __children[pid].first.c_str(), pid) != -1) {
                    msg.message = tmp;
                    free(tmp);
                }
            }

            if (WIFEXITED(status) || WIFSIGNALED(status)) {
                if (WIFSIGNALED(status)) {
                    int sig = WTERMSIG(status);
                    if (sig == SIGSEGV) {
                        // inform about faulty program
                        ROS_WARN("Program %s (%s) died with segfault", __children[pid].first.c_str(),
                                 __children[pid].second.c_str());

                        char *tmp;
                        msg.event_type |= lmt_msgs::NodeEvent::NODE_SEGFAULT;
                        update_status(msg.node_name,"segfault");
                        if (asprintf(&tmp, "%s (PID %i) died with segfault",
                                     __children[pid].first.c_str(), pid) != -1) {
                            msg.message = tmp;
                            free(tmp);
                        }
                    }
                }
                __children.erase(pid);
            }

            __pub_node_events.publish(msg);


        }
    }

    std::string get_process_state(pid_t pid)
    {
        char *procpath;
        if (asprintf(&procpath, "/proc/%i/stat", pid) != -1) {
            FILE *f = fopen(procpath, "r");
            if (f) {
                int pid;
                char *program;
                char state[2]; state[1] = 0;
                if (fscanf(f, "%d %as %c", &pid, &program, state) == 3) {
                    free(program);
                    return state;
                }
                fclose(f);
            }
            free(procpath);
        }

        return "?";
    }

    pid_t get_pid(std::string &node_file_name)
    {
        for (ChildrenMap::iterator i = __children.begin(); i != __children.end(); ++i) {
            if (i->second.first == node_file_name) {
                return i->first;
            }
        }
        return 0;
    }

    bool start_node(lmt_msgs::NodeAction::Request &req,
                    lmt_msgs::NodeAction::Response &resp)
    {

        return fork_and_exec(get_name(req.node_file_name));
    }

    bool send_signal(std::string &node_file_name, int signum)
    {
        pid_t pid = get_pid(node_file_name);
        if (pid != 0) {
            ROS_INFO("Sending signal %s (%i) to %s (PID %i)", strsignal(signum), signum,
                     __children[pid].first.c_str(), pid);
            ::kill(pid, signum);
            return true;
        } else {
            return false;
        }
    }

    bool pause_node(lmt_msgs::NodeAction::Request &req,
                    lmt_msgs::NodeAction::Response &resp)
    {
        return send_signal(req.node_file_name, SIGSTOP);
    }

    bool continue_node(lmt_msgs::NodeAction::Request &req,
                       lmt_msgs::NodeAction::Response &resp)
    {
        return send_signal(req.node_file_name, SIGCONT);
    }

    bool startall_node(lmt_msgs::NodeAction::Request &req,lmt_msgs::NodeAction::Response &resp)
    {
        std::cout<<"startall"<<std::endl;
        std::string line = req.node_file_name;

        std::vector<std::string> strs;
        boost::algorithm::split(strs,line,boost::algorithm::is_any_of(","));
        for ( int i = 0 ; i < strs.size() ; i++ )
        {
            fork_and_exec(get_name(strs.at(i)));
        }
    }


    string get_name(string sindex)
    {
        int index = boost::lexical_cast<int>(sindex);
        index--;
        return node_list.at(index).name;
    } 

    bool stopnode(std::string name)
    {
        string nodeName = get_name(name);

        std_out("GET STOP :" + nodeName,"magenta");

        pid_t pid = get_pid(nodeName);
        if (pid != 0) {
            std::string state = get_process_state(pid);
            ROS_INFO("Sending signal %s (%i) to %s (PID %i)", strsignal(SIGINT), SIGINT,
                     __children[pid].first.c_str(), pid);
            ::kill(pid, SIGINT);

            //additional kill
            bool isshell = boost::algorithm::contains(nodeName, ".sh");
            
            if ( isshell )
            {
                boost::erase_all(nodeName, ".sh");
                nodeName += "kill.sh";

                std_out("KILL : " + nodeName,"red");
                fork_and_exec(nodeName);
            }

            return true;
        } else {
            return false;
        }
    }

    bool stopall_node(lmt_msgs::NodeAction::Request &req, lmt_msgs::NodeAction::Response &resp)
    {
        std::cout<<"stopall"<<std::endl;
        std::string line = req.node_file_name;

        std::vector<std::string> strs;
        boost::algorithm::split(strs,line,boost::algorithm::is_any_of(","));

        for ( int i = 0 ; i < strs.size() ; i++ )
        {
            stopnode(strs.at(i));
        }
    }

    bool stop_node(lmt_msgs::NodeAction::Request &req,lmt_msgs::NodeAction::Response &resp)
    {
        return stopnode(req.node_file_name);
    }

    bool list_loaded(lmt_msgs::ListLoaded::Request &req,
                     lmt_msgs::ListLoaded::Response &resp)
    {
        for (ChildrenMap::iterator i = __children.begin(); i != __children.end(); ++i) {
            resp.nodes.clear();
            resp.nodes.push_back(i->second.first);
        }
        return true;
    }

    bool list_available(lmt_msgs::ListAvailable::Request &req,
                        lmt_msgs::ListAvailable::Response &resp)
    {
        resp.bin_files.clear();
        for (std::list<std::string>::iterator i = __search_paths.begin();
             i != __search_paths.end(); ++i) {
            for (unsigned int j = 0; (j < sizeof(test_paths) / sizeof(const char *)); ++j) {
                char *tmp;
                if (asprintf(&tmp, test_paths[j], i->c_str(), "") != -1) {
                    struct stat s;
                    if (stat(tmp, &s) == 0) {
                        if (S_ISDIR(s.st_mode) && (access(tmp, X_OK) == 0)) {
                            // check for files
                            DIR *d = opendir(tmp);
                            if (d != NULL) {
                                struct dirent de, *deres;
                                if ((readdir_r(d, &de, &deres) == 0) && (deres != NULL)) {
                                    do {
                                        char *tmp2;
                                        if (asprintf(&tmp2, test_paths[j], i->c_str(), de.d_name) != -1) {
                                            struct stat filestat;
                                            if (stat(tmp2, &filestat) == 0) {
                                                if (S_ISREG(filestat.st_mode) && (access(tmp2, X_OK) == 0)) {
                                                    resp.bin_files.push_back(de.d_name);
                                                }
                                            }
                                            free(tmp2);
                                        }
                                    } while ((readdir_r(d, &de, &deres) == 0) && (deres != NULL));
                                }
                                closedir(d);
                            }
                        }
                    }
                    free(tmp);
                }
            }
        }
        return true;
    }

private:
    ros::NodeHandle &__n;
    ros::Publisher     __pub_node_events;
    ros::ServiceServer __srv_startall;
    ros::ServiceServer __srv_stopall;
    ros::ServiceServer __srv_start;
    ros::ServiceServer __srv_stop;
    ros::ServiceServer __srv_pause;
    ros::ServiceServer __srv_continue;
    ros::ServiceServer __srv_list_loaded;
    ros::ServiceServer __srv_list_avail;
    ros::ServiceClient service_markerkill;
    std::list<std::string> __search_paths;
    typedef std::map<int, std::pair<std::string, std::string> > ChildrenMap;
    ChildrenMap                __children;
    boost::mutex               __children_mutex;
    boost::condition_variable  __children_cond;
    boost::thread              __children_wait_thread;
    bool                   __use_acceptable_modules_file;
    std::string            __acceptable_modules_file;
    std::list<std::string> __acceptable_modules;
    regex_t __re_alnum;
};



void loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension, std::vector<std::string> &models)
{
    if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    {
        ROS_INFO("Not found");
        return;
    }

    for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
    {
        if (boost::filesystem::is_directory (it->status ()))
        {
            //ROS_INFO("in folder");
            loadFeatureModels (it->path (), extension, models);
        }
        if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
        {
            //ROS_INFO("pushed");
            std::string m;
            boost::filesystem::path path;
            path=base_dir / it->path ().filename ();
            m=path.string();
            models.push_back (m);
        }
    }
}

std::vector<std::string> names;

int init_nodes()
{
	//DCM [9]
    names.push_back("kinect2.sh");         //1
    names.push_back("kinect1.sh");         //2
    names.push_back("motor.sh");           //3
    names.push_back("lowerbodycore");      //4
	names.push_back("upperbodycore");      //5
	names.push_back("laser.sh");           //6
	names.push_back("tcp_core");           //7
    names.push_back("odometry");           //8
    names.push_back("cmd_watchdog");       //9
    names.push_back("texttospeech");        //10
    names.push_back("websocket.sh");        //11
    names.push_back("hector_calib.sh");     //12
    names.push_back("hector_rec.sh");       //13
    names.push_back("hector_main.sh");      //14
    names.push_back("lmtmapengine");    //15
    names.push_back("move.sh");             //16
    names.push_back("lmt_movebase");     //17
    names.push_back("object_recognition_action_server");  //18
    names.push_back("human.sh");            //19
	names.push_back("scenario1");           //20

	for ( int i = 0 ; i < names.size() ; i++)
    {
	     main_node _main_node;
	    _main_node.name = names.at(i);
	  
	    _main_node.status = "stop";
	    node_list.push_back(_main_node);
    }
}

void publisher()
{
    while ( App_exit == false )
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        
        lmt_msgs::nodestatuslist list;

        for ( int i = 0 ; i < node_list.size() ; i++ )
        {
            lmt_msgs::nodestatus ns;
            ns.name = node_list.at(i).name;
            ns.status = node_list.at(i).status;
            //ns.ack = node_list.at(i).wd->ack;
            list.statuslist.push_back(ns);
        }

        pub_status.publish(list);
    }

    //kill all wds
    for ( int i = 0 ; i < node_list.size() ; i++)
    {
        //node_list.at(i).wd->kill();
    }

}

string get_time()
{
    time_t now = time(0);
    tm* localtm = localtime(&now);

    int day = localtm->tm_mday;
    int month = localtm->tm_mon;
    int year = localtm->tm_year - 100 + 2000;

    int hour = localtm->tm_hour;
    int min = localtm->tm_min;
    int sec = localtm->tm_sec;

    std::string sday = boost::lexical_cast<string>(day);
    std::string smonth = boost::lexical_cast<string>(month);
    std::string syear = boost::lexical_cast<string>(year);
    std::string shour = boost::lexical_cast<string>(hour);
    std::string smin = boost::lexical_cast<string>(min);
    std::string ssec = boost::lexical_cast<string>(sec);

    if ( day < 10 ) sday = "0" + sday;
    if ( month < 10 ) smonth = "0" + smonth;
    if ( year < 10 ) syear = "0" + syear;

    if ( hour < 10 ) shour = "0" + shour;
    if ( min < 10 ) smin = "0" + smin;
    if ( sec < 10 ) ssec = "0" + ssec;

    std::string time_stamp =  syear  + "/" + smonth + "/" + sday + " ";
    time_stamp += shour  + ":" + smin + ":" + ssec + " => ";

    return time_stamp;
}

void save_log(std::string id,string message)
{
    return;
    std::string path_points =  ros::package::getPath("managment") + "/logs/" + id + ".txt";
    std::string line;
    std::ofstream text;

    string time_stamp =  get_time();

    text.open(path_points.c_str(), std::fstream::out | std::fstream::app); //second time append the current log txt

    if (text.is_open())
    {
        text<<time_stamp<<message<<endl;
        text.flush();
        text.close();
    }
    else
    {
        std::cout<<coutcolor_red << "Unable to open file" << coutcolor0 << std::endl;
    }

}

void main_log_callback(const lmt_msgs::log::ConstPtr &msg)
{
   save_log(msg->id,msg->message);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manager");
    ros::Time::init();

    init_nodes();

    ros::NodeHandle n;
    //=========================================================================
    std::string homedir = getenv("HOME");
    std::string root_path = homedir + "/catkin_ws/devel/lib/";
    std::string extension = "";
    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
    loadFeatureModels (root_path, extension, models_fpath);

    for (int i = 0 ; i < models_fpath.size() ; i++ )
    {
        std::vector<std::string> strs;
        std::string path = models_fpath[i];
        boost::algorithm::split(strs,path,boost::algorithm::is_any_of("/"));

        std::string name = strs.at(strs.size()-1);
        models_ename.push_back(name);
        //std_out(name,"brown");
    }

    std::cout<< models_ename.size() << std::endl;

    root_path =  ros::package::getPath("managment") + "/shell/";
    extension = ".sh";

    transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
    loadFeatureModels (root_path, extension, shell_fpath);

    for (int i = 0 ; i < shell_fpath.size() ; i++ )
    {
        std::vector<std::string> strs;
        std::string path = shell_fpath[i];
        boost::algorithm::split(strs,path,boost::algorithm::is_any_of("/"));

        std::string name = strs.at(strs.size()-1);
        shell_ename.push_back(name);
        //std_out(name,"brown");
    }

    std::cout<< shell_ename.size() << std::endl;

    std_out("======================================== Ready !","green");

    pub_status = n.advertise<lmt_msgs::nodestatuslist>("manager/node_status", 10);
    sub_log = n.subscribe("/manager/log", 1, main_log_callback);

    lmt_msgsMain lmt_msgs(n);

    boost::thread _thread_wd(&publisher);
  
    ros::spin();

    _thread_wd.interrupt();
    _thread_wd.join();

    App_exit = true;
    return 0;

}
