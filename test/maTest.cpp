#include <cstdlib>
#include <csignal>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "ros/callback_queue.h"
#include <ros/package.h>
#include "visual_servo/manipulate.h"
#include "VisualServoMetaType.h"

#include "JoyTeleop.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "tr1/memory"
using std::tr1::shared_ptr;
using namespace JOYTELEOP;
class Listener
{
    typedef void(*sighandler_t)(int);
private:
    ros::NodeHandle nh;
    ros::ServiceClient client;
    visual_servo::manipulate srv;
    shared_ptr<boost::thread> thread_ptr_;
    bool isCalling_{};
public:
    bool isCalling() const{return isCalling_;};
    Listener();
    virtual ~Listener();
    void worker(int SrvRequestType);
    bool callSrv(int SrvRequestType);
    int pox_system(const char* commandsz);
};
Listener::Listener()
{
    client =nh.serviceClient<visual_servo::manipulate>("manipulate");
}
Listener::~Listener()
{
}

bool Listener::callSrv(int SrvRequestType)
{
    thread_ptr_.reset(new boost::thread(boost::bind(&Listener::worker,this,SrvRequestType)));
}
void Listener::worker(int SrvRequestType)
{
    isCalling_=true;
    srv.request.type=SrvRequestType;
    if (client.call(srv))
    {
        visual_servo_namespace::printServiceStatus(srv.response.status);
        switch (srv.response.status)
        {
            case visual_servo_namespace::SERVICE_STATUS_SUCCEED:
                ROS_INFO("SUCCESS");
                /*Do something when success*/
                break;
            case visual_servo_namespace::SERVICE_STATUS_EMPTY:
                ROS_INFO("EMPTY");
                break;
            default:
                ROS_ERROR("Failed");
                break;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service detect_once");
    }
    isCalling_=false;
}
int Listener::pox_system(const char *commands)
{
    int ret=0;
    sighandler_t old_handler;
    old_handler = signal(SIGCHLD,SIG_DFL);
    ret = system(commands);
    signal(SIGCHLD,old_handler);
    return ret;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "maTest");
    ros::NodeHandle nh_("~");
    Listener listener;
    double max_linear_velocity,max_angular_velocity;
    nh_.param("max_linear_velocity",max_linear_velocity,(double)0.8);
    nh_.param("max_angular_velocity",max_angular_velocity,(double)0.5);
    JOYTELEOP::JoyTeleop joyTeleop("joy",true,max_linear_velocity,max_angular_velocity);
    
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        switch(joyTeleop.getControlTrigger())
        {
            case ManipulateOn:
                if(!listener.isCalling())
                    listener.callSrv(visual_servo::manipulate::Request::CUT);
                break;
            case ChargeOn:
                if(!listener.isCalling())
                    listener.callSrv(visual_servo::manipulate::Request::CHARGE);
                break;
            case UpOn:
                if(!listener.isCalling())
                    listener.callSrv(visual_servo::manipulate::Request::UP);
                break;
            case HomeOn:
                if(!listener.isCalling())
                    listener.callSrv(visual_servo::manipulate::Request::HOME);
                break;
            case ClockGo:
                ros::param::set("/visual_servo/clockGo",1.0);
                break;
            case AntiClockGo:
                ros::param::set("/visual_servo/antiClockGo",1.0);
                break;
            case SingleAntiClockGo:
                ros::param::set("/visual_servo/singleAntiClockGo",1.0);
                break;
            case SingleClockGo:
                ros::param::set("/visual_servo/singleClockGo",1.0);
                break;
            case KnifeOn:
                ros::param::set("/visual_servo/knifeOn",1.0);
                break;
            case KnifeOff:
                ros::param::set("/visual_servo/knifeOff",1.0);
                break;
            case KnifeUnplug:
                ros::param::set("/visual_servo/knifeUnplug",1.0);
                break;
            case LightOn:
                ros::param::set("/visual_servo/lightOn",1.0);
                break;
            case LightOff:
                ros::param::set("/visual_servo/lightOff",1.0);
                break;
            case RobotArmOn:
            {
                ros::V_string AllNodes;
                ros::master::getNodes(AllNodes);
                for(auto & node_name:AllNodes)
                {
                    if(node_name=="/aubo_gazebo_driver"||node_name=="/rviz")
                    {
                        listener.pox_system(("rosnode kill " + node_name).c_str());
                        sleep(20);
                    }
                }
                listener.pox_system("gnome-terminal -x bash -c \"roslaunch visual_servo visual_servo_sim.launch\";exit;exec bash;");
                break;
            }
            case NavigationOn:
            {
                ros::V_string AllNodes;
                ros::master::getNodes(AllNodes);
                for(auto & node_name:AllNodes)
                {
                    if(node_name=="/goalSaver"||node_name=="/odomFilter"||node_name=="/auboRobot")
                    {
                        listener.pox_system(("rosnode kill " + node_name).c_str());
                    }
                }
                sleep(5);
                listener.pox_system("gnome-terminal -x bash -c \"roslaunch cartographer_ros demo_velodyne2d_localization.launch load_state_filename:=/home/xcy/subbag2.bag.pbstream\";exit;exec bash;");
                sleep(2);
                listener.pox_system("gnome-terminal -x bash -c \"roslaunch rubber_navigation rubber_navigation.launch\";exit;exec bash");
                break;
            }
            case ShutDown:
            {
                ros::V_string AllNodes;
                ros::master::getNodes(AllNodes);
                for(auto & node_name:AllNodes)
                {
                    if(node_name!="/joystick"&&node_name!="/maTest"&&node_name!="/rosout")
                    {
                        listener.pox_system(("rosnode kill " + node_name).c_str());
                    }
                }
                sleep(1);
                listener.pox_system("gnome-terminal -x bash -c \"shutdown\"; exec bash;");
            }
            case MappingOn:
            {
                ros::V_string AllNodes;
                ros::master::getNodes(AllNodes);
                for(auto & node_name:AllNodes)
                {
                    if(node_name=="/rviz")
                    {
                        listener.pox_system(("rosnode kill " + node_name).c_str());
                        sleep(5);
                        break;
                    }
                }
                listener.pox_system("gnome-terminal -x bash -c \"roslaunch cartographer_ros demo_velodyne2d.launch\";exit;exec bash;");
                break;
            }
            case MappingOff:
            {
                ros::V_string AllNodes;
                ros::master::getNodes(AllNodes);
                for(auto & node_name:AllNodes)
                {
                    if(node_name=="/rviz")
                    {
                        listener.pox_system("rosservice call /finish_trajectory 0");
                        sleep(1);
                        listener.pox_system("gnome-terminal -x bash -c \"rosservice call /write_state \"filename: \'/home/xcy/subbag2.bag.pbstream\' include_unfinished_submaps: \'true\'\"\";exit;exec bash;");
                        sleep(2);
                        break;
                    }
                }
                break;
            }
            default:
                break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}