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

#include <yaml-cpp/yaml.h>

#include "JoyTeleop.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "tr1/memory"

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> ( const YAML::Node& node, T& i )
{
    i = node.as<T>();
}
#endif

using std::tr1::shared_ptr;
using namespace JOYTELEOP;
class Listener
{
    typedef void(*sighandler_t)(int);
private:
    ros::NodeHandle nh;
	ros::Subscriber tag_info_sub;
	int tagID_;
	void tagInfoCallback(const visual_servo::TagsDetection_msg &msg);
public:
    Listener(int tag_id);
    virtual ~Listener();
    visual_servo_namespace::ServiceCaller* serviceCaller;
    int pox_system(const char* commands);
	double getParameter();
};
Listener::Listener(int tag_id):tagID_(tag_id)
{
    serviceCaller = new visual_servo_namespace::ServiceCaller();
	tag_info_sub=nh.subscribe("TagsDetected",10, &Listener::tagInfoCallback,this);
}
Listener::~Listener()
{
    delete serviceCaller;
}
void Listener::tagInfoCallback(const visual_servo::TagsDetection_msg &msg)
{
	if(!msg.tags_information.empty())
	{
	 	for(auto & tag_information : msg.tags_information)
    	{
        	tagID_=tag_information.id;
    	}
	}

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
double Listener::getParameter()
{
	double result=190.0;
	YAML::Node doc = YAML::LoadFile(ros::package::getPath("visual_servo")+"/config/tagParam.yaml");
    try
    {
        result = doc["ID"+std::to_string(tagID_)]["steeringDistance"].as<double>(); 
   }
    catch (YAML::InvalidScalar)
    {
        ROS_ERROR("tagParam.yaml is invalid.");
    }
	ROS_INFO_STREAM("the tag id is "<<tagID_<<" the steeringDistance is "<<result);
	return result;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "maTest");
    ros::NodeHandle nh_("~");
    Listener listener(400);
    double max_linear_velocity,max_angular_velocity;
    nh_.param("max_linear_velocity",max_linear_velocity,(double)0.8);
    nh_.param("max_angular_velocity",max_angular_velocity,(double)0.5);
    JOYTELEOP::JoyTeleop joyTeleop("joy",true,max_linear_velocity,max_angular_velocity);

    
	ros::V_string AllNodes;
   	ros::master::getNodes(AllNodes);
    for(auto & node_name:AllNodes)
    {
    	if(node_name=="/base_only")
        {
         	listener.pox_system(("rosnode kill " + node_name).c_str());
			sleep(10);     
	   }
	}
    listener.pox_system("gnome-terminal -x bash -c \"roslaunch rubber_navigation baseOnly.launch publish_robot_source_odom_tf:=true\";exit;exec bash");

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        switch(joyTeleop.getControlTrigger())
        {
            case ManipulateOn:
                if(!listener.serviceCaller->srvCalling())
                    listener.serviceCaller->callSrv(visual_servo::manipulate::Request::CUT);
                break;
            case ChargeOn:
                if(!listener.serviceCaller->srvCalling())
                    listener.serviceCaller->callSrv(visual_servo::manipulate::Request::CHARGE);
                break;
            case UpOn:
                if(!listener.serviceCaller->srvCalling())
                    listener.serviceCaller->callSrv(visual_servo::manipulate::Request::UP);
                break;
            case HomeOn:
                if(!listener.serviceCaller->srvCalling())
                    listener.serviceCaller->callSrv(visual_servo::manipulate::Request::HOME);
                break;
			case CutBack:
				if(!listener.serviceCaller->srvCalling())
                    listener.serviceCaller->callSrv(visual_servo::manipulate::Request::CUTBACK);
                break;
			case LinearUp:
				if(!listener.serviceCaller->srvCalling())
					listener.serviceCaller->callSrv(visual_servo::manipulate::Request::LINEAR,Eigen::Vector3d(0.0,0.0,0.0015));
				break;
			case LinearDown:
				 if(!listener.serviceCaller->srvCalling())
					listener.serviceCaller->callSrv(visual_servo::manipulate::Request::LINEAR,Eigen::Vector3d(0.0,0.0,-0.0015));
				break;
			case LinearForward:
				if(!listener.serviceCaller->srvCalling())
					listener.serviceCaller->callSrv(visual_servo::manipulate::Request::LINEAR,Eigen::Vector3d(0.0,-0.002,0.0));
				break;
			case LinearBack:
				if(!listener.serviceCaller->srvCalling())
					listener.serviceCaller->callSrv(visual_servo::manipulate::Request::LINEAR,Eigen::Vector3d(0.0,0.002,0.0));
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
			case SteeringIn:
				ros::param::set("/visual_servo/steeringIn",listener.getParameter());
				break;
			case SteeringOut:
				ros::param::set("visual_servo/steeringOut",1.0);
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
                listener.pox_system("gnome-terminal -x bash -c \"roslaunch visual_servo visual_servo_real.launch\";exit;exec bash;");
                break;
            }
            case NavigationOn:
            {
                ros::V_string AllNodes;
                ros::master::getNodes(AllNodes);
                for(auto & node_name:AllNodes)
                {
                    if(node_name=="/rubber_navigation"||node_name=="/auboRobot"||node_name=="/base_only")
                    {
                        listener.pox_system(("rosnode kill " + node_name).c_str());
                    }
                }
				sleep(10);
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
                        std::string temp{ros::package::getPath("rubber_navigation")+"/config/map/map.bag.pbstream"};
                        std::string final_temp="gnome-terminal -x bash -c \"rosservice call /write_state \"filename: \'"+temp+"\' include_unfinished_submaps: \'true\'\"\";exit;exec bash;";
                        listener.pox_system(final_temp.c_str());
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