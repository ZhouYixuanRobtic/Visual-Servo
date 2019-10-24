#include <unistd.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "ros/callback_queue.h"
#include "visual_servo/manipulate.h"
#include "VisualServoMetaType.h"
#include "KeyboardTeleop.h"
#include "JoyTeleop.h"
#include "../src/JoyTeleop.cpp"
using namespace JOYTELEOP;
class Listener
{
private:
    ros::NodeHandle nh;
    ros::ServiceClient client;
    visual_servo::manipulate srv;
public:
    Listener();
    virtual ~Listener();
    bool callSrv(int SrvRequestType);
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
                return false;
        }
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service detect_once");
        return false;
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "maTest");
    Listener listener;
    JoyTeleop joyTeleop("joy");
    
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        switch(joyTeleop.getControlTrigger())
        {
            case ManipulateOn:
                listener.callSrv(visual_servo::manipulate::Request::CUT);
                break;
            case ChargeOn:
                ros::param::set("/user/inverse",true);
                listener.callSrv(visual_servo::manipulate::Request::CHARGE);
                ros::param::set("/user/inverse",false);
                break;
            case UpOn:
                listener.callSrv(visual_servo::manipulate::Request::UP);
                break;
            case HomeOn:
                listener.callSrv(visual_servo::manipulate::Request::HOME);
                break;
            case toolStart:
                ros::param::set("/visual_servo/isToolStarted",1.0);
                std::cout<<"tool start is send"<<std::endl;
                break;
            case toolWork:
                ros::param::set("/visual_servo/isToolStopped",1.0);
                std::cout<<"tool work is send"<<std::endl;
                break;
            case toolReset:
                ros::param::set("/visual_servo/isToolReset",1.0);
                std::cout<<"tool reset is send"<<std::endl;
                break;
            case toolClear:
                ros::param::set("/visual_servo/toolAllClear",1.0);
                std::cout<<"tool clear is send"<<std::endl;
                break;
            default:
                break;
        }
        joyTeleop.resetControlTrigger(JOYTELEOP::Default);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}