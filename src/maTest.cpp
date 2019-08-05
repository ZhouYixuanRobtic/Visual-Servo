#include <unistd.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "ros/callback_queue.h"
#include "visual_servo/manipulate.h"
#include "VisualServoMetaType.h"
#include "KeyboardTeleop.h"

class Listener
{
private:
    ros::NodeHandle nh;
    ros::ServiceClient client;
    visual_servo::manipulate srv;
    static void printServiceStatus(const int & ServiceStatus);
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
void Listener::printServiceStatus(const int & ServiceStatus)
{
    switch(ServiceStatus)
    {
        case visual_servo_namespace::SERVICE_STATUS_SUCCEED:
            std::cout<<"Service call succeed and manipulate succeed"<<std::endl;
            break;
        case visual_servo_namespace::SERVICE_STATUS_NO_TAG:
            std::cout<<"Service call failed because no tag searched"<<std::endl;
            break;
        case visual_servo_namespace::SERVICE_STATUS_CLOSE_FAILED:
            std::cout<<"Service call failed because can't get close"<<std::endl;
            break;
        case visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED:
            std::cout<<"Service call failed because can't servo to right place"<<std::endl;
            break;
        case visual_servo_namespace::SERVICE_STATUS_CUT_FAILED:
            std::cout<<"Service call failed because can't cut "<<std::endl;
            break;
        case visual_servo_namespace::SERVICE_STATUS_ROBOT_ABORT:
            std::cout<<"Service call failed because robot abort "<<std::endl;
            break;
        default:
            std::cout<<"Service call succeed but no response"<<std::endl;
            break;
    }
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
    KeyboardTeleop tbk;
    boost::thread t= boost::thread(boost::bind(&KeyboardTeleop::keyboardLoop,&tbk));

    while(ros::ok())
    {
        if(tbk.maOn)
        {
            listener.callSrv(visual_servo::manipulate::Request::CUT);
            tbk.maOn=false;
        }
        if(tbk.chargeOn)
        {
            listener.callSrv(visual_servo::manipulate::Request::CHARGE);
            tbk.chargeOn=false;
        }
    }

    ros::spin();
    t.interrupt();
    t.join();
    ros::shutdown();
    return 0;
}