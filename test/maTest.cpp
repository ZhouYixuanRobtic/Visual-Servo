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
    KeyboardTeleop tbk;
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        if(tbk.maOn)
        {
            listener.callSrv(visual_servo::manipulate::Request::CUT);
            //std::cout<<"this is a try "<<std::endl;
            tbk.maOn=false;
        }
        if(tbk.chargeOn)
        {
            listener.callSrv(visual_servo::manipulate::Request::CHARGE);
            tbk.chargeOn=false;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}