//
// Created by zyx on 19-6-17.
//

#include <unistd.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "ros/callback_queue.h"
#include "visual_servo/manipulate.h"

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
        switch (srv.response.status)
        {
            case visual_servo::manipulate::Response::SUCCESS:
                ROS_INFO("SUCCESS");
                /*Do something when success*/
                break;
            case visual_servo::manipulate::Response::ERROR:
                ROS_INFO("ERROR");
                break;
            case visual_servo::manipulate::Response::ABORT:
                /*Do something when abort*/
                break;
            default:
                ROS_ERROR("Wrong response status");
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
    while(ros::ok())
    {
        listener.callSrv(visual_servo::manipulate::Request::CUT);
        sleep(5);
    }

    ros::spin();
    ros::shutdown();
    return 0;
}
