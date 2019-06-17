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
    bool callSrv();
};
Listener::Listener()
{
    client =nh.serviceClient<visual_servo::manipulate>("manipulate");
    srv.request.type=visual_servo::manipulate::Request::CUT;
}
Listener::~Listener()
{
}
bool Listener::callSrv()
{
    if (client.call(srv))
    {
        if(srv.response.status==visual_servo::manipulate::Response::SUCCESS)
            ROS_INFO("TEST SUCCESS");
        else
            ROS_INFO("TEST ERROR");
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
    bool ready_go=true;
    Listener listener;
    if(ready_go)
        ready_go=!listener.callSrv();
    ros::spin();
    ros::shutdown();
    return 0;
}
