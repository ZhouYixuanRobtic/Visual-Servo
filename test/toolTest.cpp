//
// Created by xcy on 2019/10/17.
//
#include <unistd.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "ros/callback_queue.h"
#include "KeyboardTeleop.h"
#include "JoyTeleop.h"
using namespace JOYTELEOP;
int main(int argc, char* argv[])
{
    ros::init(argc,argv,"toolTest");
    ros::NodeHandle n;
    JoyTeleop joyTeleop("joy");
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        switch(joyTeleop.getControlTrigger())
        {
            case toolStart:
                ros::param::set("/visual_servo/isToolStarted",1.0);
                break;
            case toolWork:
                ros::param::set("/visual_servo/isToolStopped",1.0);
                break;
            case toolReset:
                ros::param::set("/visual_servo/isToolReset",1.0);
                break;
            case toolClear:
                ros::param::set("/visual_servo/toolAllClear",1.0);
                break;
            default:
                break;
        }
        joyTeleop.resetControlTrigger(JOYTELEOP::Default);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}