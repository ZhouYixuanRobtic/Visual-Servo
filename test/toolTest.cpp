//
// Created by xcy on 2019/10/17.
//
#include <unistd.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "ros/callback_queue.h"
#include "KeyboardTeleop.h"
#include "ControlTeleop.h"
using namespace CONTROLTELEOP;
int main(int argc, char* argv[])
{
    ros::init(argc,argv,"toolTest");
    ros::NodeHandle n;
    ControlTeleop controlTeleop;
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        switch(controlTeleop.getControlTrigger())
        {
            case ClockGo:
                ros::param::set("/visual_servo/clockGo",1.0);
                break;
            case AntiClockGo:
                ros::param::set("/visual_servo/antiClockGo",1.0);
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
            default:
                break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}