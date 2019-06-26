//
// Created by xcy on 19-6-26.
//
#include <iostream>

#include <stdio.h>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "ros/callback_queue.h"

#include "AuboSDK.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "auboSDK");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    AuboSDK auboSdk;
    if(auboSdk.robotStartUp())
    {
        auboSdk.robotEventRegister();
        while (ros::ok())
        {
           // auboSdk.getSwitchStatus();
            loop_rate.sleep();
            ros::spinOnce();
        }
        auboSdk.robotShutDown();
    }
    else
    {
        ROS_INFO("launch failed");
    }
    ros::shutdown();

}