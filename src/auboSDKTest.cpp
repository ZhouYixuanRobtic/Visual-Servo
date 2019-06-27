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
#include "visual_servo/VisualServoMetaTypeMsg.h"
#include "VisualServoMetaType.h"
extern bool ExitSoftEmergency;
extern bool RobotStartup;
extern bool RobotShutDown;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "auboSDK");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    visual_servo::VisualServoMetaTypeMsg status;
    ros::Publisher status_pub;
    status_pub = nh.advertise<visual_servo::VisualServoMetaTypeMsg>("VisualServoStatus", 100);
    AuboSDK auboSdk;
    status.RobotAllRight = auboSdk.loginSucceed && auboSdk.robotStartUp();
    while (ros::ok())
    {
        // auboSdk.getSwitchStatus();
        if(RobotShutDown&&ExitSoftEmergency)
        {
            status.RobotAllRight=false;
            if(status.RobotAllRight=auboSdk.robotStartUp())
                ExitSoftEmergency=false;
            /*急停时应中断所有操作*/
        }
        if(status.RobotAllRight)
            status.RobotSwitchOn=auboSdk.getSwitchStatus();

        status_pub.publish(status);
        loop_rate.sleep();
        ros::spinOnce();
    }
    if(status.RobotAllRight)
        auboSdk.robotShutDown();
    ros::shutdown();
}