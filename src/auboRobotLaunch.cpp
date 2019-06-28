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
extern bool RobotMoveStop;
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
    std::cout<<"RobotAllRight "<<(int) status.RobotAllRight<<std::endl;

    while (ros::ok())
    {
        // auboSdk.getSwitchStatus();
        if(RobotMoveStop)
        {
            status.RobotAllRight=false;
            if(ExitSoftEmergency)
            {
                if(status.RobotAllRight=auboSdk.robotStartUp())
                    ExitSoftEmergency=false;
            }
            /*急停时应中断所有操作*/
        }
        if(auboSdk.loginSucceed)
            status.RobotSwitchOn=auboSdk.getSwitchStatus();

        status_pub.publish(status);
        loop_rate.sleep();
        ros::spinOnce();
    }
    if(auboSdk.loginSucceed)
        auboSdk.robotShutDown();
    sleep(5);
    ros::shutdown();
}