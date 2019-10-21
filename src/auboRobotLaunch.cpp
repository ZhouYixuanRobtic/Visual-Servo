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
#include "KeyboardTeleop.h"
#include "parameterTeleop.h"

extern bool ExitSoftEmergency;
extern bool RobotMoveStop;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "auboSDK");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    bool isMoveStop=false;
    ParameterListener parameterListener(40,8);
    const std::vector<std::string> parameterNames{"/visual_servo/isChargingStatusChanged","/visual_servo/isToolStarted","/visual_servo/isToolStopped","/visual_servo/isToolReset","/visual_servo/toolAllClear"};
    parameterListener.registerParameterCallback(parameterNames,false);

    KeyboardTeleop tbk;

    visual_servo::VisualServoMetaTypeMsg status;
    ros::Publisher status_pub;
    status_pub = nh.advertise<visual_servo::VisualServoMetaTypeMsg>("VisualServoStatus", 100);

    AuboSDK auboSdk;
    status.RobotAllRight = auboSdk.loginSucceed && auboSdk.robotStartUp();
    std::cout<<"RobotAllRight "<<(int) status.RobotAllRight<<std::endl;

    while (ros::ok())
    {
        // auboSdk.getSwitchStatus();
        if((bool) parameterListener.parameters()[0])
        {
            auboSdk.OverturnIOStatus();
            ros::param::set(parameterNames[0],(double)false);
        }
        if((bool) parameterListener.parameters()[1])
        {
            auboSdk.toolStart();
            ros::param::set(parameterNames[1], (double)false);
        }
        if((bool) parameterListener.parameters()[2])
        {
            auboSdk.toolStop();
            ros::param::set(parameterNames[2], (double)false);
        }
        if((bool) parameterListener.parameters()[3])
        {
            auboSdk.toolReset();
            ros::param::set(parameterNames[3], (double)false);
        }
        if((bool) parameterListener.parameters()[4])
        {
            auboSdk.toolAllclear();
            ros::param::set(parameterNames[4],(double)false);
        }
        if(tbk.moveChange)
        {

            if(!isMoveStop)
            {
                isMoveStop=auboSdk.robotFastMoveStop();
                std::cout<<"机械臂已刹车"<<std::endl;
            }
            else
            {
                isMoveStop=!auboSdk.robotFastMoveRelease();
                std::cout<<"机械臂松刹车"<<std::endl;
            }
            tbk.moveChange=false;

        }
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
        status_pub.publish(status);
        loop_rate.sleep();
        ros::spinOnce();
    }
    if(auboSdk.loginSucceed)
        auboSdk.robotShutDown();
    sleep(10);
    ros::shutdown();
}
