//
// Created by xcy on 2019/10/17.
//
#include <unistd.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "ros/callback_queue.h"
#include "KeyboardTeleop.h"
int main(int argc, char* argv[])
{
    ros::init(argc,argv,"toolTest");
    ros::NodeHandle n;
    KeyboardTeleop tbk;
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        if(tbk.maOn)
        {
            ros::param::set("/visual_servo/isToolStarted",1.0);
            tbk.maOn=false;
            ROS_INFO("Tool Start Send!!!");
        }
        if(tbk.goHomeOn)
        {
            ros::param::set("/visual_servo/isToolStopped",1.0);
            tbk.goHomeOn=false;
            ROS_INFO("Tool Stop Send!!!");
        }
        if(tbk.goUpOn)
        {
            ros::param::set("/visual_servo/isToolReset",1.0);
            tbk.goUpOn=false;
            ROS_INFO("Tool ResetSend!!!!");
        }
        if(tbk.navOn)
        {
            ros::param::set("/visual_servo/toolAllClear",1.0);
            tbk.navOn=false;
            ROS_INFO("Tool all clear send");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}