//
// Created by xcy on 2020/9/28.
//
#include "visual_servo/BLManager.h"

#include <cstdlib>
#include <csignal>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "ros/callback_queue.h"
#include <ros/package.h>
#include <std_msgs/String.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "tr1/memory"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"BLRosWrapper");
    ros::NodeHandle nh;
    ros::Publisher bl_content_pub = nh.advertise<std_msgs::String>("bl_content",10);
    ros::Rate loop_rate(30);
    BLManager blManager;
    while(ros::ok())
    {
        std_msgs::String blContent{};
        blContent.data=blManager.read();
        bl_content_pub.publish(blContent);
        ros::spinOnce();
        loop_rate.sleep();
    }
}