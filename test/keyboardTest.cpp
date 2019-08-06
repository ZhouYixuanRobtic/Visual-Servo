//
// Created by xcy on 19-8-5.
//
#include <unistd.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "ros/callback_queue.h"
#include "KeyboardTeleop.h"

int main(int argc, char** argv)
{

    pthread_t camera_id;
    ros::init(argc, argv, "visual_servo");
    ros::NodeHandle n;

    KeyboardTeleop tbk;
    boost::thread t = boost::thread(boost::bind(&KeyboardTeleop::keyboardLoop, &tbk));

    ros::spin();
    t.interrupt();
    t.join();
    ros::shutdown();
    return 0;

}
