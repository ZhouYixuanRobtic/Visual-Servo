//
// Created by zyx on 19-4-1.
//
#ifndef _SERVO_H_
#define _SERVO_H_

#include "sophus/se3.h"
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "ros/callback_queue.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include "VisualServoMetaType.h"
using namespace cv;


class Servo
{

private:
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener *tfListener_;

public:
    Servo();
    virtual ~Servo();

    void getTransform(const std::string source_frame,const std::string target_frame,Eigen::Affine3d &transformMatrix);
    Eigen::Affine3d getTransform(const std::string source_frame,const std::string target_frame);
    /*
     * Function computes the end effector motion when achieving camera motion as a transform matrix
     * @param Trans_C2T         [the target pose with respect to camera described as a transform matrix]
     * @param Trans_E2C         [the camera pose with respect to end effector described as a transform matrix]
     * @param ExpectTrans_C2T   [the desired target pose with respect to camera described as a transform matrix]
     * @return the end effector motion described as a homogeneous matrix and its position error
     */
    Destination_t getCameraEE (Eigen::Affine3d Trans_C2T,Eigen::Affine3d Trans_E2C, Eigen::Affine3d ExpectTrans_C2T,double lambda=0.8);

};


#endif
