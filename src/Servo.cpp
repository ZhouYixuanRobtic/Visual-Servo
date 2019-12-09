//
// Created by zyx on 19-4-1.
//

#include "Servo.h"

Servo::Servo()
{
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
    EndDestinationDelta = Destination_t{};
}
Servo::~Servo()
{
    delete tfListener_;
}
void Servo::getTransform(const std::string & target_frame,const std::string & source_frame,Eigen::Affine3d &transformMatrix)
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer_.lookupTransform(target_frame, source_frame,
                                                ros::Time(0),ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        //ros::Duration(1.0).sleep();
    }
    transformMatrix=tf2::transformToEigen(transformStamped.transform);
}
Eigen::Affine3d Servo::getTransform(const std::string & target_frame,const std::string & source_frame)
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer_.lookupTransform(target_frame, source_frame,
                                                     ros::Time(0),ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        //ros::Duration(1.0).sleep();
    }
    return tf2::transformToEigen(transformStamped.transform);
}
/*
 * Function computes the end effector motion when achieving camera motion as a transform matrix
 * @param Trans_C2T         [the target pose with respect to camera described as a transform matrix]
 * @param Trans_E2C         [the camera pose with respect to end effector described as a transform matrix]
 * @param ExpectTrans_C2T   [the desired target pose with respect to camera described as a transform matrix]
 * @return the end effector motion described as a homogeneous matrix and its position error
*/
const Destination_t & Servo::getCameraEE(const Eigen::Affine3d & Trans_C2T,const Eigen::Affine3d & Trans_E2C, const Eigen::Affine3d & ExpectTrans_C2T,double lambda)
{
    Eigen::Affine3d EndMotion{Trans_E2C*Trans_C2T*ExpectTrans_C2T.inverse()*Trans_E2C.inverse()};
    //dynamic interpolation

    //the minimum tolerance of interpolation
    double Interpolate_tolerance=0.05;
    Eigen::Matrix3d R{EndMotion.rotation()};
    Eigen::Vector3d t{EndMotion.translation()};
    Sophus::SE3 DestinationSE3(R,t);
    //rotation sphere interpolate
    Eigen::Quaterniond Td{Sophus::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,0)).unit_quaternion().slerp(lambda,
            DestinationSE3.unit_quaternion())};

    //translation linear interpolate
    for(int i=0;i<3;++i)
    {
     if(abs(t(i))>Interpolate_tolerance)
         t(i)=lambda*t(i);
    }
    Sophus::SE3 EndMotionDelta(Td,t);
    //increment
    EndDestinationDelta.EE_Motion.matrix()=EndMotionDelta.matrix();
    Sophus::Vector6d error_vector{EndMotionDelta.log()};
    error_vector[0]=0.0;
    error_vector[1]=0.0;
    error_vector[2]=0.0;
    EndDestinationDelta.error=error_vector.norm();
    return EndDestinationDelta;
}

