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
        transformMatrix=tf2::transformToEigen(transformStamped.transform);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        //ros::Duration(1.0).sleep();
    }
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

    Eigen::Matrix3d R{EndMotion.rotation()};
    Eigen::Vector3d t{EndMotion.translation()};
    // Interpolation with rotation slerp and linear translation
    Sophus::SO3 goal(R);
    auto interminate_rotation = Sophus::SO3::exp(lambda * goal.log());
    Eigen::Vector3d interminate_translation = lambda*t;
    Sophus::SE3 EndMotionDelta(goal,t);
    //increment
    EndDestinationDelta.EE_Motion.matrix()=EndMotionDelta.matrix();
    Sophus::Vector6d error_vector{EndMotionDelta.log()};
    EndDestinationDelta.error_log=error_vector;

    // orientation error
    double orientation_error =  interminate_rotation.log().norm();
    double translation_error = interminate_translation.norm();
    // THis formula could be adjusted
    EndDestinationDelta.error = orientation_error;
	std::cout<<"error: "<<EndDestinationDelta.error<<std::endl;
	return EndDestinationDelta;
}

