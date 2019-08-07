//
// Created by zyx on 19-4-1.
//

#include "Servo.h"

Servo::Servo()
{

}
Servo::~Servo()
{

}
/*
 * Function computes the end effector motion when achieving camera motion as a transform matrix
 * @param Trans_C2T         [the target pose with respect to camera described as a transform matrix]
 * @param Trans_E2C         [the camera pose with respect to end effector described as a transform matrix]
 * @param ExpectTrans_C2T   [the desired target pose with respect to camera described as a transform matrix]
 * @return the end effector motion described as a homogeneous matrix and its position error
*/
Destination_t Servo::GetCameraDestination(Eigen::Affine3d Trans_C2T,Eigen::Affine3d Trans_E2C, Eigen::Affine3d ExpectTrans_C2T)
{
    Eigen::Affine3d EndMotion;
    EndMotion=Trans_E2C*Trans_C2T*ExpectTrans_C2T.inverse()*Trans_E2C.inverse();
    //dynamic interpolation

    //interpolate scale factor
    double lambda=0.8;
    //the minimum tolerance of interpolation
    double Interpolate_tolerance=0.05;
    Eigen::Matrix3d R=EndMotion.rotation();
    Eigen::Vector3d t=EndMotion.translation();
    Sophus::SE3 DestinationSE3(R,t);
    //rotation sphere interpolate
    Eigen::Quaterniond Td=Sophus::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,0)).unit_quaternion().slerp(lambda,
            DestinationSE3.unit_quaternion());

    //translation linear interpolate
    for(int i=0;i<3;++i)
    {
     if(abs(t(i))>Interpolate_tolerance)
         t(i)=lambda*t(i);
    }
    Sophus::SE3 EndMotionDelta(Td,t);
    //increment
    Destination_t EndDestinationDelta{};
    EndDestinationDelta.EE_Motion.matrix()=EndMotionDelta.matrix();
    EndDestinationDelta.error=sqrt(EndMotion(0,3)*EndMotion(0,3)+EndMotion(1,3)*EndMotion(1,3)+
                                              EndMotion(2,3)*EndMotion(2,3));
    return EndDestinationDelta;
}


