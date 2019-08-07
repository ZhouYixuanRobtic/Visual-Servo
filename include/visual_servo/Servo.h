//
// Created by zyx on 19-4-1.
//
#ifndef _SERVO_H_
#define _SERVO_H_

#include "sophus/se3.h"

#include "VisualServoMetaType.h"
using namespace cv;


class Servo
{

private:

public:
    Servo();
    virtual ~Servo();

    /*
     * Function computes the end effector motion when achieving camera motion as a transform matrix
     * @param Trans_C2T         [the target pose with respect to camera described as a transform matrix]
     * @param Trans_E2C         [the camera pose with respect to end effector described as a transform matrix]
     * @param ExpectTrans_C2T   [the desired target pose with respect to camera described as a transform matrix]
     * @return the end effector motion described as a homogeneous matrix and its position error
     */
    Destination_t GetCameraDestination (Eigen::Affine3d Trans_C2T,Eigen::Affine3d Trans_E2C, Eigen::Affine3d ExpectTrans_C2T);
};


#endif
