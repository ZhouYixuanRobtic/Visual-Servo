//
// Created by zyx on 19-4-1.
//
#ifndef _SERVO_H_
#define _SERVO_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <math.h>
#include <vector>

#include <string.h>
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cv.h"
#include "highgui.h"

#include <dirent.h>

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
#include "apriltag_pose.h"
}

#include "sophus/se3.h"
#include <sstream>

using namespace cv;

struct Destination_t{
    //the end effector motion described as a homogeneous matrix
    Eigen::Affine3d EE_Motion;
    //the position error with respect to expected position
    double error;
};

struct TagDetectInfo{
    //the tag pose with respect to camera described as a homogeneous matrix
    Eigen::Affine3d Trans_C2T;
    //the tag id
    int id;
    //the coefficient unit meters per pixel
    double PixelCoef;
    //the pixel point of tag center
    Point2d Center;
};
class Servo
{

private:

    //camera intrinsic parameter
    double fx,fy,u0,v0;

public:
    Servo();
    virtual ~Servo();

    /*
     * Function computes tag information of all tags detected
     * @param UserImage [the image prepared to detect tag]
     * @param TagSize   [the physic size of tag unit meters]
     * @return a vector contains all tag information detected, empty when no tags detected
     */
    std::vector<TagDetectInfo> GetTargetPoseMatrix(Mat UserImage, double TagSize);
    /*
     * Function computes the end effector motion when achieving camera motion as a transform matrix
     * @param Trans_C2T         [the target pose with respect to camera described as a transform matrix]
     * @param Trans_E2C         [the camera pose with respect to end effector described as a transform matrix]
     * @param ExpectTrans_C2T   [the desired target pose with respect to camera described as a transform matrix]
     * @return the end effector motion described as a homogeneous matrix and its position error
     */
    Destination_t GetCameraDestination (Eigen::Affine3d Trans_C2T,Eigen::Affine3d Trans_E2C, Eigen::Affine3d ExpectTrans_C2T);
    /*
     * Function sets camera intrinsic parameter
     * @param fx,fy,u0,v0   [camera intrinsic parameter]
     */
    void SetCameraParameter(const double fx,const double fy, const double u0, const double v0);
};


#endif
