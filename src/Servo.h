//
// Created by xzy on 19-4-1.
//
#ifndef _SERVO_H_
#define _SERVO_H_

#include <stdio.h>
#include <stdlib.h>  // exit
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <math.h>
#include <vector>

#include <string.h>   		// bzero
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include <OpenNI.h>
#include <NiTE.h>

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
using namespace std;
using namespace cv;


struct Destination_t{
    Eigen::Affine3d EE_Motion;
    double error;
};
struct TagDetectInfo{
    Eigen::Affine3d Trans_C2T;
    int id;
    double PixelCoef;
    Point2d Center;
};
class Servo
{

private:

    double fx,fy,u0,v0;

public:
    Servo();
    virtual ~Servo();

    vector<TagDetectInfo> GetTargetPoseMatrix(Mat UserImage, double TagSize);
    Destination_t GetCameraDestination (Eigen::Affine3d Trans_C2T,Eigen::Affine3d Trans_E2C, Eigen::Affine3d ExpectTrans_C2T);
    void SetCameraParameter(const double fx,const double fy, const double u0, const double v0);
};


#endif
