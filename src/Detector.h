//
// Created by xcy on 19-6-4.
//

#ifndef VISUAL_SERVO_DETECTOR_H
#define VISUAL_SERVO_DETECTOR_H

#include <math.h>
#include <stdlib.h>
#include <string>
#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>

#include <opencv2/ximgproc.hpp>
#include "radon_transform.hpp"

using namespace std;

class Detector{
private:
    enum ProcessType
    {
        OBLIQUE =1,
        VERTICAL=2
    };
    cv::Mat test_image,obliqueImg,verticalImg;
    cv::Point obliqueMaxLoc,verticalMaxLoc;

    void oblique_threshold(cv::Mat src);
    void vertical_threshold(cv::Mat src);
    cv::Mat pretreatment(int processType);
    cv::Point getBeginLoc();

public:
    Detector();
    virtual ~Detector();
    int radonAngleRange;
    int radonOperation;
    cv::Point get_BeginPoint(cv::Mat test_image);
};


#endif //VISUAL_SERVO_DETECTOR_H
