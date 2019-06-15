//
// Created by zyx on 19-6-4.
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
    /*
     * Specifies process types
     * The enum specifies two process types, OBLIQUE,VERTICAL
     * of processing image
     */
    enum ProcessType
    {
        OBLIQUE =1,
        VERTICAL=2
    };

    cv::Mat operate_image,display_image,obliqueImg,verticalImg;
    cv::Point obliqueMaxLoc,verticalMaxLoc;
    cv::Point borderPoint;

    /* Function thresholds the input oblique image in three channels
     * @param src   [the input image]
     * @return none
     * note: the input image is modified in this function
     */
    void oblique_threshold(cv::Mat src);

    /* Function thresholds the input vertical image in three channels
     * @param src   [the input image]
     * @return none
     * note: the input image is modified in this function
     */
    void vertical_threshold(cv::Mat src);

    /* Function processes the private variable operate_image in defined process type
     * @param processType   [process type, OBLIQUE for oblique knife image, VERTICAL for vertical knife image]
     * note: the private variable operate_image is modified in this function
     */
    cv::Mat pretreatment(int processType);

    /* Function gets the radon transform corresponding line
     * look up the complete coordinate online by (x,-1) or (-1,y).
     * @param priorRadonImgSize         [The size of the image not transformed]
     * @param subsequentRadonImgSize    [The size of the image already transformed]
     * @param referencePoint            [The randon transform maximum point]
     * @param targetPointX              [The x coordinate of point online you wanna get,-1 for non-use]
     * @param targetPointY              [The y coordinate of point online you wanna get,-1 for non-use]
     * @return the complete coordinate of a point online known one coordinate.
     */
    cv::Point getPointOnline(const cv::Size& priorRadonImgSize,const cv::Size& subsequentRadonImgSize,
                            const cv::Point& referencePoint,const int& targetPointX,const int& targetPointY=-1);
    /*
     * Function refines the intersection coordinate
     */
    void refineLoc();


    /*
     * Function computes all coordinates of the knife trace
     */
    std::vector<cv::Point> get_knife_trace(bool debug=false);

public:
    Detector();
    virtual ~Detector();
    int radonAngleRange;
    int radonOperation;

    cv::Point beginPoint;
    /*
     * Function computes the begin point of knife trace which is the intersection of
     * the oblique knife trace and the vertical knife trace
     * @return the pixel coordinate of the begin point.
     */
    cv::Point get_BeginPoint(const cv::Mat& test_image_,bool show_result=false);
    /*
     * Function gets the complete knife Trace (the oblique one)
     * @param test_image_   [the input test image]
     * @return all coordinates of the knife trace
     */
    std::vector<cv::Point> get_knifeTrace(const cv::Mat& test_image_,bool show_result=false);
};


#endif //VISUAL_SERVO_DETECTOR_H
