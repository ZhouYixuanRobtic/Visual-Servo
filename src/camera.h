/******************************************************

******************************************************/
#ifndef _CAMERA_H_
#define _CAMERA_H_

#include <stdio.h>
#include <stdlib.h>  // exit
#include <iostream>
#include <math.h>
#include <vector>

#include <string.h>        // bzero
#include "Eigen/Dense"
#include "Draw.h"

#include <OpenNI.h>
#include <NiTE.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*#include <pcl/io/pcd_io.h>  
#include <pcl/io/ply_io.h>  
#include <pcl/point_types.h>*/

#include <sstream>

const float fxl = 617.622;
const float fyl = 614.82;
const float cxl = 349.287;
const float cyl = 259.493;

const float fxr = 568.915;
const float fyr = 570.225;
const float cxr = 362.32;
const float cyr = 234.534;

const double Transform[3][3] = {
        {(double) 0.9995,  (double) -0.0025, (double) -0.0307},
        {(double) -0.0000, (double) 0.9968,  (double) -0.0797},
        {(double) 0.0308,  (double) 0.0796,  (double) 0.9963}
};
const double Position[3] = {(double) -0.000000, (double) 0.000000, (double) -0.5200834};
const double dsf[4][4] = {
        {0.99996,     0.00346245, -0.00821128, -24.1927},
        {-0.00355971, 0.999923,   -0.0118598,  -1.29574},
        {0.00816959,  0.0118885,  0.999896,    11.025},
        {0,           0,          0,           1}
};
const float th = 0.508;
#define TXT_PATH1 "/home/xcy/cmake_project/project/build/record.txt"
#define TXT_PATH2 "/home/xcy/cmake_project/project/build/record2.txt"

class Camera {
/*
主要来完成深度彩色数据的获取
*/
private:

    //摄像头设备
    openni::Device mdevAnyDevice_;
    //深度和彩色数据流
    openni::VideoStream mDepthStream_;
    openni::VideoStream mColorStream_;

    // 创建用户跟踪器
    nite::UserTracker mUserTracker_;

    // 循环读取数据流信息并保存在VideoFrameRef中
    openni::VideoFrameRef mDepthFrame_;
    openni::VideoFrameRef mColorFrame_;
    nite::UserTrackerFrameRef mUserFrame_;

    bool bSave_Pointcloud_;
    bool bSave_ColorPointcloud_;
    bool bSave_CloudFile_ = true;
    bool bSave_Image_ = true;
    bool bSave_Map_ = true;
    bool bSave_MapFile_ = true;

    Eigen::Matrix<double, 4, 4> R2L_Mat;
    Eigen::Matrix<double, 4, 4> RK_Mat;
    Eigen::Matrix<double, 4, 4> LK_Mat;
    Eigen::Matrix<double, 4, 4> All_Mat;

    double d2c[16];
    int m_W;
    int m_H;
    Draw mDraw;

    void CameraInit();

public:
    Camera();

    ~Camera();

    cv::Mat ShowColor();

    cv::Mat ShowDepth();

    int getNextFrame(cv::Mat &color, cv::Mat &ir, cv::Mat &depth);

    bool toggleRegister(bool setRegister);

    void checkLines(cv::Mat &CaliDepth, std::vector <cv::Vec4i> &lines, cv::Mat &show);

    void d2cRegister(const cv::Mat &color, cv::Mat &CaliDepth);

    void MappingDepth2Color(cv::Mat &src, cv::Mat &dst);

    std::vector <cv::Point3d>
    getPositions(const cv::Mat &src, const cv::Mat &CaliDepth, const std::vector <cv::Point> &crossPoints);

    // Bresenham's line algorithm
    bool GetLinePts(int x1, int y1, int x2, int y2, std::vector <cv::Point> &vPts);

    inline void swap_int(int *a, int *b) {
        *a ^= *b;
        *b ^= *a;
        *a ^= *b;
    }

    bool Run(bool bcolor = true, bool bdepth = true, bool bMap = true);

    //等待有新的帧出现
    bool WaitForNew();
};


#endif







