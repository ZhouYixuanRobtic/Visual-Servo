//
// Created by xcy on 19-6-4.
//

#include "Detector.h"
Detector::Detector()
{
    radonAngleRange=63;
    radonOperation=radon::RT_SUM;
}
Detector::~Detector()
{

}
void Detector::oblique_threshold(cv::Mat src)
{
    const int R_MAX_1=170;
    cv::Mat_<Vec3b>::iterator it1 = src.begin<Vec3b>();
    while (it1 != src.end<Vec3b>())
    {
        if ((*it1)[2]-(*it1)[1] > 20 && (*it1)[2]-(*it1)[0] > 20 && (*it1)[2] > R_MAX_1)
        {
            (*it1)[0] = 255;
            (*it1)[1] = 255;
            (*it1)[2] = 255;
        }
        else
        {
            (*it1)[0] = 0;
            (*it1)[1] = 0;
            (*it1)[2] = 0;
        }
        it1++;
    }
}
void Detector::vertical_threshold(cv::Mat src)
{
    cv::Mat_<Vec3b>::iterator it1 = src.begin<Vec3b>();
    while (it1 != src.end<Vec3b>())
    {
        if ((*it1)[2]-(*it1)[1] > 15 && (*it1)[2]-(*it1)[0] > 0 && (*it1)[1]-(*it1)[0] < 15)
        {
            (*it1)[0] = 255;
            (*it1)[1] = 255;
            (*it1)[2] = 255;
        }
        else
        {
            (*it1)[0] = 0;
            (*it1)[1] = 0;
            (*it1)[2] = 0;
        }
        it1++;
    }
}
cv::Mat Detector::pretreatment(int processType)
{
    double minVal, maxVal;
    cv::Point minLoc,maxLoc;
    cv::Mat gaussianImg,grayImg;

    cv::GaussianBlur(this->test_image,gaussianImg,Size(5,5),0,0,cv::BORDER_DEFAULT);
    switch (processType)
    {
        case OBLIQUE:
            oblique_threshold(gaussianImg);
            break;
        case VERTICAL:
            vertical_threshold(gaussianImg);
            break;
        default:
            break;
    }
    cv::cvtColor(gaussianImg,grayImg,CV_BGRA2GRAY);

    radon::radonTransform(grayImg,grayImg,radonAngleRange,radonOperation);

    cv::minMaxLoc(grayImg, &minVal, &maxVal);
    grayImg -= minVal;
    grayImg.convertTo(grayImg, CV_8U, 255.0/(maxVal-minVal) );

    return grayImg;
}
cv::Point Detector::getBeginLoc()
{
    int x0,y0,x1,y1,x2,y2;
    float k;
    x0 = this->test_image.size().width/2-((obliqueImg.size().height/2-obliqueMaxLoc.y)*cos(M_PI*((270-obliqueMaxLoc.x)-90)/180));
    y0 = -((this->test_image.size().height/2)+((obliqueImg.size().height/2-obliqueMaxLoc.y)*sin(M_PI*((270-obliqueMaxLoc.x)-90)/180)));
    k = tan(M_PI*(270-obliqueMaxLoc.x)/180);
    x1 = 0;
    y1 = -k*x0+y0;
    x2 = (-this->test_image.size().height-y0)/k+x0;
    y2 = this->test_image.size().height;
    Point p0(x0,-y0),p1(x1,-y1),p2(x2,y2);
    int x_1;
    x_1=obliqueImg.size().height/2-verticalMaxLoc.y+this->test_image.size().width/2;

    cv::line(this->test_image, p1, p2, Scalar(0, 0, 255), 2);
    cv::line(this->test_image, Point(x_1,0), Point(x_1,this->test_image.size().height), Scalar(0, 255, 0), 2);

    return cv::Point(x_1,-(k*(x_1-x0)+y0));
}
cv::Point Detector::get_BeginPoint(cv::Mat test_image)
{
    this->test_image=test_image.clone();

    obliqueImg=pretreatment(OBLIQUE);
    verticalImg=pretreatment(VERTICAL);

    double minVal,maxVal;
    cv::Point minLoc,maxLoc;

    minMaxLoc(obliqueImg, &minVal, &maxVal, &minLoc, &maxLoc);
    obliqueMaxLoc=maxLoc;

    minMaxLoc(verticalImg.col(0), &minVal, &maxVal, &minLoc, &maxLoc);
    verticalMaxLoc=maxLoc;

    cv::Point beginPoint=getBeginLoc();
    cv::circle(this->test_image,beginPoint,10,Scalar(255,0,0),-1);
    cv::imshow("detector",this->test_image);
    cv::waitKey(3);
    return beginPoint;

}