//
// Created by xcy on 19-6-4.
//

#include "Detector.h"
Detector::Detector()
{
    radonAngleRange=63;
    radonOperation=radon::RT_SUM;
    beginPoint=cv::Point(-1,-1);
}
Detector::~Detector()
{

}
/*
 * Function thresholds the input oblique image in three channels
 * @param src   [the input image]
 * @return none
 * note: the input image is modified in this function
*/
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
/*
 * Function thresholds the input vertical image in three channels
 * @param src   [the input image]
 * @return none
 * note: the input image is modified in this function
*/
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
/*
 * Function processes the private variable operate_image in defined process type
 * @param processType   [process type, OBLIQUE for oblique knife image, VERTICAL for vertical knife image]
 * note: the private variable operate_image is modified in this function
*/
cv::Mat Detector::pretreatment(int processType)
{
    double minVal, maxVal;
    cv::Mat gaussianImg,grayImg,grayImgRadon;

    cv::GaussianBlur(this->operate_image,gaussianImg,Size(5,5),0,0,cv::BORDER_DEFAULT);
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

    radon::radonTransform(grayImg,grayImgRadon,radonAngleRange,radonOperation);

    cv::minMaxLoc(grayImgRadon, &minVal, &maxVal);
    grayImgRadon -= minVal;
    grayImgRadon.convertTo(grayImgRadon, CV_8UC1, 255.0/(maxVal-minVal) );

    return grayImgRadon;
}
/*
 * Function gets the radon transform corresponding line
 * look up the complete coordinate online by (x,-1) or (-1,y).
 * @param priorRadonImgSize         [The size of the image not transformed]
 * @param subsequentRadonImgSize    [The size of the image already transformed]
 * @param referencePoint            [The randon transform maximum point]
 * @param targetPointX              [The x coordinate of point online you wanna get,-1 for non-use]
 * @param targetPointY              [The y coordinate of point online you wanna get,-1 for non-use]
 * @return the complete coordinate of a point online known one coordinate.
*/
cv::Point Detector::getPointOnline(const cv::Size& priorRadonImgSize,const cv::Size& subsequentRadonImgSize,
                                    const cv::Point& referencePoint,const int& targetPointX,const int& targetPointY)
{
    int x0,y0;
    double k;
    double beta=M_PI*(obliqueMaxLoc.x)/180.0;
    int offset=subsequentRadonImgSize.height/2-referencePoint.y;
    x0 = priorRadonImgSize.width/2+offset*cos(beta);
    y0 = priorRadonImgSize.height/2+offset*sin(beta);
    k = -1/tan(beta);

    if(targetPointX>=0)
        return cv::Point(targetPointX,k*(targetPointX-x0)+y0);
    else if(targetPointY>=0)
        return cv::Point((targetPointY-y0)/k+x0,targetPointY);
}
/*
 * Function refines the intersection coordinate
*/
void Detector::refineLoc()
{
    //image pretreatment
    cv::Mat gaussianImg,grayImg;
    cv::GaussianBlur(this->operate_image,gaussianImg,Size(15, 15), 0, 0, cv::BORDER_DEFAULT);
    cv::cvtColor(gaussianImg,grayImg,CV_BGRA2GRAY);
    cv::Canny(grayImg,grayImg,5,40,3);

    cv::Size RectSize(200,160);
    cv::Point initPoint(beginPoint.x-RectSize.width/1.5,beginPoint.y-RectSize.height/1.5);
    cv::Rect rect(initPoint,RectSize);

    cv::Mat refineImg=grayImg(rect);
    cv::Mat refineImgRadon;
    radon::radonTransform(refineImg,refineImgRadon,radonAngleRange,radonOperation);

    cv::Point maxLoc;
    cv::Mat mask=Mat::zeros(refineImgRadon.size(),CV_8UC1);
    mask(cv::Rect(cv::Point(20,0),cv::Size(130,refineImgRadon.size().height))).setTo(255);
    cv::minMaxLoc(refineImgRadon, nullptr, nullptr, nullptr,&maxLoc,mask);
    cv::Point refinedBeginPoint=getPointOnline(refineImg.size(),refineImgRadon.size(),maxLoc,(beginPoint-initPoint).x,-1);
    beginPoint=refinedBeginPoint+initPoint;
}
/*
 * Function draws a box described by a CvBox2D
 * @param box [the box properties]
 * @param img [the canva]
 * @return none
*/
static void DrawBox(CvBox2D box, cv::Mat img)
{
    CvPoint2D32f point[4];

    cvBoxPoints(box, point); //计算二维盒子顶点
    CvPoint pt[4];
    for (int i = 0; i<4; i++)
    {
        pt[i].x = (int)point[i].x;
        pt[i].y = (int)point[i].y;

    }

    cv::line(img, pt[0], pt[1], cvScalar(255), 2, 8, 0);
    cv::line(img, pt[1], pt[2], cvScalar(255), 2, 8, 0);
    cv::line(img, pt[2], pt[3], cvScalar(255), 2, 8, 0);
    cv::line(img, pt[3], pt[0], cvScalar(255), 2, 8, 0);

}
/*
 * Function computes the begin point of knife trace which is the intersection of
 * the oblique knife trace and the vertical knife trace
 * @return the pixel coordinate of the begin point.
*/
cv::Point Detector::get_BeginPoint(const cv::Mat& test_image_,bool show_result)
{
    cv::Mat clone_image;
    cv::resize(test_image_,clone_image,cv::Size(640,480));
    this->operate_image=clone_image.clone();
    this->display_image=clone_image.clone();

    obliqueImg=pretreatment(OBLIQUE);
    verticalImg=pretreatment(VERTICAL);

    assert(obliqueImg.empty());
    assert(verticalImg.empty());
    cv::minMaxLoc(obliqueImg, nullptr, nullptr, nullptr, &obliqueMaxLoc);
    cv::minMaxLoc(verticalImg.col(0), nullptr, nullptr, nullptr, &verticalMaxLoc);

    verticalMaxLoc.x=verticalImg.size().height/2-verticalMaxLoc.y+this->operate_image.size().width/2;
    beginPoint=getPointOnline(this->operate_image.size(),obliqueImg.size(),obliqueMaxLoc,verticalMaxLoc.x);

    refineLoc();

    this->borderPoint=getPointOnline(this->operate_image.size(),obliqueImg.size(),obliqueMaxLoc,-1,0);
    if(this->borderPoint.x<0 || this->borderPoint.x>this->operate_image.cols)
        this->borderPoint=getPointOnline(this->operate_image.size(),obliqueImg.size(),obliqueMaxLoc,0,-1);


    if(show_result)
    {
        cv::circle(this->display_image,beginPoint,5,Scalar(255,0,0),-1);
        imshow("detection result",this->display_image);
        waitKey(0);
    }
    return beginPoint;

}
/*
 * Function gets the complete knife Trace (the oblique one)
 * @param test_image_   [the input test image]
 * @return all coordinates of the knife trace
*/
std::vector<cv::Point> Detector::get_knifeTrace(const cv::Mat &test_image_,bool show_result)
{

    if(beginPoint==cv::Point(-1,-1))
        get_BeginPoint(test_image_,false);

    std::vector<cv::Point> knife_trace=get_knife_trace(false);

    if(show_result)
    {
        cv::polylines(this->display_image,knife_trace,false,Scalar(0,255,0),2);
        imshow("detection result",this->display_image);
        waitKey(0);
        cv::destroyAllWindows();
    }
    return knife_trace;
}
/*
 * Function computes all coordinates of the knife trace
 * @debug   [trigger for displaying all intermediate processed image]
 * @return all coordinates of the knife trace
*/
std::vector<cv::Point> Detector::get_knife_trace(bool debug)
{
    cv::Mat mask=Mat::zeros(this->operate_image.size(),CV_8UC1);
    CvBox2D box;
    box.size.width = norm(borderPoint-beginPoint);
    box.size.height = 15;
    box.angle = atan2((beginPoint-borderPoint).y,(beginPoint-borderPoint).x)*180.0/M_PI;
    box.center = (borderPoint+beginPoint)/2;
    DrawBox(box,mask);
    cv::floodFill(mask,box.center,255, nullptr,cvScalarAll(0), cvScalarAll(0), CV_FLOODFILL_FIXED_RANGE);

    cv::Mat newImg,newImgCanny;
    cv::GaussianBlur(this->operate_image,newImgCanny,Size(15, 15), 0, 0, cv::BORDER_DEFAULT);
    cv::cvtColor(newImgCanny,newImgCanny,CV_BGRA2GRAY);
    cv::Canny(newImgCanny,newImgCanny,5,40);
    newImgCanny.copyTo(newImg,mask);
    if(debug)
    {
        imshow("knife trace canny image",newImg);
        waitKey(0);
    }
    cv::blur(newImg,newImg,cv::Size(3,3));
    if(debug)
    {
        imshow("knife trace mean blur image",newImg);
        waitKey(0);
    }
    cv::threshold(newImg, newImg, 30,255 , CV_THRESH_BINARY);
    if(debug)
    {
        imshow("knife trace threshold image",newImg);
        waitKey(0);
    }
    Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
    morphologyEx(newImg, newImg, MORPH_GRADIENT, element);
    if(debug)
    {
        imshow("knife trace gradient image",newImg);
        waitKey(0);
    }
    vector<vector<Point>> contours;
    vector<Vec4i> hierarcy;
    findContours(newImg, contours, hierarcy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    double maxArea = 0;
    vector<cv::Point> maxContour;

    for(const auto & contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area > maxArea)
        {
            maxArea = area;
            maxContour = contour;
        }
    }
    if(debug)
    {
        std::vector<std::vector<cv::Point>> maxContours;
        maxContours.push_back(maxContour);
        cv::Mat contoursDisplayImg=Mat::zeros(this->operate_image.size(),CV_8UC1);
        cv::drawContours(contoursDisplayImg,maxContours,0,Scalar(255,255,255));
        imshow("knife trace contours image",contoursDisplayImg);
        waitKey(0);
    }
    std::vector<cv::Point> knife_trace;
    knife_trace=maxContour;
    knife_trace.resize(knife_trace.size()/2);

    return knife_trace;
}