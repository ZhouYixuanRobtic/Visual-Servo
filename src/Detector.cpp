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
 * Function processes the private variable test_image in defined process type
 * @param processType   [process type, OBLIQUE for oblique knife image, VERTICAL for vertical knife image]
 * note: the private variable test_image is modified in this function
*/
cv::Mat Detector::pretreatment(int processType)
{
    double minVal, maxVal;
    cv::Mat gaussianImg,grayImg,grayImgRadon;

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

    radon::radonTransform(grayImg,grayImgRadon,radonAngleRange,radonOperation);

    cv::minMaxLoc(grayImgRadon, &minVal, &maxVal);
    grayImgRadon -= minVal;
    grayImgRadon.convertTo(grayImgRadon, CV_8UC1, 255.0/(maxVal-minVal) );

    return grayImgRadon;
}
/*
 * Function gets the intersection of
 * the oblique line and the vertical line
 * @return the pixel coordinate of the intersection
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
void Detector::refineLoc()
{
    cv::Mat gaussianImg,grayImg;
    cv::GaussianBlur(this->test_image,gaussianImg,Size(15, 15), 0, 0, cv::BORDER_DEFAULT);
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
 * Function computes the begin point of knife trace which is the intersection of
 * the oblique knife trace and the vertical knife trace
 * @return the pixel coordinate of the begin point.
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
cv::Point Detector::get_BeginPoint(const cv::Mat& test_image_)
{
    this->test_image=test_image_.clone();
    this->display_image=test_image_.clone();
    obliqueImg=pretreatment(OBLIQUE);
    verticalImg=pretreatment(VERTICAL);

    cv::minMaxLoc(obliqueImg, nullptr, nullptr, nullptr, &obliqueMaxLoc);
    cv::minMaxLoc(verticalImg.col(0), nullptr, nullptr, nullptr, &verticalMaxLoc);
    verticalMaxLoc.x=verticalImg.size().height/2-verticalMaxLoc.y+this->test_image.size().width/2;
    beginPoint=getPointOnline(this->test_image.size(),obliqueImg.size(),obliqueMaxLoc,verticalMaxLoc.x);
    refineLoc();
     this->borderPoint=getPointOnline(this->test_image.size(),obliqueImg.size(),obliqueMaxLoc,-1,0);
    if(this->borderPoint.x<0)
        this->borderPoint=getPointOnline(this->test_image.size(),obliqueImg.size(),obliqueMaxLoc,0,-1);
    cv::circle(this->display_image,beginPoint,5,Scalar(255,0,0),-1);
    return beginPoint;

}
std::vector<cv::Point> Detector::get_knifeTrace(const cv::Mat &test_image_)
{

    if(beginPoint==cv::Point(-1,-1))
        get_BeginPoint(test_image_);

    std::vector<cv::Point> knife_trace=get_knife_trace();
    cv::polylines(this->display_image,knife_trace,false,Scalar(0,255,0),2);
    imshow("newsddf",this->display_image);
    waitKey(0);
    return knife_trace;

}
std::vector<cv::Point> Detector::trace_segement()
{
    std::vector<cv::Point> knife_trace;
    cv::Mat gaussianImg,grayImg,grayImgCanny;
    cv::GaussianBlur(this->test_image,gaussianImg,Size(15, 15), 0, 0, cv::BORDER_DEFAULT);
    cv::cvtColor(gaussianImg,grayImg,CV_BGRA2GRAY);
    cv::Canny(grayImg,grayImgCanny,5,40,3);
    cv::Point init_cutPoint = beginPoint;
    cv::Point init_cutPoint_static = beginPoint / 4;
    int i = 1;
    while (i < 4)
    {
        cv::Rect rect(init_cutPoint.x - init_cutPoint_static.x, init_cutPoint.y - init_cutPoint_static.y, init_cutPoint_static.x, init_cutPoint_static.y);
        cv::rectangle(this->display_image,rect,Scalar(255,255,255));
        cv::Mat refineImg = grayImgCanny(rect).clone();
        imshow("sdfssdf",refineImg);
        waitKey(0);
        cv::Mat refineImgRadon;
        radon::radonTransform(refineImg,refineImgRadon,radonAngleRange,radonOperation);

        imshow("sdfs",refineImgRadon);
        waitKey(0);
        cv::Point maxLoc;
        cv::Mat mask=Mat::zeros(refineImgRadon.size(),CV_8UC1);
        mask(cv::Rect(cv::Point(20,0),cv::Size(130,refineImgRadon.size().height))).setTo(255);

        cv::minMaxLoc(refineImgRadon,nullptr, nullptr,nullptr,&maxLoc,mask);
        cv::Point borderPoint=getPointOnline(refineImg.size(),refineImgRadon.size(),maxLoc,-1,0);
        if(borderPoint.x<0)
            borderPoint=getPointOnline(refineImg.size(),refineImgRadon.size(),maxLoc,0,-1);

        init_cutPoint=cv::Point(init_cutPoint.x - init_cutPoint_static.x, init_cutPoint.y - (init_cutPoint_static.y))
                      +borderPoint;
        knife_trace.push_back(init_cutPoint);
        cout << "init_cutPoint: " << init_cutPoint.x << ", " << init_cutPoint.y  << endl;
        circle(this->display_image, init_cutPoint, 5, Scalar(0, 0, 255), -1);
        i++;

    }

    return knife_trace;
}
std::vector<cv::Point> Detector::get_knife_trace()
{
    cv::Mat mask=Mat::zeros(this->test_image.size(),CV_8UC1);
    CvBox2D box;
    box.size.width = norm(borderPoint-beginPoint);
    box.size.height = 10;
    box.angle = atan2((beginPoint-borderPoint).y,(beginPoint-borderPoint).x)*180.0/M_PI;
    box.center = (borderPoint+beginPoint)/2;
    DrawBox(box,mask);
    cv::floodFill(mask,box.center,255,NULL,cvScalarAll(0), cvScalarAll(0), CV_FLOODFILL_FIXED_RANGE);
    cv::Mat newImg,newImgCanny;
    cv::GaussianBlur(this->test_image,newImgCanny,Size(15, 15), 0, 0, cv::BORDER_DEFAULT);
    cv::cvtColor(newImgCanny,newImgCanny,CV_BGRA2GRAY);
    cv::Canny(newImgCanny,newImgCanny,5,40);
    newImgCanny.copyTo(newImg,mask);

    cv::blur(newImg,newImg,cv::Size(3,3));
    cv::threshold(newImg, newImg, 30,255 , CV_THRESH_BINARY);

    Mat element = getStructuringElement(MORPH_RECT, Size(2, 2));
    morphologyEx(newImg, newImg, MORPH_GRADIENT, element);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarcy;
    findContours(newImg, contours, hierarcy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
    double maxArea = 0;
    vector<vector<cv::Point>> maxContours;
    maxContours.resize(1);
    for(const auto & contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area > maxArea)
        {
            maxArea = area;
            maxContours[0] = contour;

        }
    }
    std::vector<cv::Point> knife_trace;
    knife_trace=maxContours[0];
    knife_trace.resize(knife_trace.size()/2);


    return knife_trace;
}