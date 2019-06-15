//
// Created by xcy on 19-6-4.
//
#include "Detector.h"

int main(int argc, const char ** argv)
{
    cv::Mat test_image = cv::imread("/home/xcy/test/test2.jpg");
    if(test_image.empty())
    {
        cout<<"unable to load image"<<endl;
        return -1;
    }

    int radonAngleRange=63,radonOperation=radon::RT_SUM;
    Detector knife_detector;
    knife_detector.radonAngleRange=radonAngleRange;
    knife_detector.radonOperation=radonOperation;

    std::vector<cv::Point> knife_trace=knife_detector.get_knifeTrace(test_image);
    cout<<"result x:"<<knife_detector.beginPoint.x<<"result y:"<<knife_detector.beginPoint.y<<endl;
    return 0;
}
