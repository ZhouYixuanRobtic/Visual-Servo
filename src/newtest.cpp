//
// Created by xcy on 19-6-4.
//
//To Do
// overexposure image has no radon transform
#include "Detector.h"
#include <ctime>

int main(int argc, const char ** argv)
{
    cv::Mat test_image = cv::imread("/home/xcy/test/test4.jpg");
    if(test_image.empty())
    {
        cout<<"unable to load image"<<endl;
        return -1;
    }

    int radonAngleRange=63,radonOperation=radon::RT_SUM;
    Detector knife_detector;
    knife_detector.radonAngleRange=radonAngleRange;
    knife_detector.radonOperation=radonOperation;
    std::clock_t c_start = std::clock();
    std::vector<cv::Point> knife_trace=knife_detector.get_knifeTrace(test_image,true);
    std::clock_t c_end = std::clock();
    long double time_elapsed_ms =1000.0*(c_end-c_start) / CLOCKS_PER_SEC;
    cout<<"result x:"<<knife_detector.beginPoint.x<<"result y:"<<knife_detector.beginPoint.y<<endl;
    std::cout << "CPU time used: " << time_elapsed_ms << " ms\n";
    return 0;
}
