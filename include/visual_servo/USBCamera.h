#ifndef VISUAL_SERVO_USBCAMERA_H
#define VISUAL_SERVO_USBCAMERA_H

#include "VisualServoMetaType.h"
#include <boost/thread/thread.hpp>
#include "tr1/memory"
#include <boost/thread/shared_mutex.hpp>
#include <atomic>

namespace USB_CAMERA
{
    struct UndistortedGroup
    {
        cv::Mat frame;
        cv::Mat intrinsic_matrix;
    };
    class USBCamera{
    private:
        const std::string INTRINSIC_PATH_;

        cv::VideoCapture *capture_;

        cv::Mat intrinsic_matrix_;
        cv::Mat distortion_coefficients_;
        cv::Size resolution_{};
        double FPS_{};

        double fx_,fy_,cx_,cy_;
        cv::Mat frame_{};
        UndistortedGroup undistortedGroup_{};

        boost::shared_mutex frame_mutex_{};
        std::tr1::shared_ptr<boost::thread> video_thread_ptr_{};

        bool isStreamingStopped_{};
        void readIntrinsic();
        void video_worker();

    public:
        explicit USBCamera(std::string intrinsic_path);
        ~USBCamera();

        bool isOpened{};
        bool isReady{};
        void setParameter(cv::Size & resolution,int fps);

        double getFPS() const {return FPS_;};
        const cv::Size & getResolution() const {return resolution_;};
        Eigen::Matrix3d getIntrinsicMatrix() const;
        void getIntrinsicParameter(double & fx, double & fy, double & cx, double & cy);

        const cv::Mat & getPicture() const { return frame_;};
        UndistortedGroup & getUndistortedGroup();

        bool stopStreaming();
        bool restartStreaming();
    };
}


#endif //VISUAL_SERVO_USBCAMERA_H
