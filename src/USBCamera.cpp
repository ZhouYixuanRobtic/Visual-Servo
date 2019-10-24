#include "USBCamera.h"
using namespace USB_CAMERA;
USBCamera::USBCamera(std::string intrinsic_path) : INTRINSIC_PATH_(std::move(intrinsic_path))
{
    capture_ = new cv::VideoCapture(0);
    isOpened = capture_->isOpened();
    fx_=fy_=cx_=cy_=0.0;
    readIntrinsic();
    if(isOpened)
    {
        FPS_= capture_->get(CV_CAP_PROP_FPS);
        resolution_ =cv::Size(capture_->get(CV_CAP_PROP_FRAME_WIDTH),capture_->get(CV_CAP_PROP_FRAME_HEIGHT));
        video_thread_ptr_.reset(new boost::thread(boost::bind(&USBCamera::video_worker,this)));
    }
    else
    {
        std::cout<<"No camera detected, is the camera connected to the computer!"<<std::endl;
    }

}
USBCamera::~USBCamera()
{
    if(!isStreamingStopped_)
    {
        video_thread_ptr_->interrupt();
        video_thread_ptr_->join();
    }
    capture_->release();
    delete capture_;
}

void USBCamera::readIntrinsic()
{
    cv::FileStorage read_fs(INTRINSIC_PATH_, cv::FileStorage::READ);
    if (!read_fs.isOpened())
        std::cout << "Cannot open the file" << std::endl;
    else
    {
        read_fs["cameraMatrix"] >> intrinsic_matrix_;
        read_fs["distCoeffs"] >> distortion_coefficients_;
        read_fs.release();
        fx_ =intrinsic_matrix_.at<double>(0,0);
        fy_ =intrinsic_matrix_.at<double>(1,1);
        cx_ =intrinsic_matrix_.at<double>(0,2);
        cy_ =intrinsic_matrix_.at<double>(1,2);
    }
}

void USBCamera::video_worker()
{
    try
    {
        boost::this_thread::interruption_enabled();
        isStreamingStopped_=false;
        while(true)
        {
            boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();
            boost::this_thread::interruption_point();
            if(isOpened)
            {
                write_mutex_.lock();
                *capture_>> frame_;
                isReady=true;
                write_mutex_.unlock();
            }
            boost::posix_time::ptime endTime = boost::posix_time::microsec_clock::local_time();
            boost::this_thread::sleep(boost::posix_time::microseconds((int)1E6/FPS_ - (endTime - startTime).total_microseconds()));
        }
    }
    catch (boost::thread_interrupted&e )
    {
        std::cout<<"the camera video quit now"<<std::endl;
        isStreamingStopped_=true;
    }
}

void USBCamera::setParameter(cv::Size& resolution, int fps)
{
    resolution_ = resolution;
    FPS_ = fps;
    if(isOpened)
    {
        capture_->set(CV_CAP_PROP_FPS,FPS_);
        capture_->set(CV_CAP_PROP_FRAME_WIDTH,resolution_.width);
        capture_->set(CV_CAP_PROP_FRAME_HEIGHT,resolution_.height);
    }
}
Eigen::Matrix3d USBCamera::getIntrinsicMatrix() const
{
    Eigen::Matrix3d intrinsic_matrix;
    cv::cv2eigen(intrinsic_matrix_,intrinsic_matrix);
    return intrinsic_matrix;
}
void USBCamera::getIntrinsicParameter(double &fx, double &fy, double &cx, double & cy)
{
    fx=fx_;
    fy=fy_;
    cx=cx_;
    cy=cy_;
}
UndistortedGroup& USBCamera::getUndistortedGroup()
{
    undistortedGroup_.intrinsic_matrix=cv::getOptimalNewCameraMatrix(intrinsic_matrix_,distortion_coefficients_,resolution_,1,resolution_);
    cv::undistort(frame_,undistortedGroup_.frame,intrinsic_matrix_,distortion_coefficients_);
    return undistortedGroup_;
}
bool USBCamera::stopStreaming()
{
    video_thread_ptr_->interrupt();
    video_thread_ptr_->join();
}
bool USBCamera::restartStreaming()
{
    video_thread_ptr_.reset(new boost::thread(boost::bind(&USBCamera::video_worker,this)));
}