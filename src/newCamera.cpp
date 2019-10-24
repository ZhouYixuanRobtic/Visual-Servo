#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf2_eigen/tf2_eigen.h>
#include "ros/callback_queue.h"

#include "USBCamera.h"
#include "Detector.h"
#include <dirent.h>

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
#include "apriltag_pose.h"
}

class Monocular
{
private:
    TagDetectInfo TagDetected_;

    USB_CAMERA::USBCamera * usbCamera_;

    //tag family name
    const char TagFamilyName[20]="tag36h11";

    const std::string CALIBRATE_PATH_;
    //april tag handle
    apriltag_family_t *tf;
    //apriltag detecter handle
    apriltag_detector_t *td;
    //apriltag detection handle
    apriltag_detection_t *det;
    //apriltag detection info handle;
    apriltag_detection_info_t info;
    //apriltag pose handle
    apriltag_pose_t pose;

    cv::Mat subscribed_rgb_{};
    //april tag physic size unit meter
    double tag_size_ ;
    //triggers
    bool traceResultOn_,traceDebugOn_,tagGraphOn_,colorOn_,tagDetectorOn_;

    ros::NodeHandle n_;

    ros::Publisher tag_pub_;

    /*
     * Function computes tag information of all tags detected
     * @param UserImage [the image prepared to detect tag]
     * fill a vector contains all tag information detected, empty when no tags detected
     */
    tag_detection_info_t & GetTargetPoseMatrix(cv::Mat & UserImage);

    void initParameter();

public:
    explicit Monocular(std::string  calibrate_path);
    ~Monocular();
    void detect_worker();
    void TagsInfoPublish(tag_detection_info_t & tags_detected_);
    void run();

};
Monocular::Monocular(std::string calibrate_path): CALIBRATE_PATH_(std::move(calibrate_path))
{
    tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    usbCamera_ = new USB_CAMERA::USBCamera(CALIBRATE_PATH_);

    tag_pub_ = n_.advertise<visual_servo::TagsDetection_msg>("TagsDetected",100);

    initParameter();
}
Monocular::~Monocular()
{
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    delete usbCamera_;
}
void Monocular::initParameter()
{
    n_.param<double>("/visual/tagSize",tag_size_,0.10);
    n_.param<bool>("/visual/tagGraphOn",tagGraphOn_,true);
    n_.param<bool>("/visual/traceResultOn",traceResultOn_,false);
    n_.param<bool>("/visual/traceDebugOn",traceDebugOn_,false);
    n_.param<bool>("/visual/colorOn",colorOn_,false);
    n_.param<bool>("/visual/tagDetectorOn",tagDetectorOn_,false);
    std::cout<<"read parameter accomplished"<<std::endl;
    td->quad_decimate = 2.0;
    td->quad_sigma = 0.0;
    td->nthreads = 2;
    td->debug = false;
    td->refine_edges = true;

    info.tagsize = tag_size_;
    usbCamera_->getIntrinsicParameter(info.fx,info.fy,info.cx,info.cy);
}
tag_detection_info_t & Monocular::GetTargetPoseMatrix(cv::Mat & UserImage)
{
    cv::Mat gray;
    static tag_detection_info_t tags_detected_;
    tags_detected_.clear();
    cvtColor(UserImage, gray, COLOR_BGR2GRAY);
    // Make an image_u8_t header for the Mat data
    image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
    };
    zarray_t *detections = apriltag_detector_detect(td, &im);
    for (int i = 0; i < zarray_size(detections); i++)
    {
        zarray_get(detections, i, &det);
        line(UserImage, Point(det->p[0][0], det->p[0][1]),
             Point(det->p[1][0], det->p[1][1]),
             Scalar(0, 0xff, 0), 2);
        line(UserImage, Point(det->p[0][0], det->p[0][1]),
             Point(det->p[3][0], det->p[3][1]),
             Scalar(0, 0, 0xff), 2);
        line(UserImage, Point(det->p[1][0], det->p[1][1]),
             Point(det->p[2][0], det->p[2][1]),
             Scalar(0xff, 0, 0), 2);
        line(UserImage, Point(det->p[2][0], det->p[2][1]),
             Point(det->p[3][0], det->p[3][1]),
             Scalar(0xff, 0, 0), 2);

        std::stringstream ss;
        ss << det->id;
        String text = ss.str();
        int baseline;
        Size textsize = getTextSize(text, FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, 2,
                                    &baseline);
        putText(UserImage, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, Scalar(0xff, 0x99, 0), 2);
        //pose_estimation
        info.det = det;
        double err = estimate_tag_pose(&info, &pose);
        TagDetected_.Trans_C2T.matrix() << pose.R->data[0],  pose.R->data[1],  pose.R->data[2],  pose.t->data[0],
                pose.R->data[3],  pose.R->data[4],  pose.R->data[5],  pose.t->data[1],
                pose.R->data[6],  pose.R->data[7],  pose.R->data[8],  pose.t->data[2],
                0.0,  0.0,  0.0,  1.0;
        TagDetected_.id=det->id;
        TagDetected_.Center=Point2d(det->c[0],det->c[1]);
        TagDetected_.PixelCoef=0.0;
        TagDetected_.PixelCoef+=tag_size_/norm(Point(det->p[0][0]-det->p[1][0],det->p[0][1]-det->p[1][1]));
        TagDetected_.PixelCoef+=tag_size_/norm(Point(det->p[0][0]-det->p[3][0],det->p[0][1]-det->p[3][1]));
        TagDetected_.PixelCoef+=tag_size_/norm(Point(det->p[1][0]-det->p[2][0],det->p[1][1]-det->p[2][1]));
        TagDetected_.PixelCoef+=tag_size_/norm(Point(det->p[2][0]-det->p[3][0],det->p[2][1]-det->p[3][1]));
        TagDetected_.PixelCoef+=sqrt(2)*tag_size_/norm(Point(det->p[0][0]-det->p[2][0],det->p[0][1]-det->p[2][1]));
        TagDetected_.PixelCoef+=sqrt(2)*tag_size_/norm(Point(det->p[1][0]-det->p[3][0],det->p[1][1]-det->p[3][1]));
        TagDetected_.PixelCoef=TagDetected_.PixelCoef/6;
        tags_detected_.push_back(TagDetected_);
    }
    zarray_destroy(detections);
    if(tagGraphOn_)
    {
        imshow("Tag Detections", UserImage);
        waitKey(3);
    }
    return  tags_detected_;
}

void Monocular::TagsInfoPublish(tag_detection_info_t & tags_detected_)
{
    visual_servo::TagsDetection_msg TagsDetection;
    visual_servo::TagDetection_msg TagDetection;
    if(!tags_detected_.empty())
    {
        //std::cout<<tags_detected_[0].Trans_C2T.matrix()<<std::endl;

        TagsDetection.header.frame_id="camera_color_optical_frame";
        for(int i=0;i<tags_detected_.size();++i)
        {
            TagDetection.id = tags_detected_[i].id;
            TagDetection.pose = tf2::toMsg(tags_detected_[i].Trans_C2T);
            TagDetection.PixelCoef = tags_detected_[i].PixelCoef;
            TagDetection.center.x = tags_detected_[i].Center.x;
            TagDetection.center.y = tags_detected_[i].Center.y;
            TagsDetection.tags_information.push_back(TagDetection);
        }
    }
    tag_pub_.publish(TagsDetection);
}
void Monocular::detect_worker()
{
    n_.param<bool>("/visual/tagDetectorOn",tagDetectorOn_,false);
    if(usbCamera_->isReady)
    {
        subscribed_rgb_ = usbCamera_->getUndistortedGroup().frame;
        if(tagDetectorOn_)
            TagsInfoPublish(GetTargetPoseMatrix(subscribed_rgb_));
        else
        {
            if(colorOn_)
            {
                cv::imshow("Tag Detections",subscribed_rgb_);
                cv::waitKey(3);
            }
        }
    }

}
int main(int argc, char* argv[])
{
    ros::init(argc,argv,"monocular");
    Monocular real_sense("/home/xcy/WorkSpace/src/Visual-Servo/config/calibrate.yml");
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        real_sense.detect_worker();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



