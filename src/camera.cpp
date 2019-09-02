#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseStamped.h>

#include <tf2_eigen/tf2_eigen.h>
#include "ros/callback_queue.h"


#include "Detector.h"
#include "VisualServoMetaType.h"

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

class RealSense
{
private:
    TagDetectInfo TagDetected_;

    //camera intrinsic parameter
    const double fx=615.3072,fy=616.1456,u0=333.4404,v0=236.2650;
    //tag family name
    const char TagFamilyName[20]="tag36h11";
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

    //april tag physic size unit meter
    double tag_size_ ;
    //triggers
    bool traceResultOn_,traceDebugOn_,tagGraphOn_,colorOn_,tagDetectorOn_;
    std::string figDir_;

    ros::NodeHandle n_;
    image_transport::ImageTransport *it_;
    image_transport::Subscriber image_sub_,depth_sub_;
    ros::Publisher tag_pub_;
    ros::ServiceServer service_;

    //Detector class for detecting knife trace and its begin point
    Detector *detector_;

    cv_bridge::CvImagePtr cv_ptr_,depth_ptr_;
    cv::Mat subscribed_rgb_,subscribed_depth_;

    //Tags information detected from camera, vector is empty when no tags are detected
    tag_detection_info_t tags_detected_;
    /*
     * Function computes tag information of all tags detected
     * @param UserImage [the image prepared to detect tag]
     * fill a vector contains all tag information detected, empty when no tags detected
     */
    void GetTargetPoseMatrix(Mat UserImage);

    /* Function computing the real 3-dimensional position of a pixel point in image with respect to camera
     * @param TargetPoint   [Homogeneous coordinate of pixel point]
     * @return real 3-dimensional coordinate in camera coordinate system
     */
    geometry_msgs::Point inverse_project(Eigen::Vector3d TargetPoint);
    /* Function computing the real 3-dimensional position of knife trace begin point
     * and fill service response
     */
    bool detect_once(visual_servo::detect_once::Request  &req,
                    visual_servo::detect_once::Response &res);
                    
    void initParameter();




public:
    //type definition of detect_once service call back function handle
    typedef boost::function<bool (visual_servo::detect_once::Request&,visual_servo::detect_once::Response& res)>
            detect_once_callback_t;
    bool image_received,depth_received;
    RealSense();
    virtual ~RealSense();
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void DepthCallback(const sensor_msgs::ImageConstPtr& msg);
    void TagsInfoPublish();

};
RealSense::RealSense()
{
    tf = tag36h11_create();
    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    this->image_received= false;
    detect_once_callback_t detect_once_callback =
            boost::bind(&RealSense::detect_once, this, _1, _2);
    it_ = new image_transport::ImageTransport(n_);
    detector_ = new Detector(traceResultOn_,traceDebugOn_,colorOn_);
    image_sub_ = it_->subscribe("/camera/color/image_raw", 100,&RealSense::ImageCallback,this);
    depth_sub_ = it_->subscribe("/camera/aligned_depth_to_color/image_raw", 100,&RealSense::DepthCallback,this);
    tag_pub_ = n_.advertise<visual_servo::TagsDetection_msg>("TagsDetected",100);
    service_ =n_.advertiseService("detect_once",detect_once_callback);

    initParameter();
}
RealSense::~RealSense()
{
    delete it_;
    delete detector_;
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
}
void RealSense::initParameter()
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
    info.fx = fx;
    info.fy = fy;
    info.cx = u0;
    info.cy = v0;
}
void RealSense::GetTargetPoseMatrix(Mat UserImage)
{
    cv::Mat gray;
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
}
void RealSense::ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    this->image_received=true;
    try
    {
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    subscribed_rgb_=cv_ptr_->image;
    n_.param<bool>("/visual/tagDetectorOn",tagDetectorOn_,false);
    if(tagDetectorOn_)
        GetTargetPoseMatrix(subscribed_rgb_);
    else
    {
        if(colorOn_)
        {
                cv::imshow("Tag Detections",subscribed_rgb_);
                cv::waitKey(3);
        }

    }

}
void RealSense::DepthCallback(const sensor_msgs::ImageConstPtr &msg)
{
    this->depth_received=true;
    try
    {
        depth_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
    }

    subscribed_depth_ = depth_ptr_->image;

}
void RealSense::TagsInfoPublish()
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
geometry_msgs::Point RealSense::inverse_project(Eigen::Vector3d TargetPoint)
{
    Eigen::Vector3d coordinate_3d_;
    Eigen::Matrix3d intrinsic_matrix;
    intrinsic_matrix<<  this->fx,0.0,this->u0,
                        0.0,this->fy,this->v0,
                        0.0,0.0,1.0;

    coordinate_3d_ = intrinsic_matrix.inverse()*TargetPoint;

    coordinate_3d_[2]=0;
    int counter=0;
    for (int v = TargetPoint[1]-2; v <= TargetPoint[1]+2; ++v)
    {
        for (int u = TargetPoint[0]-2; u <= TargetPoint[0]+2; ++u)
        {
            counter++;
            coordinate_3d_[2] += subscribed_depth_.at<float>(v, u)*0.001f;
        }
    }
    coordinate_3d_[2] /=25.0;
    coordinate_3d_[0] *=coordinate_3d_[2];
    coordinate_3d_[1] *=coordinate_3d_[2];
    geometry_msgs::Point coordinate_3d;
    coordinate_3d.x=coordinate_3d_[0];
    coordinate_3d.y=coordinate_3d_[1];
    coordinate_3d.z=coordinate_3d_[2];
    return coordinate_3d;
}
bool RealSense::detect_once(visual_servo::detect_once::Request  &req,
                          visual_servo::detect_once::Response &res)
{
    cv::Point beginPoint;
    std::vector<cv::Point> knife_trace=Detector::get_traceSegments(detector_->get_knifeTrace(subscribed_rgb_));
    if(knife_trace.empty())
        return false;
    beginPoint = detector_->beginPoint;
    geometry_msgs::Point RealPoint;
    RealPoint=inverse_project(Eigen::Vector3d(beginPoint.x,beginPoint.y,1));
    res.knife_trace.push_back(RealPoint);
    std::cout<<beginPoint.x<<beginPoint.y<<std::endl;
    for(auto & point :knife_trace)
    {
        RealPoint=inverse_project(Eigen::Vector3d(point.x,point.y,1));
        res.knife_trace.push_back(RealPoint);
    }
    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc,argv,"real_sense");
   RealSense real_sense;
   ros::Rate loop_rate(30);
   int counter=0;

   while(ros::ok())
   {
       counter++;
       counter %=31;
       if(counter>=30&&!real_sense.image_received)
       {
           ROS_ERROR("No image received!!!!");
           //break;
       }
       real_sense.TagsInfoPublish();
       ros::spinOnce();
       loop_rate.sleep();
   }
    ros::shutdown();
}

