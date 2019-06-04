#include <Eigen/Core>
#include "Servo.h"
#include <unistd.h>
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
#include  <math.h>
#include "visual_servo/TagDetection_msg.h"
#include "visual_servo/TagsDetection_msg.h"
#include "Detector.h"
#include "visual_servo/detect_once.h"

class RealSense
{
private:
    ros::NodeHandle n_;
    image_transport::ImageTransport *it_;
    image_transport::Subscriber image_sub_,depth_sub_;
    ros::Publisher tag_pub_;
    ros::ServiceServer service_;
    double fx,fy,u0,v0;
    Servo *realSense_;
    double tag_size_;
    Detector *detector_;
    cv_bridge::CvImagePtr cv_ptr_,depth_ptr_;
    cv::Mat subscribed_rgb_,subscribed_depth_;

    std::vector<TagDetectInfo> Tags_detected_;
    std::vector<Eigen::Vector3d> coordinate_3d_;
    cv::Point beginPoint_;
    Eigen::Vector3d inverse_project(Eigen::Vector3d TargetPoint);
    bool detect_once(visual_servo::detect_once::Request  &req,
                    visual_servo::detect_once::Response &res);
public:
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
    this->fx=615.3072;
    this->fy=616.1456;
    this->u0=333.4404;
    this->v0=236.2650;
    this->image_received= false;
    this->tag_size_= 0.10;
    detect_once_callback_t detect_once_callback =
            boost::bind(&RealSense::detect_once, this, _1, _2);
    it_ = new image_transport::ImageTransport(n_);
    realSense_ = new Servo;
    detector_ = new Detector;
    image_sub_ = it_->subscribe("/camera/color/image_raw", 1,&RealSense::ImageCallback,this);
    depth_sub_ = it_->subscribe("/camera/aligned_depth_to_color/image_raw", 1,&RealSense::DepthCallback,this);
    tag_pub_ = n_.advertise<visual_servo::TagsDetection_msg>("TagsDetected",1000);
    service_ =n_.advertiseService("detect_once",detect_once_callback);

}
RealSense::~RealSense()
{
    delete it_;
    delete realSense_;
    delete detector_;
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
    realSense_->SetCameraParameter(fx,fy,u0,v0);
    Tags_detected_ = realSense_->GetTargetPoseMatrix(subscribed_rgb_,tag_size_);
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
    coordinate_3d_.clear();
    if(!Tags_detected_.empty())
    {
        for(auto & tag_detected:Tags_detected_)
        {
            Eigen::Vector3d TargetPoint;
            TargetPoint[0]=beginPoint_.x;
            TargetPoint[1]=beginPoint_.y;
            TargetPoint[2]=1;
            coordinate_3d_.push_back(inverse_project(TargetPoint));
            //std::cout<<TargetPoint<<std::endl;
        }

    }

}
void RealSense::TagsInfoPublish()
{
    visual_servo::TagsDetection_msg TagsDetection;
    visual_servo::TagDetection_msg TagDetection;
    if(!Tags_detected_.empty()&&!coordinate_3d_.empty())
    {
        //std::cout<<Tags_detected_[0].Trans_C2T.matrix()<<std::endl;

        TagsDetection.header.frame_id="camera_color_optical_frame";
        for(int i=0;i<Tags_detected_.size();++i)
        {
            TagDetection.id = Tags_detected_[i].id;
            TagDetection.pose = Eigen::toMsg(Tags_detected_[i].Trans_C2T);
            TagDetection.PixelCoef = Tags_detected_[i].PixelCoef;
            TagDetection.center.x = Tags_detected_[i].Center.x;
            TagDetection.center.y = Tags_detected_[i].Center.y;
            TagDetection.Center3d.x = coordinate_3d_[i][0];
            TagDetection.Center3d.y = coordinate_3d_[i][1];
            TagDetection.Center3d.z = coordinate_3d_[i][2];
            TagsDetection.tags_information.push_back(TagDetection);
        }
    }
    tag_pub_.publish(TagsDetection);
}
Eigen::Vector3d RealSense::inverse_project(Eigen::Vector3d TargetPoint)
{
    Eigen::Vector3d coordinate_3d;
    Eigen::Matrix3d intrinsic_matrix;
    intrinsic_matrix<<  this->fx,0.0,this->u0,
                        0.0,this->fy,this->v0,
                        0.0,0.0,1.0;

    coordinate_3d = intrinsic_matrix.inverse()*TargetPoint;

    coordinate_3d[2]=0;
    int counter=0;
    for (int v = TargetPoint[1]-2; v <= TargetPoint[1]+2; ++v)
    {
        for (int u = TargetPoint[0]-2; u <= TargetPoint[0]+2; ++u)
        {
            counter++;
            coordinate_3d[2] += subscribed_depth_.at<float>(v, u)*0.001f;
        }
    }
    coordinate_3d[2] /=25.0;
    coordinate_3d[0] *=coordinate_3d[2];
    coordinate_3d[1] *=coordinate_3d[2];
    return coordinate_3d;
}
bool RealSense::detect_once(visual_servo::detect_once::Request  &req,
                          visual_servo::detect_once::Response &res)
{
    beginPoint_ = detector_->get_BeginPoint(subscribed_rgb_);
    std::cout<<beginPoint_.x<<beginPoint_.y<<std::endl;
    Eigen::Vector3d TargetPoint,RealPoint;
    TargetPoint[0]=beginPoint_.x;
    TargetPoint[1]=beginPoint_.y;
    TargetPoint[2]=1;
    RealPoint=inverse_project(TargetPoint);
    res.beginPoint.x=RealPoint[0];
    res.beginPoint.y=RealPoint[1];
    res.beginPoint.z=RealPoint[2];
}

int main(int argc, char** argv)
{
   ros::init(argc,argv,"real_sense");
   RealSense real_sense;
   ros::Rate loop_rate(30);
   int counter;

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

