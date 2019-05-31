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


class RealSense
{
private:
    ros::NodeHandle n_;
    image_transport::ImageTransport *it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher tag_pub_;
    double fx,fy,u0,v0;
    Servo *realSense_;
    double tag_size_;
    cv_bridge::CvImagePtr cv_ptr_;
    cv::Mat subscribed_rgb_;

    std::vector<TagDetectInfo> Tags_detected_;

public:
    bool image_received;
    RealSense();
    virtual ~RealSense();
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
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

    it_ = new image_transport::ImageTransport(n_);
    realSense_ = new Servo;
    image_sub_ = it_->subscribe("/camera/color/image_raw", 1,&RealSense::ImageCallback,this);
    tag_pub_ = n_.advertise<visual_servo::TagsDetection_msg>("TagsDetected",1000);

}
RealSense::~RealSense()
{
    delete it_;
    delete realSense_;
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
void RealSense::TagsInfoPublish()
{
    visual_servo::TagsDetection_msg TagsDetection;
    visual_servo::TagDetection_msg TagDetection;
    if(!Tags_detected_.empty())
    {
        //std::cout<<Tags_detected_[0].Trans_C2T.matrix()<<std::endl;

        TagsDetection.header.frame_id="camera_color_optical_frame";
        for(auto & tag_detected : Tags_detected_ )
        {
            TagDetection.id = tag_detected.id;
            TagDetection.pose = Eigen::toMsg(tag_detected.Trans_C2T);
            TagDetection.PixelCoef = tag_detected.PixelCoef;
            TagDetection.center.x = tag_detected.Center.x;
            TagDetection.center.y = tag_detected.Center.y;
            TagsDetection.tags_information.push_back(TagDetection);
        }
    }
    tag_pub_.publish(TagsDetection);
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

