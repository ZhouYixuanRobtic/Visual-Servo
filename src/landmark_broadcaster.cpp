/*
 * This is a landmark publisher node required by cartographer
 * thus when you want to compile this node, please ensure that you've installed the cartographer
 * and add its include path into CMakeLists.txt and add cartographer_ros into package.xml.
 * A example：
 *  CMakeLists.txt:
 *      set(CARTOGRAPHER_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../catkin_cartographer_ws/install_isolated/include/)
 *      include_directories (${CARTOGRAPHER_INCLUDE_DIR})
 *  Package.xml:
 *      <build_depend>cartographer_ros</build_depend>
 *      <build_export_depend>cartographer_ros</build_export_depend>
 *      <exec_depend>cartographer_ros</exec_depend>
 */

#include "Servo.h"

#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "ros/callback_queue.h"
#include <ros/package.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <cartographer_ros_msgs/LandmarkList.h>
#include <cartographer_ros_msgs/LandmarkEntry.h>

#include<iostream>

class Listener
{
private:
    const double WATCHDOG_PERIOD_=1.0;
    const std::string BASE_FOOT_PRINT_;
	const std::string CAMERA_FRAME_;
    const double TRANSLATION_WEIGHT_;
    const double ROTATION_WEIGHT_;
    ros::NodeHandle nh;
    ros::Subscriber tag_info_sub;
    ros::Publisher landmark_list_pub_;
    ros::Timer watchdog_timer_;
    void tagInfoCallback(const visual_servo::TagsDetection_msg &msg);
    void watchdog(const ros::TimerEvent &e);
	Servo *transform_tool_;
public:
    bool tag_info_received;
    Listener(std::string base_foot_print,std::string camera_frame,double translation_weight,double rotation_weight);
    virtual ~Listener();
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "landmark_broadcaster");
    ros::NodeHandle nh_("~");
    std::string base_foot_print,camera_frame;
    double translation_weight,rotation_weight;
    nh_.param("base_foot_print",base_foot_print,(std::string)"base_link");
	nh_.param("camera_frame",camera_frame,(std::string)"camera_color_optical_frame");
    nh_.param("translation_weight",translation_weight,(double)1e3);
    nh_.param("rotation_weight",rotation_weight,(double)1e2);
    Listener listener(base_foot_print,camera_frame,translation_weight,rotation_weight);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

}
Listener::Listener(std::string base_foot_print,std::string camera_frame,double translation_weight,double rotation_weight):BASE_FOOT_PRINT_(std::move(base_foot_print)),CAMERA_FRAME_(std::move(camera_frame)),TRANSLATION_WEIGHT_(std::move(translation_weight)),ROTATION_WEIGHT_(rotation_weight)
{
    this->tag_info_received=false;
    tag_info_sub =nh.subscribe("TagsDetected",1000, &Listener::tagInfoCallback,this);
    watchdog_timer_ = nh.createTimer(ros::Duration(WATCHDOG_PERIOD_), &Listener::watchdog, this, true);
    landmark_list_pub_ =nh.advertise<cartographer_ros_msgs::LandmarkList>("landmark",100);
	transform_tool_ = new Servo;
}
Listener::~Listener()
{
    watchdog_timer_.stop();
	delete transform_tool_;
}

void Listener::tagInfoCallback(const visual_servo::TagsDetection_msg &msg)
{
    watchdog_timer_.stop();
    watchdog_timer_.start();
    this->tag_info_received=true;
    cartographer_ros_msgs::LandmarkList landmark_list{};
    landmark_list.header.stamp = ros::Time::now();
    landmark_list.header.frame_id = BASE_FOOT_PRINT_;
    landmark_list.landmarks.clear();
	Eigen::Affine3d Trans_C2T,Trans_B2C;
    for(auto & tag_information : msg.tags_information)
    {
        cartographer_ros_msgs::LandmarkEntry landmark{};
        landmark.id=std::to_string(tag_information.id);
		tf2::fromMsg(tag_information.pose,Trans_C2T);
		transform_tool_->getTransform(BASE_FOOT_PRINT_,CAMERA_FRAME_,Trans_B2C);
        landmark.tracking_from_landmark_transform = tf2::toMsg(Trans_B2C*Trans_C2T);
        landmark.translation_weight =TRANSLATION_WEIGHT_;
        landmark.rotation_weight =ROTATION_WEIGHT_;
        landmark_list.landmarks.push_back(landmark);
    }
    landmark_list_pub_.publish(landmark_list);
}
void Listener::watchdog(const ros::TimerEvent &e)
{
    ROS_WARN("tag info not received for %f seconds, is the camera node drop?", WATCHDOG_PERIOD_);
    this->tag_info_received=false;
}
