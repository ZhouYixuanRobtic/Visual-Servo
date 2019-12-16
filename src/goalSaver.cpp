
#include <ros/ros.h>
#include <ros/package.h>

#include <tf2_ros/transform_listener.h>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Pose2D.h"
#include <iostream>
#include <fstream>
#include "JoyTeleop.h"
using namespace JOYTELEOP;
class GoalSaver{
private:
    const std::string MAP_FRAME_;
    const std::string BASE_FOOTPRINT_FRAME_;
    const std::string FILE_NAME_;
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener *tfListener_;
public:
    enum GoalType{
        CUT,
        CHARGE,
        NAVI,
        TURN
    };
    GoalSaver(std::string map_frame,std::string base_footprint_frame,std::string file_name);
    ~GoalSaver();
    bool getCurrentPose(geometry_msgs::Pose2D & current_pose);
    void saveGoal(const GoalType &goal_type);
};
GoalSaver::GoalSaver(std::string map_frame,std::string base_footprint_frame,std::string file_name): MAP_FRAME_(std::move(map_frame)),BASE_FOOTPRINT_FRAME_(std::move(base_footprint_frame)),FILE_NAME_(std::move(file_name))
{
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);
}
GoalSaver::~GoalSaver()
{
    delete tfListener_;
}
bool GoalSaver::getCurrentPose(geometry_msgs::Pose2D & current_pose)
{
    geometry_msgs::TransformStamped transformStamped{};
    try
    {
        transformStamped = tfBuffer_.lookupTransform(MAP_FRAME_,BASE_FOOTPRINT_FRAME_,
                                                     ros::Time(0),ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        //ros::Duration(1.0).sleep();
        return false;
    }

    current_pose.x=transformStamped.transform.translation.x;
    current_pose.y=transformStamped.transform.translation.y;
    current_pose.theta=tf::getYaw(transformStamped.transform.rotation);
    return true;
}
void GoalSaver::saveGoal(const enum GoalSaver::GoalType & goal_type)
{
    geometry_msgs::Pose2D current_pose{};
    std::ofstream output_file;
    if(getCurrentPose(current_pose))
    {
        static bool first_time=true;
        if(first_time)
        {
            output_file.open(FILE_NAME_.c_str(), std::ios::out | std::ios::trunc);
            first_time=false;
        }
        else
            output_file.open(FILE_NAME_.c_str(), std::ios::out | std::ios::app);
        if(output_file.is_open())
        {
            switch(goal_type)
            {
                case CUT:
                    output_file << (int)goal_type << " " << current_pose.x << " " << current_pose.y << " " << current_pose.theta << std::endl;
                    break;
                case CHARGE:
                    output_file << (int)goal_type << " " << current_pose.x << " " << current_pose.y << " " << current_pose.theta << std::endl;
                    break;
                case TURN:
                    output_file << (int)goal_type << " " << current_pose.x << " " << current_pose.y << " " << current_pose.theta << std::endl;
                    break;
                case NAVI:
                    output_file << (int)goal_type << " " << current_pose.x << " " << current_pose.y << " " << current_pose.theta << std::endl;
                    break;
                default:
                    break;
            }
            output_file.close();
            ROS_INFO("write done with%d %lf  %lf and %lf",(int)goal_type,current_pose.x,current_pose.y,current_pose.theta);
        }
        else
            ROS_ERROR("Can not create or open the pointed file!!!!");
    }
    else
        ROS_ERROR("Can not check out the transform!!!! may the tf goes wrong or no localization!!!");
}
int main(int argc,char* argv[])
{
    ros::init(argc,argv,"goalSaver");
    ros::NodeHandle nh_("~");
    std::string map_frame,base_footprint_frame,file_name;
    nh_.param<std::string>("map_frame",map_frame,"map");
    nh_.param<std::string>("base_footprint_frame",base_footprint_frame,"base_link");
    nh_.param<std::string>("file_name",file_name,ros::package::getPath("visual_servo")+"/config/treeTarget.txt");
    GoalSaver goalSaver(map_frame,base_footprint_frame,file_name);
    JoyTeleop joyTeleop ( "joy" );

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        switch ( joyTeleop.getControlTrigger() )
        {
            case SaveCutPoint:
                goalSaver.saveGoal(GoalSaver::CUT);
                break;
            case SaveChargePoint:
                goalSaver.saveGoal(GoalSaver::CHARGE);
                break;
            case SaveNavPoint:
                goalSaver.saveGoal(GoalSaver::NAVI);
                break;
            case SaveTurnPoint:
                goalSaver.saveGoal(GoalSaver::TURN);
                break;
            default:
                break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}