/*
    visual_servo
    TO DO:  Add BREAK statement in go_search;
            Test the search function;
            Add re-search function in go_servo;
*/
#include <iostream>

#include "Servo.h"
#include <unistd.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.h>

#include "ros/callback_queue.h"
#include  <math.h>
#include "visual_servo/TagDetection_msg.h"
#include "visual_servo/TagsDetection_msg.h"


#define _USE_MATH_DEFINES
using namespace std;

//全局变量
vector<TagDetectInfo> Tags_detected;

Servo astra;

pthread_t camera_id;

//函数声明
void *camera_thread(void *data);
void ImageCallback(const sensor_msgs::ImageConstPtr& msg);

class Manipulator
{
    private:
        const std::string PLANNING_GROUP = "manipulator";
        moveit::planning_interface::MoveGroupInterface *move_group;
        moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
        const robot_state::JointModelGroup* joint_model_group;

        geometry_msgs::PoseStamped current_pose;
        Eigen::Affine3d Trans_W2E,Trans_W2EP;
        geometry_msgs::Pose target_pose;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    public:
        double radius=0.2;
        Manipulator();
        virtual ~Manipulator();
        void add_planning_constraint();
        void go_up(double goal_tolerance=0.001,double velocity_scale=1.0);
        bool get_task_plan(string task_name,double task_parameter);
        bool go_search(double search_angle=M_PI,double velocity_scale=0.05,double goal_tolerance=0.001);
        bool go_servo(Eigen::Affine3d ExpectMatrix,Eigen::Affine3d Trans_E2C,double goal_tolerance=0.02,double velocity_scale=1.0);
        void go_cut(double goal_tolerance=0.02,double velocity_scale=0.1);
};

Manipulator::Manipulator()
{
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface;
    joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    ROS_INFO_NAMED("Visual Servo", "Reference frame: %s", move_group->getPlanningFrame().c_str());
    ROS_INFO_NAMED("Visual Servo", "End effector link: %s", move_group->getEndEffectorLink().c_str());
}
Manipulator::~Manipulator()
{
    delete move_group;
    delete planning_scene_interface;
}
void Manipulator::add_planning_constraint()
{
    vector<moveit_msgs::CollisionObject> objects;
    //添加地面以限制规划
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group->getPlanningFrame();
    collision_object.id = "ground";
    shape_msgs::Plane plane;
    plane.coef={0,0,1,0};
    geometry_msgs::Pose object_pose;
    object_pose.position.x=0;
    object_pose.position.y=0;
    object_pose.position.z=-0.2;
    object_pose.orientation.w=1.0;
    collision_object.planes.push_back(plane);
    collision_object.plane_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;
    objects.push_back(collision_object);

    //添加柱子以限制规划
    moveit_msgs::CollisionObject collision_object1;
    collision_object1.header.frame_id = move_group->getPlanningFrame();
    collision_object1.id = "Heava";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 5;
    primitive.dimensions[1] = radius;
    object_pose.position.y=0.7;
    collision_object1.primitives.push_back(primitive);
    collision_object1.primitive_poses.push_back(object_pose);
    collision_object1.operation = collision_object.ADD;
    objects.push_back(collision_object1);

    planning_scene_interface->addCollisionObjects(objects);
}
void Manipulator::go_up(double goal_tolerance,double velocity_scale)
{
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    move_group->setNamedTarget("up");
    move_group->move();
    sleep(2);
}

bool Manipulator::get_task_plan(string task_name,double task_parameter)
{
    geometry_msgs::Pose target_pose = move_group->getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints;
    if(task_name=="cut")
    {
        double init_th = asin(target_pose.position.x / radius);
        double init_y = target_pose.position.y + radius * cos(init_th);

        for (double th = 0.0; th < task_parameter; th += 0.1)
        {
            target_pose.position.x = radius * sin(init_th + th);
            target_pose.position.y = init_y - radius * cos(init_th - th);
            target_pose.position.z -= 0.01;
            waypoints.push_back(target_pose);
        }
    }
    else if (task_name=="back")
    {
        double theta = atan(target_pose.position.y / target_pose.position.x);
        for (int counter = 0; counter < (int) task_parameter*100; ++counter)
        {
            target_pose.position.x -= 0.01 * cos(theta);
            target_pose.position.y -= 0.01 * sin(theta);
            waypoints.push_back(target_pose);
        }
    }

    move_group->allowReplanning(true);
    move_group->setStartStateToCurrentState();
    moveit_msgs::RobotTrajectory trajectory;
    double fraction=0.0;
    int attempts=0;
    while(fraction<1.0&&attempts<100)
    {
        fraction = move_group->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
        attempts++;
        if(attempts%10==0)
            ROS_INFO("Still trying after %d attempts",attempts);
    }
    if(fraction<1.0)
        return false;
    else
    {
        move_group->setStartStateToCurrentState();
        my_plan.trajectory_=trajectory;
    }
    return true;
}
bool Manipulator::go_search(double search_angle,double velocity_scale,double goal_tolerance)
{
    //旋转关节以搜索tag
    if(get_task_plan("back",0.2));
    {
        move_group->setGoalTolerance(goal_tolerance);
        move_group->setMaxVelocityScalingFactor(velocity_scale);

        move_group->asyncExecute(my_plan);
        while(Tags_detected.empty())
        {
            //TO DO;
        }
        move_group->stop();
    }

    return true;
}

bool Manipulator::go_servo(Eigen::Affine3d ExpectMatrix,Eigen::Affine3d Trans_E2C,double goal_tolerance,double velocity_scale)
{
    current_pose=move_group->getCurrentPose();
    Eigen::fromMsg(current_pose.pose,Trans_W2E);
    double error=100;
    Destination_t EndDestination;
    //再检查一次
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);

    while(error>goal_tolerance&&!Tags_detected.empty())
    {
        EndDestination=astra.GetCameraDestination(Tags_detected[0].Trans_C2T,Trans_E2C,ExpectMatrix);
        error=EndDestination.error;
        Trans_W2EP=Trans_W2E*EndDestination.EE_Motion;
        move_group->setPoseTarget(Trans_W2EP);

        bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success)
        {
            ROS_INFO("Reachable");
            move_group->execute(my_plan);
            sleep(2);
        }
        else
        {
            ROS_ERROR("Failed");
            return false;
        }
        current_pose=move_group->getCurrentPose();
        Eigen::fromMsg(current_pose.pose,Trans_W2E);
    }
    return true;
}
void Manipulator::go_cut(double goal_tolerance, double velocity_scale)
{
    if(get_task_plan("cut",M_PI/3))
    {
        move_group->setGoalTolerance(goal_tolerance);
        move_group->setMaxVelocityScalingFactor(velocity_scale);
        move_group->execute(my_plan);
        sleep(2);
    }
    else
    {
        ROS_INFO("CUT LOSE!!!!!");
    }

}


int main(int argc, char** argv)
{
  
    Eigen::Affine3d ExpectMatrix,Trans_E2C;
  
    ExpectMatrix.matrix() << 1,0,0,0,
            0,1,0,0,
            0,0,1,0.2,
            0,0,0,1;
    Trans_E2C.matrix()<<0,0,1,-0.0823,
            -1,0,0,0,
            0,-1,0,0.0675,
            0,0,0,1;
  
    ros::init(argc, argv, "visual_servo");
    ros::NodeHandle n;
    pthread_create(&camera_id, NULL, camera_thread, NULL);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    Manipulator robot_manipulator;

    robot_manipulator.add_planning_constraint();

    robot_manipulator.go_up();
    if(Tags_detected.empty())
    {
        if(robot_manipulator.go_search())
        {
            bool servo_success=robot_manipulator.go_servo(ExpectMatrix,Trans_E2C);
            if(servo_success)
                robot_manipulator.go_cut();
            robot_manipulator.go_up();
        }
        else
        {
            ROS_INFO("!!!!!!NO TAG SEARCHED!!!!!");
            /*
             * PRESERVE SPACE FOR ACT WITH NAVIGATION;
             */
        }
    }
    else
    {
        bool servo_success=robot_manipulator.go_servo(ExpectMatrix,Trans_E2C);
        if(servo_success)
            robot_manipulator.go_cut();
        robot_manipulator.go_up();
    }
    ros::waitForShutdown();
    return 0;

}
void *camera_thread(void *data)
{
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe("/rrbot/camera1/image_raw", 1,&ImageCallback);
    ros::Publisher tag_pub = nh.advertise<visual_servo::TagsDetection_msg>("TagsDetected", 1000);
    ros::Rate loop_rate(15);
    while(ros::ok())
    {
        if(!Tags_detected.empty())
        {
            visual_servo::TagsDetection_msg TagsDetection;
            visual_servo::TagDetection_msg TagDetection;
            TagsDetection.header.frame_id="camera_link";
            TagsDetection.header.stamp=ros::Time::now();
            for(auto & tag_detected : Tags_detected )
            {
                TagDetection.id = tag_detected.id;
                TagDetection.pose = Eigen::toMsg(tag_detected.Trans_C2T);
                TagDetection.PixelCoef = tag_detected.PixelCoef;    
                TagDetection.center.x = tag_detected.Center.x;
                TagDetection.center.y = tag_detected.Center.y;
                TagsDetection.tags_information.push_back(TagDetection);
            } 
            tag_pub.publish(TagsDetection);
        }   
        ros::spinOnce();
        loop_rate.sleep();
    }
}
void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat subscribed_rgb=cv_ptr->image;
    double tagsize=0.8*0.1;
    astra.SetCameraParameter(617.5472412109375,  617.5472412109375,324.5312805175781, 236.61178588867188);
    Tags_detected= astra.GetTargetPoseMatrix(subscribed_rgb, tagsize);
}