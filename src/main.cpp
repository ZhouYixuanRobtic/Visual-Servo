/*
    visual_servo
    TO DO:  DELETE COLLISION OBJECT UNDER THE PLANNING SCENE done
            ADD GO HOME done

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
#include "visp_hand2eye_calibration/TransformArray.h"


#define _USE_MATH_DEFINES


//全局变量
std::vector<TagDetectInfo> Tags_detected;

Servo astra;

pthread_t camera_id;
int counter=0;
//函数声明
void *camera_thread(void *data);

//操作器类声明，物理上对应机械臂的一系列操作
class Manipulator
{
private:
    const std::string PLANNING_GROUP = "Manipulator";
    moveit::planning_interface::MoveGroupInterface *move_group;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group;

    geometry_msgs::PoseStamped current_pose;
    Eigen::Affine3d Trans_W2E,Trans_W2EP;


public:
    double radius=0.2;
    Manipulator();
    virtual ~Manipulator();
    void add_planning_constraint();
    void go_up(double goal_tolerance=0.001,double velocity_scale=1.0);
    void go_home(double goal_tolerance=0.001,double velocity_scale=1.0);
    bool go_search_once(std::string search_type,double search_angle=M_PI,double velocity_scale=0.05,double goal_tolerance=0.001);
    bool go_search(double search_angle=M_PI,double velocity_scale=0.05,double goal_tolerance=0.001);
    bool go_servo(Eigen::Affine3d ExpectMatrix,Eigen::Affine3d Trans_E2C,double goal_tolerance=0.02,double velocity_scale=1.0);
    void go_cut(double velocity_scale=0.1);
};

//相机相关类声明，负责相机有关操作
class Listener
{
private:
    double fx,fy,u0,v0;
    ros::Publisher tag_pub;
    ros::NodeHandle nh;
    image_transport::ImageTransport *it;
    image_transport::Subscriber image_sub;
    ros::Subscriber camera_info_sub;
    ros::Subscriber tf_sub;
    Eigen::Affine3d Trans_W2_W3;

public:
    bool image_received,camera_info_received;
    Listener();
    virtual ~Listener();
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void CameraInfoCallback(const sensor_msgs::CameraInfo& msg);
    void TfCallback(const tf2_msgs::TFMessage &msg);
    void TagsInfoPublish();
};

int main(int argc, char** argv)
{

    Eigen::Affine3d ExpectMatrix,Trans_E2C;

    ExpectMatrix.matrix() << 1,0,0,0,
            0,1,0,0,
            0,0,1,0.2,
            0,0,0,1;
    Trans_E2C.matrix()<<    0.00867266,-0.00163184,0.999961,-0.138307,
                            0.00659928,-0.999977,-0.0016891,0.065175,
                            0.999941,0.00661367,-0.00866169,0.0790898,
                            0,0,0,1;

    ros::init(argc, argv, "visual_servo");
    ros::NodeHandle n;

    pthread_create(&camera_id, NULL, camera_thread, NULL);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    Manipulator robot_manipulator;

    robot_manipulator.add_planning_constraint();
    robot_manipulator.go_home();
    /*if(Tags_detected.empty())
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

        }
    }
    else
    {
        bool servo_success=robot_manipulator.go_servo(ExpectMatrix,Trans_E2C);
        if(servo_success)
            robot_manipulator.go_cut();
        robot_manipulator.go_up();
    }
    */
    ros::waitForShutdown();
    return 0;

}

void *camera_thread(void *data)
{

    Listener listener;

    ros::Rate loop_rate(15);
    int counter;
    while(ros::ok())
    {
        listener.TagsInfoPublish();
        counter++;
        counter %=31;
        if(counter>=30&&!listener.image_received)
        {
            ROS_ERROR("No image received!!!!");
            //break;
        }

        ros::spinOnce();
        loop_rate.sleep();

    }
    ros::shutdown();
}


Manipulator::Manipulator()
{
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    planning_scene_interface = new  moveit::planning_interface::PlanningSceneInterface;
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
    std::vector<std::string> object_names;
    object_names=planning_scene_interface->getKnownObjectNames();
    if(!object_names.empty())
    {
        planning_scene_interface->removeCollisionObjects(object_names);
    }
    std::vector<moveit_msgs::CollisionObject> objects;
    //添加地面以限制规划
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group->getPlanningFrame();
    collision_object.id = "ground";
    shape_msgs::Plane plane;
    plane.coef={0,0,1,0};
    geometry_msgs::Pose object_pose;
    object_pose.position.x=0;
    object_pose.position.y=0;
    object_pose.position.z=0;
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
    object_pose.position.y=-0.7;
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
void Manipulator::go_home(double goal_tolerance,double velocity_scale)
{
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    move_group->setNamedTarget("home");
    move_group->move();
    sleep(2);
}
bool Manipulator::go_search_once(std::string search_type,double search_angle,double velocity_scale,double goal_tolerance)
{
    //旋转关节以搜索tag
    move_group->setGoalTolerance(goal_tolerance);
    std::vector<double> goal;
    if(search_type=="flat")
        goal=move_group->getCurrentJointValues();
    else if(search_type=="look up")
        goal={-0.801147672791,-1.46294256182,-0.503654629856,-0.851993156669,0.827502945626,-0.222524404845};
    else if(search_type=="look down")
        goal={-0.961324653895,-1.47830998925,-0.343705555765,-1.60330331121,0.980292289712,0.161102308097};
    else
    {
        ROS_ERROR("Wrong search type!!!!");
        return false;
    }
    move_group->setMaxVelocityScalingFactor(1.0);
    goal[4]-=search_angle/2;
    move_group->setJointValueTarget(goal);
    move_group->move();
    sleep(2);
    goal[4]+=search_angle;
    move_group->setJointValueTarget(goal);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    move_group->asyncMove();
    std::vector<double> current_joints;
    while(Tags_detected.empty())
    {
        //ROS_INFO("Searching tag!!!!!");
        current_joints=move_group->getCurrentJointValues();
        std::vector<double>	auxiliary;

        std::transform (current_joints.begin(), current_joints.end(), goal.begin(), std::back_inserter(auxiliary),//
                        [](double element1, double element2) {return pow((element1-element2),2);});
        auxiliary.shrink_to_fit();
        if(sqrt(std::accumulate(auxiliary.begin(), auxiliary.end(), 0.0))<=goal_tolerance*2)
            return false;
    }
    move_group->stop();
    return true;
}
bool Manipulator::go_search(double search_angle, double velocity_scale, double goal_tolerance)
{
    return  go_search_once("flat",search_angle,velocity_scale,goal_tolerance) ||
            go_search_once("look down",search_angle,velocity_scale,goal_tolerance) ||
            go_search_once("look up",search_angle,velocity_scale,goal_tolerance);
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

    while(error>0.02)
    {
        if(!Tags_detected.empty()||go_search())
        {
            EndDestination=astra.GetCameraDestination(Tags_detected[0].Trans_C2T,Trans_E2C,ExpectMatrix);
            error=EndDestination.error;
            Trans_W2EP=Trans_W2E*EndDestination.EE_Motion;
            move_group->setPoseTarget(Trans_W2EP);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
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

    }
    return true;
}
void Manipulator::go_cut(double velocity_scale)
{
    geometry_msgs::Pose target_pose3 = move_group->getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints;

    double init_th=asin(target_pose3.position.x/radius);
    double init_y=target_pose3.position.y+radius*cos(init_th);

    for(double th=0.0;th<M_PI/3;th+=0.1)
    {
        target_pose3.position.x = radius*sin(init_th+th);
        target_pose3.position.y = init_y-radius*cos(init_th-th) ;
        target_pose3.position.z -=0.01;
        waypoints.push_back(target_pose3);
    }
    move_group->allowReplanning(true);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
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
    ROS_INFO_NAMED("visual servo", "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if(fraction<1.0)
        ROS_INFO("NO Trajectory");
    else
    {
        my_plan.trajectory_=trajectory;
        move_group->execute(my_plan);
        sleep(2);
    }
}

Listener::Listener()
{
    this->fx=615.3072;
    this->fy=616.1456;
    this->u0=333.4404;
    this->v0=236.2650;

    this->image_received= false;
    this->camera_info_received=false;

    Trans_W2_W3.matrix()<<   1,0,0,0,
            0,0,-1,-0.094,
            0,1,0,0,
            0,0,0,1;
    it = new image_transport::ImageTransport(nh);
    image_sub = it->subscribe("/camera/color/image_raw", 1,&Listener::ImageCallback,this);
    camera_info_sub = nh.subscribe("/rrbot/camera/image_raw",1,&Listener::CameraInfoCallback,this);
    //tf_sub = nh.subscribe("/tf_static",1000,&Listener::TfCallback,this);
    tag_pub = nh.advertise<visual_servo::TagsDetection_msg>("TagsDetected", 1000);
}
Listener::~Listener()
{
    delete it;
}
void Listener::CameraInfoCallback(const sensor_msgs::CameraInfo &msg)
{
    this->camera_info_received= true;
    this->fx=msg.K[0];
    this->fy=msg.K[4];
    this->u0=msg.K[2];
    this->v0=msg.K[5];
}
void Listener::ImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    this->image_received=true;
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
    double tag_size=0.8*0.1;
    astra.SetCameraParameter(this->fx,this->fy,this->u0,this->v0);
    Tags_detected= astra.GetTargetPoseMatrix(subscribed_rgb, tag_size);
}
void Listener::TfCallback(const tf2_msgs::TFMessage &msg)
{
    Eigen::Affine3d Trans_E2C,Trans_W2_C,Trans_W3_K,Trans_K_E,Trans_C_CC,Trans_CC_CCO;
    for (auto & transform : msg.transforms)
    {
        if(transform.header.frame_id=="wrist2_Link"&&transform.child_frame_id=="camera_link")
            Trans_W2_C=tf2::transformToEigen(transform);
        else if(transform.header.frame_id=="wrist3_Link"&&transform.child_frame_id=="knife_link")
            Trans_W3_K=tf2::transformToEigen(transform);
        else if(transform.header.frame_id=="knife_link"&&transform.child_frame_id=="ee_link")
            Trans_K_E=tf2::transformToEigen(transform);
        else if(transform.header.frame_id=="camera_link"&&transform.child_frame_id=="camera_color_frame")
            Trans_C_CC=tf2::transformToEigen(transform);
        else if(transform.header.frame_id=="camera_color_frame"&&transform.child_frame_id=="camera_color_optical_frame")
            Trans_CC_CCO=tf2::transformToEigen(transform);
    }
    Trans_E2C=Trans_K_E.inverse()*Trans_W3_K.inverse()*Trans_W2_W3.inverse()*Trans_W2_C*Trans_C_CC*Trans_CC_CCO;
    std::cout<<Trans_E2C.matrix()<<std::endl;

}
void Listener::TagsInfoPublish()
{
    if(!Tags_detected.empty())
    {
        visual_servo::TagsDetection_msg TagsDetection;
        visual_servo::TagDetection_msg TagDetection;
        TagsDetection.header.frame_id="camera_color_optical_frame";
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
}

