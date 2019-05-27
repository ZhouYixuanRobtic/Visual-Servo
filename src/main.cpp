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
#include <tf2_ros/transform_listener.h>


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
    const std::string PLANNING_GROUP = "manipulator_i5";
    moveit::planning_interface::MoveGroupInterface *move_group;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group;

    geometry_msgs::PoseStamped current_pose;
    Eigen::Affine3d Trans_W2E,Trans_W2EP;
    double goal_tolerance;

    double all_close(std::vector<double> goal);

public:
    double radius=0.2;
    Manipulator();
    virtual ~Manipulator();
    void add_planning_constraint(Eigen::Affine3d Trans_E2C);
    void go_up(double velocity_scale=0.5);
    void go_home(double velocity_scale=0.5);
    bool go_search_once(std::string search_type,double search_angle=M_PI,double velocity_scale=0.5);
    bool go_search(double search_angle=M_PI,double velocity_scale=0.5);
    bool go_servo(Eigen::Affine3d ExpectMatrix,Eigen::Affine3d Trans_E2C,double velocity_scale=0.5);
    void go_cut(double velocity_scale=0.1);
    void go_calibrate(double velocity_scale=0.5);
    void go_test(double velocity_scale=0.5);

};

//相机相关类声明，负责相机有关操作
class Listener
{
private:
    ros::NodeHandle nh;

    ros::Subscriber tag_info_sub;
    ros::Subscriber tf_sub;
    ros::Publisher calibrate1,calibrate2;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;


public:
    bool tag_info_received;

    Listener();
    virtual ~Listener();
    void TagInfoCallback(const visual_servo::TagsDetection_msg& msg);
    void TfCallback(const tf2_msgs::TFMessage &msg);
    void CalibrateInfoPublish();
};

int main(int argc, char** argv)
{

    Eigen::Affine3d ExpectMatrix,Trans_E2C;
    ExpectMatrix.matrix() << 1,0,0,0,
            0,1,0,0,
            0,0,1,0.3,
            0,0,0,1;

    Trans_E2C.matrix()<<    -0.999933,-0.00661497,0.00945798,0.0298311,
                            0.00659931,-0.999977,-0.00168542,0.0801745,
                            0.00946891,-0.0016229,0.999954, -0.141952,
                            0,           0,           0,           1;


    ros::init(argc, argv, "visual_servo");
    ros::NodeHandle n;

    pthread_create(&camera_id, NULL, camera_thread, NULL);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    Manipulator robot_manipulator;

    robot_manipulator.go_up();
    Tags_detected.clear();
    sleep(2);

    if(Tags_detected.empty())
    {

        if(robot_manipulator.go_search())
        {
            robot_manipulator.add_planning_constraint(Trans_E2C);
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
    {       while(true)
        {
            ROS_INFO("ANOTHER ONE");
            bool servo_success = robot_manipulator.go_servo(ExpectMatrix, Trans_E2C);
            if (servo_success)
                robot_manipulator.go_cut();
            robot_manipulator.go_up();
        }
    }
    ros::waitForShutdown();
    return 0;

}

void *camera_thread(void *data)
{

    Listener listener;

    ros::Rate loop_rate(30);
    int counter;
    while(ros::ok())
    {
        //listener.CalibrateInfoPublish();
        counter++;
        counter %=61;
        if(counter>=60)
        {
            if(!listener.tag_info_received)
            {
                ROS_ERROR("No tag received!!!!");
                //break;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();

    }
    ros::shutdown();
}


Manipulator::Manipulator()
{
    goal_tolerance=0.001;
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
void Manipulator::add_planning_constraint(Eigen::Affine3d Trans_E2C)
{
    Eigen::Affine3d Trans_B2E,Trans_B2T;
    current_pose=move_group->getCurrentPose();
    Eigen::fromMsg(current_pose.pose,Trans_B2E);
    Trans_B2T=Trans_B2E*Trans_E2C*Tags_detected[0].Trans_C2T;
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
    object_pose.position.z=-0.08;
    object_pose.orientation.w=1.0;
    collision_object.planes.push_back(plane);
    collision_object.plane_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;
    objects.push_back(collision_object);
    //侧边虚拟墙
    moveit_msgs::CollisionObject collision_object2;
    collision_object2.header.frame_id = move_group->getPlanningFrame();
    collision_object2.id = "virtual_wall";
    shape_msgs::Plane plane1;
    plane1.coef={1,0,0,-0.6};
    object_pose.position.x=0;
    object_pose.position.y=0;
    object_pose.position.z=0;
    object_pose.orientation.w=1.0;
    collision_object2.planes.push_back(plane1);
    collision_object2.plane_poses.push_back(object_pose);
    collision_object2.operation = collision_object2.ADD;
    objects.push_back(collision_object2);

    //添加柱子以限制规划
    moveit_msgs::CollisionObject collision_object1;
    collision_object1.header.frame_id = move_group->getPlanningFrame();
    collision_object1.id = "Heava";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 5;
    primitive.dimensions[1] = radius;
    object_pose.position.y=Trans_B2T.translation()[1]-radius;
    collision_object1.primitives.push_back(primitive);
    collision_object1.primitive_poses.push_back(object_pose);
    collision_object1.operation = collision_object.ADD;
    objects.push_back(collision_object1);

    planning_scene_interface->addCollisionObjects(objects);
}
void Manipulator::go_up(double velocity_scale)
{
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    move_group->setNamedTarget("up");
    move_group->move();

}
void Manipulator::go_home(double velocity_scale)
{
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    move_group->setNamedTarget("home");
    move_group->move();
}
double Manipulator::all_close(std::vector<double> goal)
{
    std::vector<double> current_joints;
    current_joints=move_group->getCurrentJointValues();
    std::vector<double>	auxiliary;

    std::transform (current_joints.begin(), current_joints.end(), goal.begin(), std::back_inserter(auxiliary),//
                    [](double element1, double element2) {return pow((element1-element2),2);});
    auxiliary.shrink_to_fit();
    return sqrt(std::accumulate(auxiliary.begin(), auxiliary.end(), 0.0));
}
bool Manipulator::go_search_once(std::string search_type,double search_angle,double velocity_scale)
{
    //旋转关节以搜索tag
    move_group->setGoalTolerance(goal_tolerance);
    std::vector<double> goal;
    if(search_type=="flat")
        goal=move_group->getCurrentJointValues();
    else if(search_type=="look up")
        goal={-1.30988073349,-0.435410380363,-0.587560653687,0.0236551128328,1.31350243092,-0.0452341213822};
    else if(search_type=="look down")
        goal={-1.21818768978,0.162731602788,0.505937635899,0.0003668028221,1.2371635437,0.11619579047};
    else
    {
        ROS_ERROR("Wrong search type!!!!");
        return false;
    }
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    goal[4]-=search_angle/3;
    move_group->setJointValueTarget(goal);
    move_group->move();
    goal[4]+=2*search_angle/3;
    move_group->setJointValueTarget(goal);
    move_group->setMaxVelocityScalingFactor(0.1);
    move_group->asyncMove();
    while(Tags_detected.empty())
    {
        //ROS_INFO("Searching tag!!!!!");
        if(all_close(goal)<=goal_tolerance*2)
            return false;
    }
    move_group->rememberJointValues("tag");
    while(all_close(goal)>goal_tolerance*2);
    sleep(1);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    move_group->setNamedTarget("tag");
    move_group->move();
    return true;
}
bool Manipulator::go_search(double search_angle, double goal_tolerance)
{
    return  go_search_once("flat",search_angle,goal_tolerance) ||
            go_search_once("look down",search_angle,goal_tolerance) ||
            go_search_once("look up",search_angle,goal_tolerance);
}
bool Manipulator::go_servo(Eigen::Affine3d ExpectMatrix,Eigen::Affine3d Trans_E2C,double velocity_scale)
{
    current_pose=move_group->getCurrentPose();
    Eigen::fromMsg(current_pose.pose,Trans_W2E);

    double error=100;
    Destination_t EndDestination;

    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);

    while(error>goal_tolerance)
    {
        if(!Tags_detected.empty())
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
    double init_y=target_pose3.position.y-radius*cos(init_th);

    for(double th=0.0;th<M_PI/3;th+=0.1)
    {
        target_pose3.position.x = radius*sin(init_th+th);
        target_pose3.position.y = init_y+radius*cos(init_th+th) ;
        target_pose3.position.z -=0.015;
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
    }
}
void Manipulator::go_calibrate(double velocity_scale)
{
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    std::vector<double> goal;
    //goal={-3.4278423142,-2.77082756955,1.87070843246,-2.59328705999,3.75959096566,0.0122381052633};
    //goal={-0.58422814502,0.177696018525,-0.541309433389,-0.843468093972,0.747598184228,-0.395293140634};
    //goal={-1.08977861425,-1.4569088675,0.379996591692,-2.1473862574,1.33579486779,-0.221435780724};
    goal={-0.10736,0.51789,0.56852,0.05063,0.10675,-0.00061};
    move_group->setJointValueTarget(goal);
    move_group->move();
}
void Manipulator::go_test( double goal_tolerance)
{
    Eigen::Vector3d x_previous,x_now;
    Eigen::Vector3d ea(-1.572, -0.007, -3.132);
    move_group->setGoalTolerance(goal_tolerance);
    //3.1 欧拉角转换为旋转矩阵
    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitX());
    Eigen::Affine3d Trans_B2C;
    Trans_B2C.linear()=rotation_matrix3;
    Eigen::Vector3d B=Eigen::Vector3d(0.0,0.0,-0.1);
    Eigen::Vector3d A=Trans_B2C*B;

    if(!Tags_detected.empty())
    {
        x_previous=Tags_detected[0].Trans_C2T.translation();
        current_pose=move_group->getCurrentPose();
        current_pose.pose.position.x +=B[0];
        current_pose.pose.position.y +=B[1];
        current_pose.pose.position.z +=B[2];
        move_group->setPoseTarget(current_pose);
        move_group->move();
        sleep(2);
        x_now=Tags_detected[0].Trans_C2T.translation();
        x_now=x_now-x_previous+A;
        std::cout<<"error:"<<(abs(x_now[0])+abs(x_now[1])+abs(x_now[2]))/3*1000.0<<std::endl;
        std::cout<<"error vector"<<x_now*1000.0<<std::endl;
    }
}
Listener::Listener()
{
    this->tag_info_received=false;
    tag_info_sub =nh.subscribe("TagsDetected",1000,&Listener::TagInfoCallback,this);
    //tf_sub = nh.subscribe("/tf_static",1000,&Listener::TfCallback,this);
    //calibrate1 = nh.advertise<geometry_msgs::Transform>("world_effector",1000);
    //calibrate2 = nh.advertise<geometry_msgs::Transform>("camera_object",1000);

}
Listener::~Listener()
{
    delete tfListener;
}

void Listener::TagInfoCallback(const visual_servo::TagsDetection_msg &msg)
{
    this->tag_info_received=true;
    TagDetectInfo Tag_detected;
    Tags_detected.clear();
    for(auto & tag_information : msg.tags_information)
    {
        Eigen::fromMsg(tag_information.pose,Tag_detected.Trans_C2T);
        Tag_detected.PixelCoef=tag_information.PixelCoef;
        Tag_detected.Center.x=tag_information.center.x;
        Tag_detected.Center.y=tag_information.center.y;
        Tag_detected.id=tag_information.id;
        Tags_detected.push_back(Tag_detected);
    }
}
void Listener::TfCallback(const tf2_msgs::TFMessage &msg)
{
    Eigen::Affine3d Trans_E2C,Trans_C2T,Trans_W2_C,Trans_W3_K,Trans_K_E,Trans_C_CC,Trans_CC_CCO;
    for (auto & transform : msg.transforms)
    {
        if(transform.header.frame_id=="wrist3_Link"&&transform.child_frame_id=="knife_link")
            Trans_W3_K=tf2::transformToEigen(transform);
        else if(transform.header.frame_id=="knife_link"&&transform.child_frame_id=="ee_link")
            Trans_K_E=tf2::transformToEigen(transform);
        else if(transform.header.frame_id=="camera_link"&&transform.child_frame_id=="camera_color_frame")
            Trans_C_CC=tf2::transformToEigen(transform);
        else if(transform.header.frame_id=="camera_color_frame"&&transform.child_frame_id=="camera_color_optical_frame")
            Trans_CC_CCO=tf2::transformToEigen(transform);
    }

}
void Listener::CalibrateInfoPublish()
{
    //geometry_msgs::Transform world_effector,camera_object;
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("camera_link", "camera_color_optical_frame",
                                                    ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        //ros::Duration(1.0).sleep();
    }
    Eigen::Affine3d Trans_E2C=tf2::transformToEigen(transformStamped.transform);
    std::cout<<"try"<<Trans_E2C.matrix()<<std::endl;
    //world_effector=transformStamped.transform;
    //calibrate1.publish(world_effector);
    //camera_object=tf2::eigenToTransform(Tags_detected[0].Trans_C2T).transform;
    //calibrate2.publish(camera_object);


}

