#include "Servo.h"

#include <unistd.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "ros/callback_queue.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include "visual_servo/TagDetection_msg.h"
#include "visual_servo/TagsDetection_msg.h"
#include "visual_servo/detect_once.h"
#include "visual_servo/manipulate.h"
#include "visual_servo/VisualServoMetaTypeMsg.h"
#include "VisualServoMetaType.h"



#define _USE_MATH_DEFINES

/*Global variables action in multi threads
 * @param Tags_detected     [Tags information received from camera thread, vector is empty when no tags were detected]
 * @param detect_srv_on     [The knife trace detect service trigger]
 * @param knife_trace       [The knife trace returned by trace detect service]
 * @param manipulate_srv_on [The manipulation service trigger]
 * @param RobotAllRight     [The robot status, false for something wrong]
 */
std::vector<TagDetectInfo> Tags_detected;
bool detect_srv_on=false;
bool manipulate_srv_on=false;
bool RobotAllRight;
std::vector<Eigen::Vector3d> knife_trace;


struct manipulateSrv{
    int srv_type;
    int srv_status;
}ManipulateSrv;

void *camera_thread(void *data);

class Manipulator
{
private:
    ros::NodeHandle n_;
    /*
     * Specifies search operations
     * The enum specifies three search operations,FLAT,DOWN,UP
     * of searching tags when no tags detected.
     * Default for doing nothing.
     */
    enum SearchType{
        FLAT=0,
        DOWN=1,
        UP=2,
        DEFAULT=3
    };

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    //the planning_group name specified by the moveit_config package
    const std::string PLANNING_GROUP = "manipulator_i5";

    //the name of rubber tree
    const std::string TREE_NAME = "Heava";
    std::string EE_NAME;
    moveit::planning_interface::MoveGroupInterface *move_group;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group;

    double goal_tolerance;

    //Servo class for computing the servo related transform matrix
    Servo *astra;

    geometry_msgs::PoseStamped current_pose;

    /*
     * The transform matrix of A with respect to B named as Trans_B2A.
     * W-world,E-end effector,C-camera,EP-desired end effector,B-base_link,T-target
     */
    Eigen::Affine3d Trans_W2E,Trans_W2EP,Trans_E2C,Trans_E2CH;

    Eigen::Affine3d ExpectMatrix;

    /* Function for computing the joint space Euclidean distance,
     * between the given goal and now joint states.
     * @param goal  [the given joint space goal]
     * @return the joint space Euclidean distance as a double number,
     *  Unit-radian
     */
    double allClose(std::vector<double> goal);
    /* Function for computing the matrix of desired end effector motion.
     * @param search_type   [define the reference coordinate system of desired motion,
     *                      UP for end effector, DOWN for world, DEFAULT for end effector]
     * @param Translation   [motion's translation part]
     * @param RPY           [motion's rotation part described by RPY values]
     * @return the desired end effector motion matrix.
     */
    Eigen::Affine3d getEndMotion(int search_type,
                                 Eigen::Vector3d Translation = Eigen::Vector3d(0.0, 0.0, 0.0),
                                 Eigen::Vector3d RPY = Eigen::Vector3d(0.0, 0.0, 0.0));
    void initParameter();

    //prameters
    //the radius of rubber tree, used as a reference for constructing planning constraints,and generating the cut locus.
    double radius;
    Eigen::Vector3d expectRPY,expectXYZ,cameraXYZ,E2CRPY,E2CXYZ,E2CHRPY,E2CHXYZ;
    bool inverse;
    bool servoPoseOn;
    double searchDownHeight;
    double basicVelocity;
    double servoTolerance;
    int traceNumber;
    double traceDistance;
    double traceAngle;
public:


    Manipulator();

    virtual ~Manipulator();

    // Function aims at adding static virtual walls to restrict planning
    void addStaticPlanningConstraint();
    // Function aims at adding dynamic virtual cylinder as tree to restrict planning
    void addDynamicPlanningConstraint();
    // Function aims at adding dynamic virtual cylinder as charger plane to restrict planning
    void addChargerDynamicPlanningConstraint();
    /* Function makes all robot joints go to zero
     * @param reset [true for go up to from the other side]
     */
    bool goUp(double velocity_scale ,bool reset=true);
    // Function makes robot go to a preset position
    void goHome(double velocity_scale);
    /* Function makes robot go search once with a defined search angle range
     * @param search_type       [define the search type,UP for looking up,DOWN for looking down,
     *                          FLAT for looking flat,DEFAULT for doing nothing]
     * @param search_angle      [define the search angle range 0-pi]
     * @param velocity_scale    [velocity scaling factor 0-1]
     * @return true if any tag searched else false
     */
    bool goSearchOnce(int search_type, double velocity_scale, double search_angle = M_PI / 3.0);
    //Function makes robot search all three types
    bool goSearch( double velocity_scale,double search_angle = M_PI / 3.0);
    /*Function makes robot camera servo to pointed pose or position
     * @param ExpectMatrix  [the desired pose of camera describing as transform matrix]
     * @param enable_pose   [true for enable pose servo mode which costs more adjust time and may makes end effector rotate,
     *                      false for position servo only]
     * @return true if achieved the desired pose or position
     */
    bool goServo(double velocity_scale );
    //Function makes robot accomplish cut task
    bool goCut(Eigen::Affine3d referTag,double velocity_scale );
    //Function makes end effector joint goes to zero position.
    void goZero(double velocity_scale = 1.0);
    /*Function makes the end effector goes to a position defined by a array with respect to camera
     * @param target_array  [describes a position with respect to camera]
     * @return true if achieved the desired position.
     */
    bool goCamera(Eigen::Vector3d target_array, double velocity_scale = 0.1);

    bool goCharge(double velocity_scale = 0.5);
    bool leaveCharge(double velocity_scale=0.5);

    int executeService(int serviceType);

    Eigen::Affine3d getTagPosition(Eigen::Affine3d Trans_C2T);
    //debug preserved functions

};

//all ros topics and services related functions
class Listener
{
private:
    ros::NodeHandle nh;
    ros::ServiceClient client;
    ros::Subscriber tag_info_sub;
    ros::Subscriber tf_sub;
    ros::Subscriber vs_status_sub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    ros::ServiceServer service_;
    visual_servo::detect_once srv;
    void tagInfoCallback(const visual_servo::TagsDetection_msg &msg);
    void tfCallback(const tf2_msgs::TFMessage &msg);
    void statusCallback(const visual_servo::VisualServoMetaTypeMsg &msg);
    bool manipulate(visual_servo::manipulate::Request &req,
                    visual_servo::manipulate::Response &res);

public:
    typedef boost::function<bool (visual_servo::manipulate::Request&,visual_servo::manipulate::Response& res)>
            manipulate_callback_t;
    bool tag_info_received;
    Listener();
    virtual ~Listener();
    void calibrateInfoPublish();
    bool callSrv();
};

int main(int argc, char** argv)
{

    pthread_t camera_id;
    ros::init(argc, argv, "visual_servo");

    pthread_create(&camera_id, NULL, camera_thread, NULL);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    Manipulator robot_manipulator;
    robot_manipulator.addStaticPlanningConstraint();
    while(ros::ok())
    {
        if(manipulate_srv_on)
        {
            if(RobotAllRight)
            {
                robot_manipulator.goUp(0.2,false);
                ManipulateSrv.srv_status=robot_manipulator.executeService(ManipulateSrv.srv_type);
            }
            else
                ManipulateSrv.srv_status=visual_servo_namespace::SERVICE_STATUS_ROBOT_ABORT;
            //check again
            if(!RobotAllRight)
                ManipulateSrv.srv_status=visual_servo_namespace::SERVICE_STATUS_ROBOT_ABORT;
            manipulate_srv_on=false;
        }
    }
    ros::waitForShutdown();
    return 0;

}

void *camera_thread(void *data)
{
    Listener listener;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Rate loop_rate(30);
    int counter;
    while(ros::ok())
    {
        //listener.calibrateInfoPublish();
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
        if(detect_srv_on)
        {
            detect_srv_on=!listener.callSrv();
        }
        //ros::spinOnce();
        loop_rate.sleep();
    }
    ros::waitForShutdown();
}


Manipulator::Manipulator()
{

    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    planning_scene_interface = new  moveit::planning_interface::PlanningSceneInterface;
    joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    astra =new Servo(false);

    ROS_INFO_NAMED("Visual Servo", "Reference frame: %s", move_group->getPlanningFrame().c_str());
    EE_NAME=move_group->getEndEffectorLink().c_str();
    ROS_INFO_NAMED("Visual Servo", "End effector link: %s", EE_NAME);
    initParameter();
}
Manipulator::~Manipulator()
{
    delete move_group;
    delete planning_scene_interface;
    delete astra;
    delete tfListener;
}
void Manipulator::initParameter()
{
    n_.getParam("/visual_servo/user/radius",radius);
    n_.getParam("/visual_servo/user/expectRoll",expectRPY[0]);
    n_.getParam("/visual_servo/user/expectPitch",expectRPY[1]);
    n_.getParam("/visual_servo/user/expectYaw",expectRPY[2]);
    n_.getParam("/visual_servo/user/expectX",expectXYZ[0]);
    n_.getParam("/visual_servo/user/expectY",expectXYZ[1]);
    n_.getParam("/visual_servo/user/expectZ",expectXYZ[2]);
    n_.getParam("/visual_servo/user/inverse",inverse);
    n_.getParam("/visual_servo/user/servoPoseOn",servoPoseOn);
    n_.getParam("/visual_servo/user/searchDownHeight",searchDownHeight);
    n_.getParam("/visual_servo/user/goCameraX",cameraXYZ[0]);
    n_.getParam("/visual_servo/user/goCameraY",cameraXYZ[1]);
    n_.getParam("/visual_servo/user/goCameraZ",cameraXYZ[2]);
    n_.getParam("/visual_servo/user/angle",traceAngle);
    n_.getParam("/visual_servo/user/distance",traceDistance);
    n_.getParam("/visual_servo/user/pointsNumber",traceNumber);
    n_.getParam("/visual_servo/robot/basicVelocity",basicVelocity);
    n_.getParam("/visual_servo/robot/goalTolerance",goal_tolerance);
    n_.getParam("/visual_servo/robot/servoTolerance",servoTolerance);

    std::cout<<"read parameter success"<<std::endl;

    Eigen::Matrix3d rotation_matrix;
    rotation_matrix=Eigen::AngleAxisd(expectRPY[2], Eigen::Vector3d::UnitZ())*
                    Eigen::AngleAxisd(expectRPY[1], Eigen::Vector3d::UnitY())*
                    Eigen::AngleAxisd(expectRPY[0], Eigen::Vector3d::UnitX());
    ExpectMatrix.linear()=rotation_matrix;
    ExpectMatrix.translation()=expectXYZ;
    geometry_msgs::TransformStamped transformE2C,transformE2CH;
    try
    {
        transformE2C = tfBuffer.lookupTransform(EE_NAME, "camera_color_optical_frame",
                                                    ros::Time(0),ros::Duration(3.0));
        transformE2CH = tfBuffer.lookupTransform(EE_NAME, "charger_link",
                                                ros::Time(0),ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        //ros::Duration(1.0).sleep();
    }
    Trans_E2C=tf2::transformToEigen(transformE2C.transform);
    Trans_E2CH=tf2::transformToEigen(transformE2CH.transform);
    /*备注：此处的变换和参数文件不一致，充电运动前需确认*/
}

void Manipulator::addStaticPlanningConstraint()
{
    std::vector<std::string> object_names;
    object_names=planning_scene_interface->getKnownObjectNames();
    if(!object_names.empty())
    {
        for(auto & object_name : object_names)
        {
            if(object_name==TREE_NAME)
            {
                object_names.erase(std::remove(std::begin(object_names),std::end(object_names),object_name),\
                std::end(object_names));
            }
        }
        planning_scene_interface->removeCollisionObjects(object_names);
    }
    std::vector<moveit_msgs::CollisionObject> objects;

    //virtual ground
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group->getPlanningFrame();
    collision_object.id = "ground";
    shape_msgs::Plane plane;
    plane.coef={0,0,1,0};
    geometry_msgs::Pose object_pose;
    object_pose.position.x=0;
    object_pose.position.y=0;
    object_pose.position.z=0.0;
    object_pose.orientation.w=1.0;
    collision_object.planes.push_back(plane);
    collision_object.plane_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;
    objects.push_back(collision_object);

    //virtual left side wall
    moveit_msgs::CollisionObject collision_object_LIDAR;
    collision_object_LIDAR.header.frame_id = move_group->getPlanningFrame();
    collision_object_LIDAR.id = "LIDAR";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 0.073;
    primitive.dimensions[1] = 0.05;
    object_pose.position.x=-0.34;
    collision_object_LIDAR.primitives.push_back(primitive);
    collision_object_LIDAR.primitive_poses.push_back(object_pose);
    collision_object_LIDAR.operation = collision_object.ADD;
    objects.push_back(collision_object_LIDAR);

    planning_scene_interface->addCollisionObjects(objects);

}
void Manipulator ::addDynamicPlanningConstraint()
{
    Eigen::Affine3d Trans_B2T=getTagPosition(Tags_detected[0].Trans_C2T);
    std::vector<std::string> object_names;
    object_names=planning_scene_interface->getKnownObjectNames();
    if(!object_names.empty())
    {
        for(auto & object_name : object_names)
        {
            if(object_name!=TREE_NAME)
            {
                object_names.erase(std::remove(std::begin(object_names),std::end(object_names),object_name),\
                std::end(object_names));
            }
        }
        planning_scene_interface->removeCollisionObjects(object_names);
    }
    std::vector<moveit_msgs::CollisionObject> objects;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group->getPlanningFrame();
    collision_object.id = TREE_NAME;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 5;
    primitive.dimensions[1] = radius;
    geometry_msgs::Pose object_pose;
    object_pose.position.x=Trans_B2T.translation()[0];
    object_pose.position.y=Trans_B2T.translation()[1]+(radius+0.15)*boost::math::sign(Trans_B2T.translation()[1]);
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;
    objects.push_back(collision_object);

    planning_scene_interface->addCollisionObjects(objects);
}
void Manipulator::addChargerDynamicPlanningConstraint()
{
    Eigen::Affine3d Trans_B2T=getTagPosition(Tags_detected[0].Trans_C2T);
    std::vector<std::string> object_names;
    object_names=planning_scene_interface->getKnownObjectNames();
    if(!object_names.empty())
    {
        for(auto & object_name : object_names)
        {
            if(object_name!="CHARGER")
            {
                object_names.erase(std::remove(std::begin(object_names),std::end(object_names),object_name),\
                std::end(object_names));
            }
        }
        planning_scene_interface->removeCollisionObjects(object_names);
    }
    std::vector<moveit_msgs::CollisionObject> objects;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group->getPlanningFrame();
    collision_object.id = "CHARGER";
    shape_msgs::Plane plane;
    plane.coef={0,-1,0,boost::math::sign(Trans_B2T.translation()[1])*(abs(Trans_B2T.translation()[1])-0.03)};
    geometry_msgs::Pose object_pose;
    object_pose.position.x=0;
    object_pose.position.y=0;
    object_pose.position.z=0.0;
    object_pose.orientation.w=1.0;
    collision_object.planes.push_back(plane);
    collision_object.plane_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;
    objects.push_back(collision_object);

    planning_scene_interface->addCollisionObjects(objects);
}
bool Manipulator::goUp(double velocity_scale,bool reset)
{
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    if(inverse)
    {
        std::vector<double> goal;
        goal={M_PI,0,0,0,0,0};
        move_group->setJointValueTarget(goal);
        return move_group->move()==moveit::planning_interface::MoveItErrorCode::SUCCESS;
    }
    else
    {
        if(!reset)
        {
            std::vector<double> goal;
            goal={M_PI,0,0,0,0,0};
            move_group->setJointValueTarget(goal);
            move_group->move();
        }
        move_group->setNamedTarget("up");
        return move_group->move()==moveit::planning_interface::MoveItErrorCode::SUCCESS;
    }
}
void Manipulator::goHome(double velocity_scale)
{
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    std::vector<double> goal;
    goal={2.18967866898,0.000737879949156,-2.65072178841,-2.3498339653,-0.0311048775911,0.000869322626386};
    move_group->setJointValueTarget(goal);
    move_group->move();
}
double Manipulator::allClose(std::vector<double> goal)
{
    std::vector<double> current_joints;
    current_joints=move_group->getCurrentJointValues();
    std::vector<double>	auxiliary;

    std::transform (current_joints.begin(), current_joints.end(), goal.begin(), std::back_inserter(auxiliary),//
                    [](double element1, double element2) {return pow((element1-element2),2);});
    auxiliary.shrink_to_fit();
    return sqrt(std::accumulate(auxiliary.begin(), auxiliary.end(), 0.0));
}
Eigen::Affine3d Manipulator::getEndMotion(int search_type, Eigen::Vector3d Translation, Eigen::Vector3d RPY)
{
    Eigen::Affine3d CameraMotion,Trans_B2E;
    CameraMotion.translation()=Translation;
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix=Eigen::AngleAxisd(RPY[2], Eigen::Vector3d::UnitZ())*
                    Eigen::AngleAxisd(RPY[1], Eigen::Vector3d::UnitY())*
                    Eigen::AngleAxisd(RPY[0], Eigen::Vector3d::UnitX());
    CameraMotion.linear()=rotation_matrix;
    current_pose=move_group->getCurrentPose();
    Eigen::fromMsg(current_pose.pose,Trans_B2E);

    switch (search_type)
    {
        case UP:
            return Trans_B2E*CameraMotion;
        case DOWN:
            return CameraMotion*Trans_B2E;
        default:
            return Trans_B2E*CameraMotion;
    }
}
bool Manipulator::goSearchOnce(int search_type, double velocity_scale,double search_angle)
{
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    std::vector<double> goal;
    switch (search_type)
    {
        case FLAT:
            goal = move_group->getCurrentJointValues();
            goal[4]-=search_angle;
            move_group->setJointValueTarget(goal);
            if(move_group->move()!=moveit_msgs::MoveItErrorCodes::SUCCESS)
                return false;
            goal[4]+=2*search_angle;
            break;
        case DOWN:
            move_group->setPoseTarget(getEndMotion(DOWN, Eigen::Vector3d(0.0, 0.0, -searchDownHeight)));
            if(move_group->move()!=moveit_msgs::MoveItErrorCodes::SUCCESS)
                return false;
            goal=move_group->getCurrentJointValues();
            goal[4]-=2*search_angle;
            break;
        case UP:
            move_group->setPoseTarget(getEndMotion(UP, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(M_PI / 6, 0, 0)));
            if(move_group->move()!=moveit_msgs::MoveItErrorCodes::SUCCESS)
                return false;
            goal=move_group->getCurrentJointValues();
            goal[4]+=2*search_angle;
            break;
        default:
            ROS_ERROR("Wrong search type!!!!");
            return false;
    }
    move_group->setJointValueTarget(goal);
    move_group->setMaxVelocityScalingFactor(0.2);
    if(move_group->asyncMove()!=moveit_msgs::MoveItErrorCodes::SUCCESS)
        return false;
    int counter=0;
    while(RobotAllRight&&(Tags_detected.empty()||counter<10))
    {
        //ROS_INFO("Searching tag!!!!!");
        if(!Tags_detected.empty())
            counter++;
        if(allClose(goal)<=goal_tolerance*2)
            return false;
    }
    //a real aubo robot can't use the stop function
    //move_group->stop();
    move_group->rememberJointValues("tag");
    while(allClose(goal)>goal_tolerance*2);
    sleep(1);
    move_group->setMaxVelocityScalingFactor(1.0);
    move_group->setNamedTarget("tag");
    return (move_group->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
}
bool Manipulator::goSearch(double velocity_scale,double search_angle)
{
    return goSearchOnce(FLAT, velocity_scale,search_angle)||
           goSearchOnce(DOWN, velocity_scale,search_angle);// ||
    //goSearchOnce(UP, velocity_scale,search_angle);
}
bool Manipulator::goServo( double velocity_scale)
{
    current_pose=move_group->getCurrentPose();
    Eigen::fromMsg(current_pose.pose,Trans_W2E);

    double error=100.0;
    Destination_t EndDestination;

    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);

    while(RobotAllRight&&error>servoTolerance)
    {
        if(!Tags_detected.empty()|| goSearch(0.2))
        {
            EndDestination=astra->GetCameraDestination(Tags_detected[0].Trans_C2T,Trans_E2C,ExpectMatrix);
            error=EndDestination.error;
            if(servoPoseOn)
            {
                if(error>0.002)
                    EndDestination.EE_Motion.linear()=Eigen::Matrix3d::Identity();
            }
            else
                EndDestination.EE_Motion.linear()=Eigen::Matrix3d::Identity();
            Trans_W2EP=Trans_W2E*EndDestination.EE_Motion;
            std::cout<<"check the traget position"<<Trans_W2EP.matrix()<<std::endl;
            move_group->setPoseTarget(Trans_W2EP);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (success)
            {
                ROS_INFO("Reachable");
                if(move_group->execute(my_plan)!=moveit::planning_interface::MoveItErrorCode::SUCCESS)
                    return false;
            }
            else
            {
                ROS_ERROR("Failed");
                return false;
            }
            current_pose=move_group->getCurrentPose();
            Eigen::fromMsg(current_pose.pose,Trans_W2E);
        }
        else
            return false;
    }
    return RobotAllRight;
}
bool Manipulator::goCut(Eigen::Affine3d referTag,double velocity_scale)
{
    Eigen::Affine3d Trans_B2E,Trans_B2EP;
    geometry_msgs::Pose target_pose3 = move_group->getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints;
    Eigen::Quaterniond q;
    //调整方位
    double init_theta=asin((target_pose3.position.x-referTag.translation()[0])/(radius+0.02));
    q=Eigen::Matrix3d::Identity()*Eigen::AngleAxisd(-init_theta,Eigen::Vector3d::UnitZ());
    target_pose3.orientation.x=q.x();
    target_pose3.orientation.y=q.y();
    target_pose3.orientation.z=q.z();
    target_pose3.orientation.w=q.w();
    move_group->setPoseTarget(target_pose3);
    if(move_group->move()!=moveit::planning_interface::MoveItErrorCode::SUCCESS)
        return false;
    //进刀
    target_pose3 = move_group->getCurrentPose().pose;
    Eigen::fromMsg(target_pose3,Trans_B2E);
    Trans_B2EP.linear()=Eigen::Matrix3d::Identity();
    Trans_B2EP.translation()=Eigen::Vector3d(0.0,boost::math::sign(target_pose3.position.y)*0.02,0.0);
    Trans_B2E=Trans_B2E*Trans_B2EP;
    move_group->setPoseTarget(Trans_B2E);
    if(move_group->move()!=moveit::planning_interface::MoveItErrorCode::SUCCESS)
        return false;
    target_pose3 = move_group->getCurrentPose().pose;
    Eigen::fromMsg(target_pose3,Trans_B2E);
    Trans_B2EP.linear()=Eigen::Matrix3d::Identity();
    Trans_B2EP.translation()=Eigen::Vector3d(0.0,0.0,-0.002);
    Trans_B2E=Trans_B2E*Trans_B2EP;
    move_group->setPoseTarget(Trans_B2E);
    if(move_group->move()!=moveit::planning_interface::MoveItErrorCode::SUCCESS)
        return false;
    //切割轨迹
    target_pose3 = move_group->getCurrentPose().pose;
    double init_th=asin((target_pose3.position.x-referTag.translation()[0])/radius);
    double init_x=target_pose3.position.y+radius*cos(init_th)*boost::math::sign(target_pose3.position.y);
    for(double th=traceAngle/(double) traceNumber;th<=traceAngle;th+=traceAngle/(double)traceNumber)
    {
        target_pose3.position.x = referTag.translation()[0]+radius*sin(init_th-th);
        target_pose3.position.y = init_x+radius*cos(init_th-th) ;
        target_pose3.position.z +=traceDistance/(double)traceNumber;
        q=Eigen::Matrix3d::Identity()*Eigen::AngleAxisd(-(init_theta-th),Eigen::Vector3d::UnitZ());
        target_pose3.orientation.x=q.x();
        target_pose3.orientation.y=q.y();
        target_pose3.orientation.z=q.z();
        target_pose3.orientation.w=q.w();
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
    {
        ROS_INFO("NO Trajectory");
        return false;
    }
    else
    {
        my_plan.trajectory_=trajectory;
        move_group->execute(my_plan);
    }
    //退刀
    target_pose3 = move_group->getCurrentPose().pose;
    Eigen::fromMsg(target_pose3,Trans_B2E);
    Trans_B2EP.linear()=Eigen::Matrix3d::Identity();
    Trans_B2EP.translation()=Eigen::Vector3d(0.0,0.0,0.002);
    Trans_B2E=Trans_B2E*Trans_B2EP;
    move_group->setPoseTarget(Trans_B2E);
    if(move_group->move()!=moveit::planning_interface::MoveItErrorCode::SUCCESS)
        return false;
    target_pose3 = move_group->getCurrentPose().pose;
    Eigen::fromMsg(target_pose3,Trans_B2E);
    Trans_B2EP.linear()=Eigen::Matrix3d::Identity();
    Trans_B2EP.translation()=Eigen::Vector3d(-0.1,-boost::math::sign(target_pose3.position.y)*0.2,-0.005);
    Trans_B2E=Trans_B2E*Trans_B2EP;
    move_group->setPoseTarget(Trans_B2E);
    return !(move_group->move() != moveit::planning_interface::MoveItErrorCode::SUCCESS);
}
void Manipulator::goZero(double velocity_scale)
{
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    std::vector<double> current_joints;
    current_joints=move_group->getCurrentJointValues();
    current_joints[5]=0.0;
    move_group->setJointValueTarget(current_joints);
    move_group->move();
}
bool Manipulator::goCamera(Eigen::Vector3d target_array, double velocity_scale)
{
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    current_pose=move_group->getCurrentPose();
    Eigen::fromMsg(current_pose.pose,Trans_W2E);

    Trans_W2EP.translation()=Trans_E2C*target_array;
    Trans_W2EP.linear()=Eigen::Matrix3d::Identity();
    Trans_W2EP=Trans_W2E*Trans_W2EP;
    move_group->setPoseTarget(Trans_W2EP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if(move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Naive Reachable");
        return move_group->execute(my_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS;
    }
    else
    {
        ROS_INFO("FAILED");
        return false;
    }
}
Eigen::Affine3d Manipulator::getTagPosition(Eigen::Affine3d Trans_C2T)
{
    Eigen::Affine3d Trans_B2E;
    Eigen::fromMsg(move_group->getCurrentPose().pose,Trans_B2E);
    return Trans_B2E*Trans_E2C*Trans_C2T;
}
bool Manipulator::goCharge(double velocity_scale)
{
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    current_pose=move_group->getCurrentPose();
    Eigen::fromMsg(current_pose.pose,Trans_W2E);
    Eigen::Affine3d Trans_B2T=Trans_W2E*Trans_E2C*Tags_detected[0].Trans_C2T;
    Trans_B2T.translation() += Eigen::Vector3d(0.0,0.037,-0.250);
    Eigen::Matrix3d rotation_matrix;
    Eigen::Affine3d tempMatrix;
    rotation_matrix=Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitY())*
                    Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX());
    tempMatrix.linear()=rotation_matrix;
    tempMatrix.translation()=Eigen::Vector3d(0.0,0.0,0.0);
    Trans_W2EP=Trans_B2T*tempMatrix*Trans_E2CH.inverse();
    move_group->setPoseTarget(Trans_W2EP);
    move_group->move();
    current_pose=move_group->getCurrentPose();
    Eigen::fromMsg(current_pose.pose,Trans_W2E);
    Trans_W2EP.linear()=Eigen::Matrix3d::Identity();
    Trans_W2EP.translation()=Eigen::Vector3d(0.020,0.00,0.00);
    Trans_W2E=Trans_W2E*Trans_W2EP;
    move_group->setPoseTarget(Trans_W2E);
    return !(move_group->move() != moveit::planning_interface::MoveItErrorCode::SUCCESS);
}
bool Manipulator::leaveCharge(double velocity_scale)
{
    current_pose=move_group->getCurrentPose();
    Eigen::fromMsg(current_pose.pose,Trans_W2E);
    Trans_W2EP.linear()=Eigen::Matrix3d::Identity();
    Trans_W2EP.translation()=Eigen::Vector3d(-0.10,0.00,0.00);
    Trans_W2E=Trans_W2E*Trans_W2EP;
    move_group->setPoseTarget(Trans_W2E);
    move_group->move();
    move_group->setNamedTarget("up");
    move_group->move();
}
int Manipulator::executeService(int serviceType)
{
    int serviceStatus;
    switch (serviceType)
    {
        case visual_servo::manipulate::Request::CUT:
            if(Tags_detected.empty())
            {
                ROS_ERROR("CHECK CHECK TAGS EMPTY");
                if(!goSearch(0.2))
                {
                    ROS_INFO("!!!!!!NO TAG SEARCHED!!!!!");
                    serviceStatus=visual_servo_namespace::SERVICE_STATUS_NO_TAG;
                    goUp(basicVelocity);
                    goHome(basicVelocity);
                }
                else
                {
                    addDynamicPlanningConstraint();
                    bool servo_success=goServo(basicVelocity);
                    if(!servo_success)
                        serviceStatus=visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED;
                    else
                    {
                        //robot_manipulator.goZero();
                        Eigen::Vector3d Center3d=Tags_detected[0].Trans_C2T.translation();
                        Eigen::Affine3d tagTransform=getTagPosition(Tags_detected[0].Trans_C2T);
                        Center3d[1]+=cameraXYZ[1];
                        Center3d[0]+=cameraXYZ[0];
                        Center3d[2]+=cameraXYZ[2];
                        if(!goCamera(Center3d,basicVelocity))
                            serviceStatus=visual_servo_namespace::SERVICE_STATUS_CLOSE_FAILED;
                        else
                        {
                            if(!goCut(tagTransform,basicVelocity/10.0))
                                serviceStatus=visual_servo_namespace::SERVICE_STATUS_CUT_FAILED;
                        }
                    }
                    goUp(basicVelocity);
                    goHome(basicVelocity);
                    serviceStatus=visual_servo_namespace::SERVICE_STATUS_SUCCEED;
                }

            }
            else
            {   addDynamicPlanningConstraint();
                bool servo_success=goServo(basicVelocity);
                if(!servo_success)
                    serviceStatus=visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED;
                else
                {
                    //robot_manipulator.goZero();
                    Eigen::Vector3d Center3d=Tags_detected[0].Trans_C2T.translation();
                    Eigen::Affine3d tagTransform=getTagPosition(Tags_detected[0].Trans_C2T);
                    Center3d[1]+=cameraXYZ[1];
                    Center3d[0]+=cameraXYZ[0];
                    Center3d[2]+=cameraXYZ[2];
                    if(!goCamera(Center3d,basicVelocity))
                        serviceStatus=visual_servo_namespace::SERVICE_STATUS_CLOSE_FAILED;
                    else
                    {
                        if(!goCut(tagTransform,basicVelocity/10.0))
                            serviceStatus=visual_servo_namespace::SERVICE_STATUS_CUT_FAILED;
                    }
                }
                goUp(basicVelocity);
                goHome(basicVelocity);
                serviceStatus=visual_servo_namespace::SERVICE_STATUS_SUCCEED;
            }
            break;
        case visual_servo::manipulate::Request::SEARCH:
            serviceStatus  = goSearch(basicVelocity) ?
                             visual_servo_namespace::SERVICE_STATUS_SUCCEED :
                             visual_servo_namespace::SERVICE_STATUS_NO_TAG;
            break;
        case visual_servo::manipulate::Request::CHARGE:
            if(Tags_detected.empty())
            {
                ROS_ERROR("CHECK CHECK TAGS EMPTY");
                if(!goSearch(0.2))
                {
                    ROS_INFO("!!!!!!NO TAG SEARCHED!!!!!");
                    serviceStatus=visual_servo_namespace::SERVICE_STATUS_NO_TAG;
                    goUp(basicVelocity);
                    goHome(basicVelocity);
                }
                else
                {
                    //addDynamicPlanningConstraint();
                    addChargerDynamicPlanningConstraint();
                    bool servo_success=goServo(basicVelocity);
                    if(!servo_success)
                        serviceStatus=visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED;
                    else
                    {
                        if(!goCharge(basicVelocity))
                            serviceStatus=visual_servo_namespace::SERVICE_STATUS_CHARGE_FAILED;
                        else
                        {
                            sleep(10);
                            leaveCharge(basicVelocity);
                        }
                    }
                    goHome(basicVelocity);
                    serviceStatus=visual_servo_namespace::SERVICE_STATUS_SUCCEED;
                }
            }
            else
            {
                addChargerDynamicPlanningConstraint();
                bool servo_success=goServo(basicVelocity);
                if(!servo_success)
                    serviceStatus=visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED;
                else
                {
                    if(!goCharge(basicVelocity))
                        serviceStatus=visual_servo_namespace::SERVICE_STATUS_CHARGE_FAILED;
                    else
                    {
                        sleep(10);
                        leaveCharge(basicVelocity);
                    }
                }
                goHome(basicVelocity);
                serviceStatus=visual_servo_namespace::SERVICE_STATUS_SUCCEED;
            }
            break;
        default:
            serviceStatus=visual_servo_namespace::SERVICE_STATUS_EMPTY;
            break;
    }
    if(!RobotAllRight)
        serviceStatus=visual_servo_namespace::SERVICE_STATUS_ROBOT_ABORT;
    return serviceStatus;
}

Listener::Listener()
{
    this->tag_info_received=false;
    tag_info_sub =nh.subscribe("TagsDetected",1000, &Listener::tagInfoCallback,this);
    vs_status_sub = nh.subscribe("VisualServoStatus",100,&Listener::statusCallback,this);
    //tf_sub = nh.subscribe("/tf_static",1000,&Listener::tfCallback,this);
    client =nh.serviceClient<visual_servo::detect_once>("detect_once");
    srv.request.ready_go=(int)detect_srv_on;
    manipulate_callback_t manipulate_callback =
            boost::bind(&Listener::manipulate, this, _1, _2);
    service_ = nh.advertiseService("manipulate",manipulate_callback);
    tfListener = new tf2_ros::TransformListener(tfBuffer);
}
Listener::~Listener()
{
    delete tfListener;
}

void Listener::tagInfoCallback(const visual_servo::TagsDetection_msg &msg)
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
void Listener::tfCallback(const tf2_msgs::TFMessage &msg)
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
void Listener::statusCallback(const visual_servo::VisualServoMetaTypeMsg &msg)
{
    RobotAllRight=msg.RobotAllRight;
    //std::cout<<"RobotAllRight: "<<(int) RobotAllRight<<"RobotSwitchOn: "<< (int) RobotSwitchOn<<std::endl;
}
void Listener::calibrateInfoPublish()
{
    //geometry_msgs::Transform world_effector,camera_object;
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("ee_link", "camera_color_optical_frame",
                                                    ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        //ros::Duration(1.0).sleep();
    }
    Eigen::Affine3d Trans_E2C=tf2::transformToEigen(transformStamped.transform);
    std::cout<<"try"<<Trans_E2C.matrix()<<std::endl;
}
bool Listener::callSrv()
{
    if (client.call(srv))
    {
        Eigen::Vector3d eigen_point;
        for(auto & point : srv.response.knife_trace)
        {
            eigen_point[0]=point.x;
            eigen_point[1]=point.y;
            eigen_point[2]=point.z;
            knife_trace.push_back(eigen_point);
        }
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service detect_once");
        return false;
    }
}
bool Listener::manipulate(visual_servo::manipulate::Request &req,
                          visual_servo::manipulate::Response &res)
{
    manipulate_srv_on=true;
    ROS_INFO("CHECK CHECK I'M HERE");
    ManipulateSrv.srv_type=req.type;
    ros::Time temp;
    while(manipulate_srv_on)
    {
        temp=ros::Time::now();
    }
    res.status=ManipulateSrv.srv_status;
    ManipulateSrv.srv_status=visual_servo_namespace::SERVICE_STATUS_EMPTY;
    ManipulateSrv.srv_type=visual_servo::manipulate::Request::EMPTY;
    return true;
}
/*
 * 0.949088096619 0.0506779626012 2.77839660645 0.261449694633 0.031603731215 -0.00739841256291
 * joint state when car is moving
 */