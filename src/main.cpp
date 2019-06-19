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

#include "visual_servo/TagDetection_msg.h"
#include "visual_servo/TagsDetection_msg.h"
#include "visual_servo/detect_once.h"
#include "visual_servo/manipulate.h"


#define _USE_MATH_DEFINES

/*Global variables action in multi threads
 * @param Tags_detected     [Tags information received from camera thread, vector is empty when no tags were detected]
 * @param detect_srv_on     [The knife trace detect service trigger]
 * @param knife_trace       [The knife trace returned by trace detect service]
 * @param manipulate_srv_on [The manipulation service trigger]
 */
std::vector<TagDetectInfo> Tags_detected;
bool detect_srv_on=false;
bool manipulate_srv_on=false;
std::vector<Eigen::Vector3d> knife_trace;

struct manipulateSrv{
    int srv_type;
    int srv_status;
}ManipulateSrv;

void *camera_thread(void *data);

class Manipulator
{
private:
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

    //the planning_group name specified by the moveit_config package
    const std::string PLANNING_GROUP = "manipulator_i5";

    //the name of rubber tree
    const std::string TREE_NAME = "Heava";

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
    Eigen::Affine3d Trans_W2E,Trans_W2EP,Trans_E2C;

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

public:
    //the radius of rubber tree, used as a reference for constructing planning constraints,and generating the cut locus.
    double radius=0.1;

    Manipulator();

    virtual ~Manipulator();

    // Function aims at adding static virtual walls to restrict planning
    void addStaticPlanningConstraint();
    // Function aims at adding dynamic virtual cylinder as tree to restrict planning
    void addDynamicPlanningConstraint();
    // Function makes all robot joints go to zero
    void goUp(double velocity_scale = 0.5);
    // Function makes robot go to a preset position
    void goHome(double velocity_scale = 0.5);
    /* Function makes robot go search once with a defined search angle range
     * @param search_type       [define the search type,UP for looking up,DOWN for looking down,
     *                          FLAT for looking flat,DEFAULT for doing nothing]
     * @param search_angle      [define the search angle range 0-pi]
     * @param velocity_scale    [velocity scaling factor 0-1]
     * @return true if any tag searched else false
     */
    bool goSearchOnce(int search_type, double search_angle = M_PI / 3, double velocity_scale = 0.5);
    //Function makes robot search all three types
    bool goSearch(double search_angle = M_PI / 3, double goal_tolerance = 0.5);
    /*Function makes robot camera servo to pointed pose or position
     * @param ExpectMatrix  [the desired pose of camera describing as transform matrix]
     * @param enable_pose   [true for enable pose servo mode which costs more adjust and may makes end effector rotate,
     *                      false for position servo only]
     * @return true if achieved the desired pose or position
     */
    bool goServo(Eigen::Affine3d expect_matrix, bool enable_pose = false, double velocity_scale = 0.5);
    //Function makes robot accomplish cut task
    void goCut(double velocity_scale = 0.05);
    //Function makes end effector joint goes to zero position.
    void goZero(double velocity_scale = 1.0);
    /*Function makes the end effector goes to a position defined by a array with respect to camera
     * @param target_array  [describes a position with respect to camera]
     * @return true if achieved the desired position.
     */
    bool goCamera(Eigen::Vector3d target_array, double velocity_scale = 0.1);

    //debug preserved functions
    void goTest(double goal_tolerance);
    void goCut1(double velocity_scale = 0.1);
    void goCut2(double velocity_scale = 0.1);
};

//all ros topics and services related functions
class Listener
{
private:
    ros::NodeHandle nh;
    ros::ServiceClient client;
    ros::Subscriber tag_info_sub;
    ros::Subscriber tf_sub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    ros::ServiceServer service_;
    visual_servo::detect_once srv;
    void tagInfoCallback(const visual_servo::TagsDetection_msg &msg);
    void tfCallback(const tf2_msgs::TFMessage &msg);
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

    Eigen::Affine3d ExpectMatrix;
    ExpectMatrix.matrix() << 1,0,0,0,
            0,1,0,0,
            0,0,1,0.30,
            0,0,0,1;
    pthread_t camera_id;
    ros::init(argc, argv, "visual_servo");
    ros::NodeHandle n;

    pthread_create(&camera_id, NULL, camera_thread, NULL);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    Manipulator robot_manipulator;

    robot_manipulator.addStaticPlanningConstraint();
    robot_manipulator.goHome(1.0);
    while(ros::ok())
    {
        if(manipulate_srv_on)
        {
            switch (ManipulateSrv.srv_type)
            {
                case visual_servo::manipulate::Request::CUT:
                    /*
                    if(Tags_detected.empty())
                    {
                        if(robot_manipulator.goSearch())
                        {
                            robot_manipulator.addDynamicPlanningConstraint();
                            bool servo_success=robot_manipulator.goServo(ExpectMatrix);
                            if(servo_success)
                            {
                                robot_manipulator.goZero();
                                robot_manipulator.goCut();
                            }
                            robot_manipulator.goUp();
                            ManipulateSrv.srv_status=visual_servo::manipulate::Response::SUCCESS;
                        }
                        else
                        {
                            ROS_INFO("!!!!!!NO TAG SEARCHED!!!!!");
                            /*
                             * PRESERVE SPACE FOR ACT WITH NAVIGATION;
                            /
                        }

                    }
                    else
                    {   robot_manipulator.addDynamicPlanningConstraint();
                        ROS_INFO("ANOTHER ONE");
                        bool servo_success = robot_manipulator.goServo(ExpectMatrix,0.5);
                        if (servo_success)
                        {
                            robot_manipulator.goZero();
                            //robot_manipulator.goCut();

                            Eigen::Vector3d Center3d=Tags_detected[0].Trans_C2T.translation();
                            Center3d[2]+=0.029;
                            Center3d[1]+=0.15;
                            robot_manipulator.goCamera(Center3d,0.5);
                            sleep(2);
                            robot_manipulator.goCut1();
                            sleep(2);
                            robot_manipulator.goCut2(0.005);
                            /*ready_go=true;
                            while(ready_go);
                            robot_manipulator.goCamera(BeginPoint,0.1);
                            BeginPoint=Eigen::Vector3d(0.0,0.0,0.0);
                             /
                        }/
                        //robot_manipulator.goUp();
                        robot_manipulator.goHome();
                        ManipulateSrv.srv_status=visual_servo::manipulate::Response::SUCCESS;
                    }*/
                    robot_manipulator.goHome(1.0);
                    robot_manipulator.goUp();
                    ManipulateSrv.srv_status=visual_servo::manipulate::Response::SUCCESS;
                    break;
                case visual_servo::manipulate::Request::SEARCH:
                    if(robot_manipulator.goSearch())
                        ManipulateSrv.srv_status=visual_servo::manipulate::Response::SUCCESS;
                    else
                    {
                        ROS_INFO("CHECK HERE");
                        ManipulateSrv.srv_status=visual_servo::manipulate::Response::ERROR;
                    }
                    robot_manipulator.goHome();

                    break;
                default:
                    ManipulateSrv.srv_status=visual_servo::manipulate::Response::EMPTY;
                    break;
            }
            manipulate_srv_on=false;
        }
    }

    ros::waitForShutdown();
    return 0;

}

void *camera_thread(void *data)
{
    Listener listener;
    ros::AsyncSpinner spinner(3);
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
    goal_tolerance=0.001;
    Trans_E2C.matrix()<<    -0.0102032,0.00213213,-0.999946,0.109,
                            0.999923,  0.00702205,-0.010188,0.184,
                            0.00699994,-0.999973, -0.00220362,0.065,
                            0,        0,           0,           1;
    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    planning_scene_interface = new  moveit::planning_interface::PlanningSceneInterface;
    joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    astra =new Servo;

    ROS_INFO_NAMED("Visual Servo", "Reference frame: %s", move_group->getPlanningFrame().c_str());
    ROS_INFO_NAMED("Visual Servo", "End effector link: %s", move_group->getEndEffectorLink().c_str());
}
Manipulator::~Manipulator()
{
    delete move_group;
    delete planning_scene_interface;
    delete astra;
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
    moveit_msgs::CollisionObject collision_object2;
    collision_object2.header.frame_id = move_group->getPlanningFrame();
    collision_object2.id = "virtual_wall";
    plane.coef={1,0,0,-0.8};
    collision_object2.planes.push_back(plane);
    collision_object2.plane_poses.push_back(object_pose);
    collision_object2.operation = collision_object2.ADD;
    objects.push_back(collision_object2);

    planning_scene_interface->addCollisionObjects(objects);
}
void Manipulator ::addDynamicPlanningConstraint()
{
    Eigen::Affine3d Trans_B2E,Trans_B2T;
    current_pose=move_group->getCurrentPose();
    Eigen::fromMsg(current_pose.pose,Trans_B2E);
    Trans_B2T=Trans_B2E*Trans_E2C*Tags_detected[0].Trans_C2T;

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
    object_pose.position.y=Trans_B2T.translation()[1]+(radius+0.030)*boost::math::sign(Trans_B2T.translation()[1]);
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;
    objects.push_back(collision_object);

    planning_scene_interface->addCollisionObjects(objects);
}
void Manipulator::goUp(double velocity_scale)
{
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    move_group->setNamedTarget("up");
    move_group->move();
}
void Manipulator::goHome(double velocity_scale)
{
    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    std::vector<double> goal;
    goal={-0.751516878605,0.599489152431,1.09664881229,0.497168213129,2.3225440979,0.0022961855866};
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
bool Manipulator::goSearchOnce(int search_type, double search_angle, double velocity_scale)
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
            move_group->setPoseTarget(getEndMotion(DOWN, Eigen::Vector3d(0.0, 0.0, -0.10)));
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
    move_group->setMaxVelocityScalingFactor(0.1);
    if(move_group->asyncMove()!=moveit_msgs::MoveItErrorCodes::SUCCESS)
        return false;
    while(Tags_detected.empty())
    {
        //ROS_INFO("Searching tag!!!!!");
        if(allClose(goal)<=goal_tolerance*2)
            return false;
    }
    //a real aubo robot can't use the stop function
    //move_group->stop();
    move_group->rememberJointValues("tag");
    while(allClose(goal)>goal_tolerance*2);
    sleep(1);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    move_group->setNamedTarget("tag");
    return (move_group->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
}
bool Manipulator::goSearch(double search_angle, double goal_tolerance)
{
    return goSearchOnce(FLAT, search_angle, goal_tolerance)||
            goSearchOnce(DOWN, search_angle, goal_tolerance) ||
            goSearchOnce(UP, search_angle, goal_tolerance);
}
bool Manipulator::goServo(Eigen::Affine3d expect_matrix, bool enable_pose, double velocity_scale)
{
    current_pose=move_group->getCurrentPose();
    Eigen::fromMsg(current_pose.pose,Trans_W2E);

    double error=100.0;
    Destination_t EndDestination;

    move_group->setGoalTolerance(goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);

    while(error>goal_tolerance)
    {
        if(!Tags_detected.empty()|| goSearch())
        {
            EndDestination=astra->GetCameraDestination(Tags_detected[0].Trans_C2T,Trans_E2C,expect_matrix);
            error=EndDestination.error;
            if(enable_pose)
            {
                if(error>goal_tolerance*2)
                    EndDestination.EE_Motion.linear()=Eigen::Matrix3d::Identity();
            }
            else
                EndDestination.EE_Motion.linear()=Eigen::Matrix3d::Identity();

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
        else
            return false;
    }
    return true;
}
void Manipulator::goCut(double velocity_scale)
{
    move_group->setPoseTarget(getEndMotion(DEFAULT, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, M_PI / 2)));
    move_group->move();
    geometry_msgs::Pose target_pose3 = move_group->getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints;

    double init_th=asin(target_pose3.position.x/radius);
    double init_y=target_pose3.position.y-radius*cos(init_th);

    Eigen::Affine3d Trans_B2E;
    Eigen::fromMsg(target_pose3,Trans_B2E);
    Eigen::Quaterniond q;
    double init_beta=Trans_B2E.linear().eulerAngles(2,1,0)[0];

    for(double th=0.0;th<M_PI/3;th+=0.08)
    {
        target_pose3.position.x = radius*sin(init_th+th);
        target_pose3.position.y = init_y+radius*cos(init_th+th) ;
        target_pose3.position.z -=0.01;
        init_beta -=0.08;
        q=(Trans_B2E*Eigen::AngleAxisd(init_beta,Eigen::Vector3d::UnitY())).linear();
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
        ROS_INFO("NO Trajectory");
    else
    {
        my_plan.trajectory_=trajectory;
        move_group->execute(my_plan);
    }
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
        move_group->execute(my_plan);
        return true;
    }
    else
    {
        ROS_INFO("FAILED");
        return false;
    }
}
void Manipulator::goTest(double goal_tolerance)
{
    Eigen::Vector3d x_previous,x_now;
    Eigen::Vector3d ea(-1.572, -0.007, -3.132);
    move_group->setGoalTolerance(goal_tolerance);
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
void Manipulator::goCut1(double velocity_scale)
{
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    current_pose=move_group->getCurrentPose();
    current_pose.pose.position.y-=0.003;
    move_group->setPoseTarget(current_pose);
    move_group->move();
}
void Manipulator::goCut2(double velocity_scale)
{
    current_pose=move_group->getCurrentPose();
    current_pose.pose.position.z-=0.05;
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    move_group->setPoseTarget(current_pose);
    move_group->move();
}

Listener::Listener()
{
    this->tag_info_received=false;
    tag_info_sub =nh.subscribe("TagsDetected",1000, &Listener::tagInfoCallback,this);
    //tf_sub = nh.subscribe("/tf_static",1000,&Listener::tfCallback,this);
    client =nh.serviceClient<visual_servo::detect_once>("detect_once");
    srv.request.ready_go=(int)detect_srv_on;
    manipulate_callback_t manipulate_callback =
            boost::bind(&Listener::manipulate, this, _1, _2);
    service_ = nh.advertiseService("manipulate",manipulate_callback);
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
void Listener::calibrateInfoPublish()
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
    ManipulateSrv.srv_type=req.type;
    while(manipulate_srv_on);
    res.status=ManipulateSrv.srv_status;
    ManipulateSrv.srv_status=visual_servo::manipulate::Response::EMPTY;
    ManipulateSrv.srv_type=visual_servo::manipulate::Request::EMPTY;
    return true;
}
/*
 * 0.949088096619 0.0506779626012 2.77839660645 0.261449694633 0.031603731215 -0.00739841256291
 * joint state when car is moving
 */