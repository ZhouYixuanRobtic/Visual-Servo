#include "Servo.h"

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
#include "parameterTeleop.h"
#include <boost/thread/thread.hpp>

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>



#define _USE_MATH_DEFINES

/*Global variables action in multi threads
 * @param Tags_detected     [Tags information received from camera thread, vector is empty when no tags were detected]
 * @param detect_srv_on     [The knife trace detect service trigger]
 * @param knife_trace       [The knife trace returned by trace detect service]
 * @param manipulate_srv_on [The manipulation service trigger]
 * @param RobotAllRight     [The robot status, false for something wrong]
 */
tag_detection_info_t Tags_detected;
bool detect_srv_on=false;
bool manipulate_srv_on=false;
bool RobotAllRight;
std::vector<Eigen::Vector3d> knife_trace;

manipulateSrv ManipulateSrv;

void *camera_thread(void *data);

class Manipulator
{
private:
    ros::NodeHandle n_;
    bool running_;
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

    const std::vector<std::string> PARAMETER_NAMES={"/user/radius","/user/goCameraX","/user/goCameraY",
                                                    "/user/goCameraZ","/user/distance","/user/angle",
                                                    "/user/pointsNumber","/user/inverse"};

    std::string EE_NAME;
    moveit::planning_interface::MoveGroupInterface *move_group;
    moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group;

    bool charging;

    //Servo class for computing the servo related transform matrix
    Servo *astra;

    //parameter listener
    ParameterListener *parameterListener_;
    /*
     * The transform matrix of A with respect to B named as Trans_B2A.
     * W-world,E-end effector,C-camera,EP-desired end effector,B-base_link,T-target
     */
    Eigen::Affine3d Trans_W2E{},Trans_W2EP{},Trans_E2C{},Trans_E2CH{},Trans_B2T_{};

    //The desired camera pose matrix with respect to tag
    Eigen::Affine3d ExpectMatrix;

    /* Function for computing the joint space Euclidean distance,
     * between the given goal and now joint states.
     * @param goal  [the given joint space goal]
     * @return the joint space Euclidean distance as a double number,
     *  Unit-radian
     */
    double allClose(const std::vector<double> & goal) const;
    /* Function for computing the matrix of desired end effector motion.
     * @param search_type   [define the reference coordinate system of desired motion,
     *                      UP for end effector, DOWN for world, DEFAULT for end effector]
     * @param Translation   [motion's translation part]
     * @param RPY           [motion's rotation part described by RPY values]
     * @return the desired end effector motion matrix.
     */
    Eigen::Affine3d getEndMotion(int search_type,
                                 const Eigen::Vector3d & Translation = Eigen::Vector3d(0.0, 0.0, 0.0),
                                 const Eigen::Vector3d & RPY = Eigen::Vector3d(0.0, 0.0, 0.0)) const;
    //read parameters and listen to tf
    void initParameter();

    void scale_trajectory_speed(moveit::planning_interface::MoveGroupInterface::Plan & plan, double scale_factor) const;
    //make the robot move as a line giving the destination
    bool linearMoveTo(const Eigen::Vector3d & destination, double velocity_scale) const;

    //prameters
    struct{
        //the radius of rubber tree, used as a reference for constructing planning constraints,and generating the cut locus.
        double radius;
        Eigen::Vector3d expectRPY;
        Eigen::Vector3d expectXYZ;
        Eigen::Vector3d cameraXYZ;
        bool inverse;
        bool servoPoseOn;
        double searchDownHeight;
        double chargeDownHeight;
        double basicVelocity;
        double servoTolerance;
        int traceNumber;
        double traceDistance;
        double traceAngle;
        double goal_tolerance;
        std::string cameraFrame;
        std::string toolFrame;
    }Parameters;

public:


    Manipulator();

    virtual ~Manipulator();

    void setParametersFromCallback();
    // Function aims at adding static virtual walls to restrict planning
    void addStaticPlanningConstraint() const;
    /* Function aims at adding dynamic virtual cylinder as tree to restrict planning
     * @param leave [true for more restrict when robot try to leave]
     */
    void addDynamicPlanningConstraint(bool leave=false);
    // Function aims at adding dynamic virtual cylinder as charger plane to restrict planning
    void addChargerDynamicPlanningConstraint() const;
    /* Function makes all robot joints go to zero
     * @param reset [true for go up to the other side]
     */
    bool goUp(double velocity_scale ,bool reset=true) const;
    // Function makes robot go to a preset position
    bool goHome(double velocity_scale) const;
    /* Function makes robot go search once with a defined search angle range
     * @param search_type       [define the search type,UP for looking up,DOWN for looking down,
     *                          FLAT for looking flat,DEFAULT for doing nothing]
     * @param search_angle      [define the search angle range 0-pi]
     * @param velocity_scale    [velocity scaling factor 0-1]
     * @return true if any tag searched else false
     */
    bool goSearchOnce(int search_type, double velocity_scale, double search_angle = M_PI / 3.0) const;
    //Function makes robot search all three types
    bool goSearch( double velocity_scale,bool underServoing=false,double search_angle = M_PI / 3.0) const;
    /*Function makes robot camera servo to pointed pose or position
     * @param ExpectMatrix  [the desired pose of camera describing as transform matrix]
     * @param enable_pose   [true for enable pose servo mode which costs more adjust time and may makes end effector rotate,
     *                      false for position servo only]
     * @return true if achieved the desired pose or position
     */
    bool goServo(double velocity_scale );
    //Function makes robot accomplish cut task
    bool goCut(const Eigen::Affine3d &referTag,double velocity_scale );
    //Function makes end effector joint goes to zero position.
    void goZero(double velocity_scale = 1.0);
    /*Function makes the end effector goes to a position defined by a array with respect to camera
     * @param target_array  [describes a position with respect to camera]
     * @return true if achieved the desired position.
     */
    bool goCamera(const Eigen::Vector3d & target_array, double velocity_scale = 0.1);
    //Function makes the charger goes to a fixed position related to the tag
    bool goCharge(double velocity_scale = 0.5);
    //Function makes the charger leave from the station
    bool leaveCharge(double velocity_scale=0.5);
    //Function executes the service
    int executeService(int serviceType);

    void removeAllDynamicConstraint() const;

    //Function computes the tag pose with respect to the planning frame
    Eigen::Affine3d getTagPosition(const Eigen::Affine3d &Trans_C2T) const;
    //debug preserved functions

};

//all ros topics and services related functions
class Listener
{
private:
    const double WATCHDOG_PERIOD_=1.0;
    ros::NodeHandle nh;
    //trace detection service client
    ros::ServiceClient client;
    ros::Subscriber tag_info_sub;
    ros::Subscriber vs_status_sub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    //manipulate service server
    ros::ServiceServer service_;
    visual_servo::detect_once srv;
    void tagInfoCallback(const visual_servo::TagsDetection_msg &msg);
    //arm controller status callback, published by auboRobot
    void statusCallback(const visual_servo::VisualServoMetaTypeMsg &msg);
    bool manipulate(visual_servo::manipulate::Request &req,
                    visual_servo::manipulate::Response &res);
    ros::Timer watchdog_timer_;
    void watchdog(const ros::TimerEvent &e);

public:
    typedef boost::function<bool (visual_servo::manipulate::Request&,visual_servo::manipulate::Response& res)>
            manipulate_callback_t;
    bool tag_info_received;
    Listener();
    virtual ~Listener();
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
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        robot_manipulator.setParametersFromCallback();
        if(manipulate_srv_on)
        {
            if(RobotAllRight)
            {
                ros::param::set("/visual/tagDetectorOn",true);
//                if(!robot_manipulator.goUp(0.2,false))
//                    ManipulateSrv.srv_status=visual_servo_namespace::SERVICE_STATUS_UP_FAILED;
//                else
                ManipulateSrv.srv_status=robot_manipulator.executeService(ManipulateSrv.srv_type);
            }
            else
                ManipulateSrv.srv_status=visual_servo_namespace::SERVICE_STATUS_ROBOT_ABORT;
            //check again
            if(!RobotAllRight)
                ManipulateSrv.srv_status=visual_servo_namespace::SERVICE_STATUS_ROBOT_ABORT;

            robot_manipulator.removeAllDynamicConstraint();
            manipulate_srv_on=false;
            ROS_INFO("Service executed!");
        }
        loop_rate.sleep();
    }
    return 0;
}

void *camera_thread(void *data)
{
    Listener listener;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        if(detect_srv_on)
        {
            detect_srv_on=!listener.callSrv();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}


Manipulator::Manipulator()
{

    move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    planning_scene_interface = new  moveit::planning_interface::PlanningSceneInterface;
    joint_model_group = move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    astra =new Servo;
    parameterListener_ = new ParameterListener(30,8);
    charging=false;
    ROS_INFO_NAMED("Visual Servo", "Reference frame: %s", move_group->getPlanningFrame().c_str());
    ROS_INFO_NAMED("Visual Servo", "End effector link: %s", move_group->getEndEffectorLink().c_str());
    EE_NAME=move_group->getEndEffectorLink().c_str();
    initParameter();

}
Manipulator::~Manipulator()
{
    delete move_group;
    delete planning_scene_interface;
    delete astra;
    delete parameterListener_;
}
void Manipulator::setParametersFromCallback()
{
    if(!running_)
    {
        Parameters.radius = parameterListener_->parameters()[0];
        Parameters.cameraXYZ = Eigen::Vector3d(parameterListener_->parameters()[1], parameterListener_->parameters()[2],
                                               parameterListener_->parameters()[3]);
        Parameters.traceDistance = parameterListener_->parameters()[4];
        Parameters.traceAngle = parameterListener_->parameters()[5];
        Parameters.traceNumber = (int) parameterListener_->parameters()[6];
        Parameters.inverse = (bool) parameterListener_->parameters()[7];
        Parameters.traceAngle=Parameters.traceAngle*M_PI/180.0;
    }
}
void Manipulator::initParameter()
{
    n_.param<double>("/user/radius",Parameters.radius,0.10);
    n_.param<double>("/user/expectRoll",Parameters.expectRPY[0],0.0);
    n_.param<double>("/user/expectPitch",Parameters.expectRPY[1],0.0);
    n_.param<double>("/user/expectYaw",Parameters.expectRPY[2],0.0);
    n_.param<double>("/user/expectX",Parameters.expectXYZ[0],0.0);
    n_.param<double>("/user/expectY",Parameters.expectXYZ[1],0.0);
    n_.param<double>("/user/expectZ",Parameters.expectXYZ[2],0.4);
    n_.param<bool>("/user/inverse",Parameters.inverse,false);
    n_.param<bool>("/user/servoPoseOn",Parameters.servoPoseOn,false);
    n_.param<double>("/user/searchDownHeight",Parameters.searchDownHeight,-0.15);
    n_.param<double>("/user/chargeDownHeight",Parameters.chargeDownHeight,-0.30);
    n_.param<double>("/user/goCameraX",Parameters.cameraXYZ[0],0.0);
    n_.param<double>("/user/goCameraY",Parameters.cameraXYZ[1],0.0);
    n_.param<double>("/user/goCameraZ",Parameters.cameraXYZ[2],-0.2);
    n_.param<double>("/user/angle",Parameters.traceAngle,60.0);
    n_.param<double>("/user/distance",Parameters.traceDistance,-0.08);
    n_.param<int>("/user/pointsNumber",Parameters.traceNumber,20);

    n_.param<double>("/robot/basicVelocity",Parameters.basicVelocity,0.5);
    n_.param<double>("/robot/goalTolerance",Parameters.goal_tolerance,0.001);
    n_.param<double>("/robot/servoTolerance",Parameters.servoTolerance,0.001);
    n_.param<std::string>("/user/cameraFrame",Parameters.cameraFrame,"camera_color_optical_frame");
    n_.param<std::string>("/user/toolFrame",Parameters.toolFrame,"charger_ee_link");
    std::cout<<"read parameter success"<<std::endl;

    Parameters.traceAngle=Parameters.traceAngle*M_PI/180.0;
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix=Eigen::AngleAxisd(Parameters.expectRPY[2], Eigen::Vector3d::UnitZ())*
                    Eigen::AngleAxisd(Parameters.expectRPY[1], Eigen::Vector3d::UnitY())*
                    Eigen::AngleAxisd(Parameters.expectRPY[0], Eigen::Vector3d::UnitX());
    ExpectMatrix.linear()=rotation_matrix;
    ExpectMatrix.translation()=Parameters.expectXYZ;
    astra->getTransform(EE_NAME,Parameters.cameraFrame,Trans_E2C);
    astra->getTransform(EE_NAME,Parameters.toolFrame,Trans_E2CH);
    running_=false;
    RobotAllRight=Parameters.cameraFrame!="camera_color_optical_frame" ?
                  true :
                  false;
    parameterListener_->registerParameterCallback(PARAMETER_NAMES,false);
}
void Manipulator::addStaticPlanningConstraint() const
{
    //in case of repeat usage
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

    //virtual LIDAR
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
void Manipulator ::addDynamicPlanningConstraint(bool leave)
{
    Eigen::Affine3d Trans_B2T;
    if(!leave)
    {
        Trans_B2T=getTagPosition(Tags_detected[0].Trans_C2T);
        Trans_B2T_=Trans_B2T;
    }
    std::vector<std::string> object_names;
    object_names=planning_scene_interface->getKnownObjectNames();
    const std::vector<std::string> tree_name{TREE_NAME};
    if(!object_names.empty())
    {
        for(auto & object_name : object_names)
        {
            if(object_name==TREE_NAME)
            {
                planning_scene_interface->removeCollisionObjects(tree_name);
            }
        }
    }
    std::vector<moveit_msgs::CollisionObject> objects;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group->getPlanningFrame();
    collision_object.id = TREE_NAME;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[0] = 5;
    primitive.dimensions[1] = Parameters.radius;
    geometry_msgs::Pose object_pose;
    object_pose.position.x=Trans_B2T.translation()[0];
    if(!leave)
        object_pose.position.y=Trans_B2T.translation()[1]+(Parameters.radius+0.07)*boost::math::sign(Trans_B2T.translation()[1]);
    else
        object_pose.position.y=Trans_B2T_.translation()[1]+(Parameters.radius-0.09)*boost::math::sign(Trans_B2T_.translation()[1]);
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;
    objects.push_back(collision_object);

    planning_scene_interface->addCollisionObjects(objects);
}
void Manipulator::addChargerDynamicPlanningConstraint() const
{
    Eigen::Affine3d Trans_B2T=getTagPosition(Tags_detected[0].Trans_C2T);
    std::vector<std::string> object_names;
    object_names=planning_scene_interface->getKnownObjectNames();
    const std::vector<std::string> charger_name{"CHARGER"};
    if(!object_names.empty())
    {
        for(auto & object_name : object_names)
        {
            if(object_name=="CHARGER")
            {
                planning_scene_interface->removeCollisionObjects(charger_name);
            }
        }
    }
    std::vector<moveit_msgs::CollisionObject> objects;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group->getPlanningFrame();
    collision_object.id = "CHARGER";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 2;
    primitive.dimensions[1] = 2;
    primitive.dimensions[2] = 0.001;
    geometry_msgs::Pose object_pose;
    object_pose.position.x=0;
    object_pose.position.y=boost::math::sign(Trans_B2T.translation()[1])*(abs(Trans_B2T.translation()[1])+0.03);
    object_pose.position.z=1;
    object_pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(M_PI/2.0,0,0);
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = collision_object.ADD;
    objects.push_back(collision_object);

    planning_scene_interface->addCollisionObjects(objects);
}
void Manipulator::removeAllDynamicConstraint() const
{
    std::vector<std::string> object_names;
    object_names=planning_scene_interface->getKnownObjectNames();
    std::vector<std::string> dynamic_names;
    if(!object_names.empty())
    {
        for(auto & object_name : object_names)
        {
            if(object_name=="CHARGER"||object_name==TREE_NAME)
            {
                dynamic_names.push_back(object_name);
            }
        }
        planning_scene_interface->removeCollisionObjects(dynamic_names);
    }
}
void Manipulator::scale_trajectory_speed(moveit::planning_interface::MoveGroupInterface::Plan &plan,double scale_factor) const
{
    scale_factor = 20*scale_factor;
    for(auto & point : plan.trajectory_.joint_trajectory.points)
    {
        point.time_from_start *=1/scale_factor;
        for(int i=0; i<plan.trajectory_.joint_trajectory.joint_names.size();++i)
        {
            point.velocities[i] *= scale_factor;
            point.accelerations[i] *= scale_factor*scale_factor;
        }
    }
}
bool Manipulator::linearMoveTo(const Eigen::Vector3d &destination_translation, double velocity_scale) const
{
    static double linear_step = 0.01;
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    Eigen::Affine3d linear_start;
    geometry_msgs::Pose linear_pose = move_group->getCurrentPose().pose;
    Eigen::fromMsg(linear_pose,linear_start);
    moveit_msgs::OrientationConstraint pcm;
    pcm.link_name = EE_NAME;
    pcm.header.frame_id = move_group->getPlanningFrame();
    pcm.header.stamp = ros::Time::now();
    pcm.absolute_x_axis_tolerance = 0.001;
    pcm.absolute_y_axis_tolerance = 0.001;
    pcm.absolute_z_axis_tolerance = 0.001;
    pcm.orientation = move_group->getCurrentPose().pose.orientation;
    pcm.weight=1.0;
    moveit_msgs::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(pcm);
    move_group->setPathConstraints(path_constraints);
    linear_start.translation() +=destination_translation;
    move_group->setPoseTarget(linear_start);
    move_group->setPlanningTime(10.0);
    if( move_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        move_group->setPlanningTime(5.0);
        move_group->clearPathConstraints();
        return true;
    }
    else
    {
        move_group->setPlanningTime(5.0);
        move_group->clearPathConstraints();
        return false;
    }
}
bool Manipulator::goUp(double velocity_scale,bool reset) const
{
    move_group->setGoalTolerance(Parameters.goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    std::vector<double> goal;
    bool success=false;
    if(Parameters.inverse)
    {
        move_group->setNamedTarget("up");
        success=move_group->move()==moveit::planning_interface::MoveItErrorCode::SUCCESS;
        if(!reset)
        {
            goal={M_PI,0,0,0,0,0};
            move_group->setJointValueTarget(goal);
            return move_group->move()==moveit::planning_interface::MoveItErrorCode::SUCCESS;
        }
    }
    else
    {
        goal={M_PI,0,0,0,0,0};
        move_group->setJointValueTarget(goal);
        success=move_group->move()==moveit::planning_interface::MoveItErrorCode::SUCCESS;
        if(!reset)
        {
            move_group->setNamedTarget("up");
            return move_group->move()==moveit::planning_interface::MoveItErrorCode::SUCCESS;
        }
    }
    return success;
}
bool Manipulator::goHome(double velocity_scale) const
{
    move_group->setGoalTolerance(Parameters.goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    move_group->setNamedTarget("home");
    return move_group->move()==moveit::planning_interface::MoveItErrorCode::SUCCESS;
}
double Manipulator::allClose(const std::vector<double> & goal) const
{
    std::vector<double> current_joints;
    current_joints=move_group->getCurrentJointValues();
    std::vector<double>	auxiliary;

    std::transform (current_joints.begin(), current_joints.end(), goal.begin(), std::back_inserter(auxiliary),\
                    [](double element1, double element2) {return pow((element1-element2),2);});
    auxiliary.shrink_to_fit();
    return sqrt(std::accumulate(auxiliary.begin(), auxiliary.end(), 0.0));
}
Eigen::Affine3d Manipulator::getEndMotion(int search_type, const Eigen::Vector3d & Translation, const Eigen::Vector3d & RPY) const
{
    Eigen::Affine3d CameraMotion,Trans_B2E;
    CameraMotion.translation()=Translation;
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix=Eigen::AngleAxisd(RPY[2], Eigen::Vector3d::UnitZ())*
                    Eigen::AngleAxisd(RPY[1], Eigen::Vector3d::UnitY())*
                    Eigen::AngleAxisd(RPY[0], Eigen::Vector3d::UnitX());
    CameraMotion.linear()=rotation_matrix;
    Eigen::fromMsg(move_group->getCurrentPose().pose,Trans_B2E);
    //left multiply or right multiply
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
bool Manipulator::goSearchOnce(int search_type, double velocity_scale,double search_angle) const
{
    move_group->setGoalTolerance(Parameters.goal_tolerance);
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
            goal = move_group->getCurrentJointValues();
            goal[4]+=search_angle;
            move_group->setJointValueTarget(goal);
            if(move_group->move()!=moveit_msgs::MoveItErrorCodes::SUCCESS)
                return false;
            if(charging)
            {
                if(!linearMoveTo(Eigen::Vector3d(0.0, 0.0, Parameters.chargeDownHeight),velocity_scale))
                    return false;
            }
            else
            {
                if(!linearMoveTo(Eigen::Vector3d(0.0, 0.0, Parameters.searchDownHeight),velocity_scale))
                    return false;
            }

            goal = move_group->getCurrentJointValues();
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
    double temp_x=10.0;
    std::vector<double> joint_values{};
    while(RobotAllRight)
    {
        ROS_INFO_ONCE("Searching tag!!!!!");

        if(!Tags_detected.empty())
        {
            if(abs(Tags_detected[0].Trans_C2T.translation()[0])<temp_x )
            {
                temp_x = abs(Tags_detected[0].Trans_C2T.translation()[0]);
                joint_values = move_group->getCurrentJointValues();
            }
        }
        if(allClose(goal)<=Parameters.goal_tolerance*2)
        {
            if(temp_x >0.2)
                return false;
            else
                break;
        }
        usleep(100000);
    }
    ROS_INFO_ONCE("Already remembered the most center joint values");
    //wait until the robot finished the asyncMove task, can't be 0 because of goal_tolerance
    while(allClose(goal)>Parameters.goal_tolerance*2);
    move_group->setMaxVelocityScalingFactor(1.0);
    move_group->setJointValueTarget(joint_values);
    return (move_group->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
}
bool Manipulator::goSearch(double velocity_scale,bool underServoing,double search_angle) const
{
    /*
    return goSearchOnce(FLAT, velocity_scale,search_angle)||
           goSearchOnce(DOWN, velocity_scale,search_angle)||
           goSearchOnce(UP, velocity_scale,search_angle);
    */
    if(!underServoing)
        return goSearchOnce(DOWN, velocity_scale,search_angle);
    else
        return goSearchOnce(FLAT, velocity_scale,search_angle);
}
bool Manipulator::goServo( double velocity_scale)
{
    Eigen::fromMsg(move_group->getCurrentPose().pose,Trans_W2E);

    double error=100.0;
    Destination_t EndDestination;

    move_group->setGoalTolerance(Parameters.goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    while(RobotAllRight&&error>Parameters.servoTolerance)
    {
        if(!Tags_detected.empty()|| goSearch(Parameters.basicVelocity,true))
        {
            EndDestination=astra->getCameraEE(Tags_detected[0].Trans_C2T,Trans_E2C,ExpectMatrix);
            error=EndDestination.error;
            if(Parameters.servoPoseOn)
            {
                if(error<0.002)
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
                if(move_group->execute(my_plan)!=moveit::planning_interface::MoveItErrorCode::SUCCESS)
                    return false;
            }
            else
            {
                ROS_ERROR("Failed");
                return false;
            }
            Eigen::fromMsg(move_group->getCurrentPose().pose,Trans_W2E);
        }
        else
            return false;
    }
    return RobotAllRight;
}
bool Manipulator::goCut(const Eigen::Affine3d &referTag,double velocity_scale)
{
    Eigen::Affine3d Trans_B2E,Trans_E2EP;
    geometry_msgs::Pose target_pose3 = move_group->getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints;
    Eigen::Quaterniond q;
    //adjust orientation
    double init_theta=asin((target_pose3.position.x-referTag.translation()[0])/(Parameters.radius+0.10));
    q=Parameters.inverse ? Eigen::Matrix3d::Identity()*Eigen::AngleAxisd(-M_PI-init_theta,Eigen::Vector3d::UnitZ())
                         : Eigen::Matrix3d::Identity()*Eigen::AngleAxisd(-init_theta,Eigen::Vector3d::UnitZ());
    target_pose3.orientation.x=q.x();
    target_pose3.orientation.y=q.y();
    target_pose3.orientation.z=q.z();
    target_pose3.orientation.w=q.w();
    move_group->setPoseTarget(target_pose3);
    if(move_group->move()!=moveit::planning_interface::MoveItErrorCode::SUCCESS)
        return false;
    //转到上方指定距离弧度一点
    target_pose3 = move_group->getCurrentPose().pose;
    double init_th = asin((target_pose3.position.x - referTag.translation()[0]) /(Parameters.radius+0.10));
    double init_x = target_pose3.position.y + (Parameters.radius + 0.10)*cos(init_th)*boost::math::sign(target_pose3.position.y);

    target_pose3.position.x = referTag.translation()[0] + (Parameters.radius + 0.10)*sin(init_th + Parameters.traceAngle);
    target_pose3.position.y = init_x + (Parameters.radius + 0.10)*cos(init_th + Parameters.traceAngle);
    target_pose3.position.z += -1*Parameters.traceDistance;
    q = Parameters.inverse ? Eigen::Matrix3d::Identity()*Eigen::AngleAxisd(-M_PI - (init_th + Parameters.traceAngle), Eigen::Vector3d::UnitZ())
                           : Eigen::Matrix3d::Identity()*Eigen::AngleAxisd(-(init_th + Parameters.traceAngle), Eigen::Vector3d::UnitZ());
    target_pose3.orientation.x = q.x();
    target_pose3.orientation.y = q.y();
    target_pose3.orientation.z = q.z();
    target_pose3.orientation.w = q.w();
    move_group->setPoseTarget(target_pose3);
    if (move_group->move() != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        return false;
    //进刀

    if(!linearMoveTo(Eigen::Vector3d(0.0,boost::math::sign(target_pose3.position.y)*0.0,0.0),velocity_scale))
        return false;

    if(!linearMoveTo(Eigen::Vector3d(0.0,0.0,-0.002),velocity_scale))
        return false;
    //切割轨迹
    target_pose3 = move_group->getCurrentPose().pose;
    Parameters.radius += 0.10;
    init_th=asin((target_pose3.position.x-referTag.translation()[0])/Parameters.radius);
    init_x=target_pose3.position.y+Parameters.radius*cos(init_th)*boost::math::sign(target_pose3.position.y);
    for(double th=(2*Parameters.traceAngle)/(double)Parameters.traceNumber;th<=2*Parameters.traceAngle;th+=2*Parameters.traceAngle/(double)Parameters.traceNumber)
    {
        target_pose3.position.x = referTag.translation()[0]+Parameters.radius*sin(init_th-th);
        target_pose3.position.y = init_x+Parameters.radius*cos(init_th-th) ;
        target_pose3.position.z +=	2*Parameters.traceDistance/(double)Parameters.traceNumber;
        q=Parameters.inverse ? Eigen::Matrix3d::Identity()*Eigen::AngleAxisd(-M_PI-(init_th-th),Eigen::Vector3d::UnitZ())
                             : Eigen::Matrix3d::Identity()*Eigen::AngleAxisd(-(init_th-th),Eigen::Vector3d::UnitZ());
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
        fraction = move_group->computeCartesianPath(waypoints, 0.001, 0.0, trajectory);
        attempts++;
        if(attempts%10==0)
            ROS_INFO("Still trying after %d attempts",attempts);
        usleep(50000);
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
        scale_trajectory_speed(my_plan,velocity_scale);
        if(move_group->execute(my_plan)!=moveit::planning_interface::MoveItErrorCode::SUCCESS)
            return false;
    }
    Parameters.radius -=0.10;
    //退刀
    if(!linearMoveTo(Eigen::Vector3d(0.0,0.0,0.002),velocity_scale))
        return false;

    return linearMoveTo(Eigen::Vector3d(0.1,-boost::math::sign(target_pose3.position.y)*0.1,0.05),velocity_scale);
}
void Manipulator::goZero(double velocity_scale)
{
    move_group->setGoalTolerance(Parameters.goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    std::vector<double> current_joints;
    current_joints=move_group->getCurrentJointValues();
    current_joints[5]=0.0;
    move_group->setJointValueTarget(current_joints);
    move_group->move();
}
bool Manipulator::goCamera(const Eigen::Vector3d & target_array, double velocity_scale)
{
    move_group->setGoalTolerance(Parameters.goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    Eigen::fromMsg(move_group->getCurrentPose().pose,Trans_W2E);

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
Eigen::Affine3d Manipulator::getTagPosition(const Eigen::Affine3d &Trans_C2T) const
{
    Eigen::Affine3d Trans_B2E;
    Eigen::fromMsg(move_group->getCurrentPose().pose,Trans_B2E);
    return Trans_B2E*Trans_E2C*Trans_C2T;
}
bool Manipulator::goCharge(double velocity_scale)
{
    move_group->setGoalTolerance(Parameters.goal_tolerance);
    move_group->setMaxVelocityScalingFactor(velocity_scale);
    Eigen::fromMsg(move_group->getCurrentPose().pose,Trans_W2E);
    Eigen::Affine3d Trans_B2T=Trans_W2E*Trans_E2C*Tags_detected[0].Trans_C2T;
    Trans_B2T.translation() += Eigen::Vector3d(boost::math::sign(Trans_B2T.translation()[1])*0.087,-0.015,-0.150);
    Eigen::Matrix3d rotation_matrix;
    Eigen::Affine3d tempMatrix;
    rotation_matrix=Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX())*
                    Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ());
    tempMatrix.linear()=rotation_matrix;
    tempMatrix.translation()=Eigen::Vector3d(0.0,0.0,0.0);
    Trans_W2EP=Trans_B2T*tempMatrix*Trans_E2CH.inverse();
    move_group->setPoseTarget(Trans_W2EP);
    if(move_group->move()!=moveit::planning_interface::MoveItErrorCode::SUCCESS)
        return false;

    return (linearMoveTo(Eigen::Vector3d(0.035,0.00,0.00),velocity_scale));
}
bool Manipulator::leaveCharge(double velocity_scale)
{

    if(!linearMoveTo(Eigen::Vector3d(-0.10,0.00,0.00),velocity_scale))
        return false;
    move_group->setNamedTarget("up");
    return (move_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}
int Manipulator::executeService(int serviceType)
{

    running_=true;
    int serviceStatus;
    switch (serviceType)
    {
        case visual_servo::manipulate::Request::CUT:
            goUp(0.8,false);
            if(Tags_detected.empty())
            {
                if(!goSearch(Parameters.basicVelocity))
                {
                    ROS_INFO("!!!!!!NO TAG SEARCHED!!!!!");
                    serviceStatus=visual_servo_namespace::SERVICE_STATUS_NO_TAG;
                    goUp(Parameters.basicVelocity);
                    goHome(Parameters.basicVelocity);
                }
                else
                {
                    addDynamicPlanningConstraint();
                    if(!goServo(Parameters.basicVelocity))
                        serviceStatus=visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED;
                    else
                    {
                        //robot_manipulator.goZero();
                        Eigen::Vector3d Center3d=Tags_detected[0].Trans_C2T.translation();
                        Eigen::Affine3d tagTransform=getTagPosition(Tags_detected[0].Trans_C2T);
                        Center3d+=Parameters.cameraXYZ;
                        if(!goCamera(Center3d,Parameters.basicVelocity))
                            serviceStatus=visual_servo_namespace::SERVICE_STATUS_CLOSE_FAILED;
                        else
                        {
                            serviceStatus=  goCut(tagTransform,Parameters.basicVelocity/10.0) ?
                                            visual_servo_namespace::SERVICE_STATUS_SUCCEED :
                                            visual_servo_namespace::SERVICE_STATUS_CUT_FAILED;
                        }
                    }
                    addDynamicPlanningConstraint(true);
                    goUp(Parameters.basicVelocity);
                    goHome(Parameters.basicVelocity);
                }
            }
            else
            {   addDynamicPlanningConstraint();
                if(!goServo(Parameters.basicVelocity))
                    serviceStatus=visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED;
                else
                {
                    Eigen::Vector3d Center3d=Tags_detected[0].Trans_C2T.translation();
                    Eigen::Affine3d tagTransform=getTagPosition(Tags_detected[0].Trans_C2T);
                    Center3d+=Parameters.cameraXYZ;
                    if(!goCamera(Center3d,Parameters.basicVelocity))
                        serviceStatus=visual_servo_namespace::SERVICE_STATUS_CLOSE_FAILED;
                    else
                    {
                        serviceStatus=  goCut(tagTransform,Parameters.basicVelocity/10.0) ?
                                        visual_servo_namespace::SERVICE_STATUS_SUCCEED :
                                        visual_servo_namespace::SERVICE_STATUS_CUT_FAILED;

                    }
                }
                addDynamicPlanningConstraint(true);
                goUp(Parameters.basicVelocity);
                goHome(Parameters.basicVelocity);
            }
            break;
        case visual_servo::manipulate::Request::SEARCH:
            goUp(0.8,false);
            serviceStatus  = goSearch(Parameters.basicVelocity) ?
                             visual_servo_namespace::SERVICE_STATUS_SUCCEED :
                             visual_servo_namespace::SERVICE_STATUS_NO_TAG;
            break;
        case visual_servo::manipulate::Request::CHARGE:
            charging=true;
            Parameters.inverse=true;
            goUp(0.8,false);
            if(Tags_detected.empty())
            {
                if(!goSearch(0.5))
                {
                    ROS_INFO("!!!!!!NO TAG SEARCHED!!!!!");
                    serviceStatus=visual_servo_namespace::SERVICE_STATUS_NO_TAG;
                    goUp(Parameters.basicVelocity);
                    goHome(Parameters.basicVelocity);
                }
                else
                {
                    addChargerDynamicPlanningConstraint();
                    bool servo_success=goServo(Parameters.basicVelocity);
                    if(!servo_success)
                        serviceStatus=visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED;
                    else
                    {
                        if(!goCharge(Parameters.basicVelocity))
                            serviceStatus=visual_servo_namespace::SERVICE_STATUS_CHARGE_FAILED;
                        else
                        {
                            sleep(20);
                            serviceStatus=   leaveCharge(Parameters.basicVelocity) ?
                                             visual_servo_namespace::SERVICE_STATUS_SUCCEED :
                                             visual_servo_namespace::SERVICE_STATUS_LEAVE_CHARGE_FAILED;
                        }
                    }
                    goHome(Parameters.basicVelocity);
                }
            }
            else
            {
                addChargerDynamicPlanningConstraint();
                bool servo_success=goServo(Parameters.basicVelocity);
                if(!servo_success)
                    serviceStatus=visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED;
                else
                {
                    if(!goCharge(Parameters.basicVelocity))
                        serviceStatus=visual_servo_namespace::SERVICE_STATUS_CHARGE_FAILED;
                    else
                    {
                        ros::param::set("/visual_servo/isChargingStatusChanged",(double)true);
                        sleep(20);
                        ros::param::set("/visual_servo/isChargingStatusChanged",(double)true);
                        serviceStatus=   leaveCharge(Parameters.basicVelocity) ?
                                         visual_servo_namespace::SERVICE_STATUS_SUCCEED :
                                         visual_servo_namespace::SERVICE_STATUS_LEAVE_CHARGE_FAILED;
                    }
                }
                goHome(Parameters.basicVelocity);
            }
            charging=false;
            Parameters.inverse=false;
            break;
        case visual_servo::manipulate::Request::UP:
            serviceStatus = goUp(Parameters.basicVelocity, false) ?
                            visual_servo_namespace::SERVICE_STATUS_SUCCEED :
                            visual_servo_namespace::SERVICE_STATUS_UP_FAILED;
            break;
        case visual_servo::manipulate::Request::HOME:
            serviceStatus = goHome(Parameters.basicVelocity) ?
                            visual_servo_namespace::SERVICE_STATUS_SUCCEED :
                            visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
            break;
        default:
            serviceStatus=visual_servo_namespace::SERVICE_STATUS_EMPTY;
            break;
    }
    if(!RobotAllRight)
        serviceStatus=visual_servo_namespace::SERVICE_STATUS_ROBOT_ABORT;
    ros::param::set("/visual/tagDetectorOn", false);
    running_=false;
    return serviceStatus;
}

Listener::Listener()
{
    this->tag_info_received=false;
    tag_info_sub =nh.subscribe("TagsDetected",1000, &Listener::tagInfoCallback,this);
    vs_status_sub = nh.subscribe("VisualServoStatus",100,&Listener::statusCallback,this);
    client =nh.serviceClient<visual_servo::detect_once>("detect_once");
    srv.request.ready_go=(int)detect_srv_on;
    manipulate_callback_t manipulate_callback = boost::bind(&Listener::manipulate, this, _1, _2);
    service_ = nh.advertiseService("manipulate",manipulate_callback);
    watchdog_timer_ = nh.createTimer(ros::Duration(WATCHDOG_PERIOD_), &Listener::watchdog, this, true);
}
Listener::~Listener()
{
    //delete tfListener;
    watchdog_timer_.stop();
}

void Listener::tagInfoCallback(const visual_servo::TagsDetection_msg &msg)
{
    watchdog_timer_.stop();
    watchdog_timer_.start();
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
void Listener::statusCallback(const visual_servo::VisualServoMetaTypeMsg &msg)
{
    RobotAllRight=msg.RobotAllRight;
    //std::cout<<"RobotAllRight: "<<(int) RobotAllRight<<"RobotSwitchOn: "<< (int) RobotSwitchOn<<std::endl;
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
    ros::Time temp;
    while(manipulate_srv_on)
    {
        temp=ros::Time::now();
        usleep(500000);
    }
    res.status=ManipulateSrv.srv_status;
    ManipulateSrv.srv_status=visual_servo_namespace::SERVICE_STATUS_EMPTY;
    ManipulateSrv.srv_type=visual_servo::manipulate::Request::EMPTY;
    return true;
}
void Listener::watchdog(const ros::TimerEvent &e)
{
    ROS_WARN("tag info not received for %f seconds, is the camera node drop?", WATCHDOG_PERIOD_);
    this->tag_info_received=false;
}