#include <geometry_msgs/PoseStamped.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <ros/forwards.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/single_subscriber_publisher.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <iterator>
#include <map>
#include <numeric>

#include "ManiSerialManager.h"
#include "Servo.h"
#include "parameterTeleop.h"
#include "ros/callback_queue.h"
#define _USE_MATH_DEFINES

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template <typename T>
void operator>>(const YAML::Node &node, T &i) {
  i = node.as<T>();
}
#endif

/*Global variables action in multi threads
 * @param Tags_detected     [Tags information received from camera thread,
 * vector is empty when no tags were detected]
 * @param manipulate_srv_on [The manipulation service trigger]
 * @param RobotAllRight     [The robot status, false for something wrong]
 * @param isRobotMoving     [The robot moving status, false for not moving]
 */

std::atomic_bool manipulate_srv_on{};
std::atomic_bool RobotAllRight{};
std::atomic_bool isRobotMoving{};

tag_detection_info_t Tags_detected{};
manipulateSrv ManipulateSrv{};

boost::shared_mutex tag_data_mutex{};

void *camera_thread(void *data);

class Manipulator {
 private:
  ros::NodeHandle n_;
  // debug test 1221
  ros::Publisher debug_test_pub_;
  // For remote debug/simper logger
  ros::Publisher log_pub_;

  bool running_{};
  /*
   * Specifies search operations
   * The enum specifies three search operations,FLAT,DOWN,UP
   * of searching tags when no tags detected.
   * Default for doing nothing.
   */
  enum SearchType { FLAT = 0, DOWN = 1, UP = 2, DEFAULT = 3 };
  enum MultiplyType {
    RIGHT = 4,
    LEFT = 5,
  };
  enum KnifeStatus {
    KNIFE_READY,
    KNIFE_LEFT,
    KNIFE_RIGHT,
    KNIFE_ON,
    KNIFE_OFF,
    KNIFE_FORWARD,
    KNIFE_BACK,
  } knife_status;
  // the planning_group name specified by the moveit_config package
  const std::string PLANNING_GROUP = "manipulator_i5";

  // the name of rubber tree
  const std::string TREE_NAME = "Heava";

  const std::vector<std::string> PARAMETER_NAMES = {
      "/user/inverse", "/robot/realWork", "/visual_servo/knifeLeftMoveEnd",
      "/visual_servo/knifeRightMoveEnd", "/user/debugOn"};

  std::string EE_NAME;
  moveit::planning_interface::MoveGroupInterface *move_group;
  moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;
  const robot_state::JointModelGroup *joint_model_group;

  bool charging{}, isTagEmpty, isInverse{false};

  // Servo class for computing the servo related transform matrix
  Servo *astra;

  // parameter listener
  ParameterListener *parameterListener_;

  // a legacy function for sending message online through the serial
  ManiSerialManager *communicator_;
  /*
   * The transform matrix of A with respect to B named as Trans_B2A.
   * W-world,E-end effector,C-camera,EP-desired end
   * effector,B-base_link,T-target
   */
  Eigen::Affine3d Trans_W2E, Trans_W2EP, Trans_E2C, Trans_E2CH, Trans_B2T_;

  // The desired camera pose matrix with respect to tag
  Eigen::Affine3d ExpectMatrix;

  /* Function for computing the joint space Euclidean distance,
   * between the given goal and now joint states.
   * @param goal  [the given joint space goal]
   * @return the joint space Euclidean distance as a double number,
   *  Unit-radian
   */
  double allClose(const std::vector<double> &goal) const;
  // make the robot move as a line giving the destination
  bool linearMoveTo(const Eigen::Vector3d &destination, double velocity_scale);
  // make the robot move in a approximate constant velocity
  bool constantMoveTo(const Eigen::Vector3d &destination,
                      double velocity_limit);
  /* Function for computing the matrix of desired end effector motion.
   * @param search_type   [define the reference coordinate system of desired
   * motion, UP for end effector, DOWN for world, DEFAULT for end effector]
   * @param Translation   [motion's translation part]
   * @param RPY           [motion's rotation part described by RPY values]
   * @return the desired end effector motion matrix.
   */
  Eigen::Affine3d getEndMotion(
      MultiplyType multiply_type, const Eigen::Vector3d &Translation,
      const Eigen::Vector3d &RPY = Eigen::Vector3d(0.0, 0.0, 0.0));
  // read parameters and listen to tf
  void initParameter();
  // scale trajectory speed by given multiple (legacy function)
  void scale_trajectory_speed(
      moveit::planning_interface::MoveGroupInterface::Plan &plan,
      double scale_factor) const;
  // reset the tool into the original(0.0) status(legacy function)
  void resetTool();
  // update the IDth parameters;
  void updateParameters(int ID);

  void saveCutTimes(int ID, double cutTimes);

  // warning, only used when the vel is low, and the velocity is constant but
  // the execute time is not equal to destination.norm()/speed;
  void setAvgCartesianSpeed(
      moveit::planning_interface::MoveGroupInterface::Plan &plan, double speed);

  // prameters
  struct {
    // the radius of rubber tree, used as a reference for constructing planning
    // constraints,and generating the cut locus.
    double radius;
    Eigen::Vector3d expectRPY;
    Eigen::Vector3d expectXYZ;
    Eigen::Vector3d cameraXYZ;
    bool inverse;
    bool servoPoseOn;
    bool debugOn;
    double searchDownHeight;
    double chargeDownHeight;
    double basicVelocity;
    double servoTolerance;
    double goal_tolerance;
    std::string cameraFrame;
    std::string toolFrame;
    double compensateP;
    int cutTimes;
    bool realWork;
    double steeringDistance;
    int ID;
  } Parameters{};

 public:
  Manipulator();

  virtual ~Manipulator();

  void setParametersFromCallback();
  // Function aims at adding static virtual walls to restrict planning
  void addStaticPlanningConstraint() const;
  /* Function aims at adding dynamic virtual cylinder as tree to restrict
   * planning
   * @param leave [true for more restrict when robot try to leave]
   */
  void addDynamicPlanningConstraint(bool goDeep = false, bool servoing = false,
                                    bool camera = false);
  // Function aims at adding dynamic virtual cylinder as charger plane to
  // restrict planning
  void addChargerDynamicPlanningConstraint() const;
  /* Function makes all robot joints go to zero
   * @param reset [true for go up to the other side]
   */
  void addDangerDynamicPlanningConstraint();
  void removeDangerDynamicPlanningConstraint() const;
  moveit_msgs::JointConstraint addJointConstraint(double tolerance_angle,
                                                  int index = 0) const;

  bool goUp(double velocity_scale) const;
  // Function makes robot go to a preset position
  bool goHome(double velocity_scale);
  /* Function makes robot go search once with a defined search angle range
   * @param search_type       [define the search type,UP for looking up,DOWN for
   * looking down, FLAT for looking flat,DEFAULT for doing nothing]
   * @param search_angle      [define the search angle range 0-pi]
   * @param velocity_scale    [velocity scaling factor 0-1]
   * @return true if any tag searched else false
   */
  bool goSearchOnce(SearchType search_type, double velocity_scale,
                    double search_angle = M_PI / 4.0);
  // Function makes robot search all three types
  bool goSearch(double velocity_scale, bool underServoing = false,
                double search_angle = M_PI / 4.0);
  /*Function makes robot camera servo to pointed pose or position
   * @param ExpectMatrix  [the desired pose of camera describing as transform
   * matrix]
   * @param enable_pose   [true for enable pose servo mode which costs more
   * adjust time and may makes end effector rotate, false for position servo
   * only]
   * @return true if achieved the desired pose or position
   */
  bool goServo(double velocity_scale);
  // Function makes robot accomplish cut task
  bool goCut(double velocity_scale);
  /*Function makes the end effector goes to a position defined by a array with
   * respect to camera
   * @param target_array  [describes a position with respect to camera]
   * @return true if achieved the desired position.
   */
  bool goCamera(const Eigen::Vector3d &target_array,
                double velocity_scale = 0.1);
  // Function makes the charger goes to a fixed position related to the tag
  bool goCharge(double velocity_scale = 0.5);
  // Function makes the charger leave from the station
  bool leaveCharge(double velocity_scale = 0.5);
  // Function executes the service
  int executeService(int serviceType);

  void removeAllDynamicConstraint();

  // Function computes the tag pose with respect to the planning frame
  Eigen::Affine3d getTagPosition(const Eigen::Affine3d &Trans_C2T) const;
  // debug preserved functions
  void logPublish(const std::string &log_data);
};

// all ros topics and services related functions
class Listener {
 private:
  const double WATCHDOG_PERIOD_ = 1.0;
  ros::NodeHandle nh;
  // trace detection service client
  ros::ServiceClient client;
  ros::Subscriber tag_info_sub;
  ros::Subscriber vs_status_sub;
  ros::Subscriber joint_state_sub_;
  // manipulate service server
  ros::ServiceServer service_;
  ros::Timer watchdog_timer_;
  sensor_msgs::JointState last_received_state_{};

  void tagInfoCallback(const visual_servo::TagsDetection_msg &msg);
  // arm controller status callback, published by auboRobot
  void statusCallback(const visual_servo::VisualServoMetaTypeMsg &msg);
  void jointStateCallback(const sensor_msgs::JointState &msg);
  bool manipulate(visual_servo::manipulate::Request &req,
                  visual_servo::manipulate::Response &res);

  void watchdog(const ros::TimerEvent &e);

 public:
  typedef boost::function<bool(visual_servo::manipulate::Request &,
                               visual_servo::manipulate::Response &res)>
      manipulate_callback_t;
  bool tag_info_received;
  Listener();
  virtual ~Listener();
};

int main(int argc, char **argv) {
  pthread_t camera_id;
  ros::init(argc, argv, "visual_servo");

  pthread_create(&camera_id, NULL, camera_thread, NULL);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  Manipulator robot_manipulator;
  robot_manipulator.addStaticPlanningConstraint();
  ros::Rate loop_rate(30);
  while (ros::ok()) {
    robot_manipulator.setParametersFromCallback();
    if (manipulate_srv_on) {
      if (RobotAllRight) {
        ros::param::set("/visual/tagDetectorOn", true);
        ManipulateSrv.srv_status =
            robot_manipulator.executeService(ManipulateSrv.srv_type);
      } else
        ManipulateSrv.srv_status =
            visual_servo_namespace::SERVICE_STATUS_ROBOT_ABORT;
      // check again
      if (!RobotAllRight)
        ManipulateSrv.srv_status =
            visual_servo_namespace::SERVICE_STATUS_ROBOT_ABORT;
      manipulate_srv_on = false;
      ROS_INFO("Service executed!");
      // robot_manipulator.logPublish("INFO: Service executed");
    }
    loop_rate.sleep();
  }
  return 0;
}

void *camera_thread(void *data) {
  Listener listener;
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::Rate loop_rate(120);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

Manipulator::Manipulator() {
  log_pub_ = n_.advertise<std_msgs::String>("robot_log", 1);
  // debug test 1221
  debug_test_pub_ = n_.advertise<std_msgs::Int8>("datatest", 10);
  move_group =
      new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
  planning_scene_interface =
      new moveit::planning_interface::PlanningSceneInterface;
  joint_model_group =
      move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  astra = new Servo;
  parameterListener_ = new ParameterListener(30, 8);
  communicator_ = new ManiSerialManager("/dev/tty/S2", B115200);
  ROS_INFO_NAMED("Visual Servo", "Reference frame: %s",
                 move_group->getPlanningFrame().c_str());
  ROS_INFO_NAMED("Visual Servo", "End effector link: %s",
                 move_group->getEndEffectorLink().c_str());
  EE_NAME = move_group->getEndEffectorLink();
  move_group->allowReplanning(true);
  move_group->setNumPlanningAttempts(20);
  initParameter();
  if (communicator_->openSerial()) {
    communicator_->registerAutoReadThread(60);
    usleep(200000);
    std::string serial_send{"rubber tapping robot is online"};
    communicator_->send(serial_send.c_str(), serial_send.size());
  } else
    ROS_ERROR_STREAM("CAN'T OPEN THE SERIAL" << communicator_->getSerialAddr());
}
Manipulator::~Manipulator() {
  delete communicator_;
  delete parameterListener_;
  delete astra;
  delete planning_scene_interface;
  delete move_group;
}
void Manipulator::logPublish(const std::string &log_data) {
  std_msgs::String log;
  log.data = log_data;
  log_pub_.publish(log);
}
void Manipulator::setParametersFromCallback() {
  if (!running_) {
    Parameters.inverse = (bool)parameterListener_->parameters()[0];
    Parameters.realWork = (bool)parameterListener_->parameters()[1];
    Parameters.debugOn = (bool)parameterListener_->parameters()[4];
  }
}
void Manipulator::initParameter() {
  n_.param<double>("/user/radius", Parameters.radius, 0.01);
  n_.param<double>("/user/expectRoll", Parameters.expectRPY[0], 0.0);
  n_.param<double>("/user/expectPitch", Parameters.expectRPY[1], 0.0);
  n_.param<double>("/user/expectYaw", Parameters.expectRPY[2], 0.0);
  n_.param<double>("/user/expectX", Parameters.expectXYZ[0], 0.0);
  n_.param<double>("/user/expectY", Parameters.expectXYZ[1], 0.0);
  n_.param<double>("/user/expectZ", Parameters.expectXYZ[2], 0.4);
  n_.param<bool>("/user/servoPoseOn", Parameters.servoPoseOn, false);
  n_.param<double>("/user/searchDownHeight", Parameters.searchDownHeight,
                   -0.15);
  n_.param<double>("/user/chargeDownHeight", Parameters.chargeDownHeight,
                   -0.30);
  n_.param<double>("/user/goCameraX", Parameters.cameraXYZ[0], 0.0);
  n_.param<double>("/user/goCameraY", Parameters.cameraXYZ[1], 0.0);
  n_.param<double>("/user/goCameraZ", Parameters.cameraXYZ[2], -0.2);
  n_.param<double>("/user/steeringDistance", Parameters.steeringDistance,
                   190.0);
  n_.param<double>("/robot/basicVelocity", Parameters.basicVelocity, 0.5);
  n_.param<double>("/robot/goalTolerance", Parameters.goal_tolerance, 0.001);
  n_.param<double>("/robot/servoTolerance", Parameters.servoTolerance, 0.001);
  n_.param<std::string>("/user/cameraFrame", Parameters.cameraFrame,
                        "camera_color_optical_frame");
  n_.param<std::string>("/user/toolFrame", Parameters.toolFrame,
                        "charger_ee_link");

  double temp{};
  n_.param<double>("/robot/realWork", temp, 0.0);
  Parameters.realWork = (bool)temp;
  n_.param<double>("/user/inverse", temp, 0.0);
  Parameters.inverse = (bool)temp;
  n_.param<double>("/user/debugOn", temp, 0.0);
  Parameters.debugOn = (bool)temp;
  std::cout << "read parameter success" << std::endl;

  ExpectMatrix.linear() = Eigen::Matrix3d{
      Eigen::AngleAxisd(Parameters.expectRPY[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(Parameters.expectRPY[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(Parameters.expectRPY[0], Eigen::Vector3d::UnitX())};
  ExpectMatrix.translation() = Parameters.expectXYZ;

  astra->getTransform(EE_NAME, Parameters.cameraFrame, Trans_E2C);
  astra->getTransform(EE_NAME, Parameters.toolFrame, Trans_E2CH);

  RobotAllRight = (Parameters.cameraFrame != "camera_color_optical_frame");
  std::cout << "RobotAllRight" << RobotAllRight << std::endl;
  parameterListener_->registerParameterCallback(PARAMETER_NAMES, false);
}
void Manipulator::addStaticPlanningConstraint() const {
  // in case of repeat usage
  std::vector<std::string> object_names{
      planning_scene_interface->getKnownObjectNames()};
  if (!object_names.empty()) {
    for (auto &object_name : object_names) {
      if (object_name == TREE_NAME) {
        object_names.erase(std::remove(std::begin(object_names),
                                       std::end(object_names), object_name),
                           std::end(object_names));
      }
    }
    planning_scene_interface->removeCollisionObjects(object_names);
  }
  std::vector<moveit_msgs::CollisionObject> objects;

  // virtual ground
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group->getPlanningFrame();
  collision_object.id = "ground";
  shape_msgs::Plane plane;
  plane.coef = {0, 0, 1, 0};
  geometry_msgs::Pose object_pose;
  object_pose.position.x = 0;
  object_pose.position.y = 0;
  object_pose.position.z = -0.305;
  object_pose.orientation.w = 1.0;
  collision_object.planes.push_back(plane);
  collision_object.plane_poses.push_back(object_pose);
  collision_object.operation = collision_object.ADD;
  objects.push_back(collision_object);

  // virtual LIDAR
  moveit_msgs::CollisionObject collision_object_LIDAR;
  collision_object_LIDAR.header.frame_id = move_group->getPlanningFrame();
  collision_object_LIDAR.id = "LIDAR";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = 0.073;
  primitive.dimensions[1] = 0.05;
  object_pose.position.x = 0.32;
  object_pose.position.z = 0.0765;
  collision_object_LIDAR.primitives.push_back(primitive);
  collision_object_LIDAR.primitive_poses.push_back(object_pose);
  collision_object_LIDAR.operation = collision_object.ADD;
  objects.push_back(collision_object_LIDAR);

  moveit_msgs::CollisionObject collision_object_CAR;
  collision_object_CAR.header.frame_id = move_group->getPlanningFrame();
  collision_object_CAR.id = "CAR";
  shape_msgs::SolidPrimitive car_primitive;
  car_primitive.type = primitive.BOX;
  car_primitive.dimensions.resize(3);
  car_primitive.dimensions[0] = 0.675;
  car_primitive.dimensions[1] = 0.360;
  car_primitive.dimensions[2] = 0.26;
  object_pose.position.x = 0.0;
  object_pose.position.y = 0.0;
  object_pose.position.z = -0.105;
  collision_object_CAR.primitives.push_back(car_primitive);
  collision_object_CAR.primitive_poses.push_back(object_pose);
  collision_object_CAR.operation = collision_object.ADD;
  objects.push_back(collision_object_CAR);

  planning_scene_interface->addCollisionObjects(objects);
}
void Manipulator ::addDynamicPlanningConstraint(bool goDeep, bool servoing,
                                                bool camera) {
  tag_data_mutex.lock_shared();
  if (!Tags_detected.empty()) {
    Trans_B2T_ = getTagPosition(Tags_detected[0].Trans_C2T);
  }
  tag_data_mutex.unlock_shared();
  std::vector<std::string> object_names{
      planning_scene_interface->getKnownObjectNames()};
  const std::vector<std::string> tree_name{TREE_NAME};
  if (!object_names.empty()) {
    for (auto &object_name : object_names) {
      if (object_name == TREE_NAME) {
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
  if (!Parameters.inverse)
    object_pose.position.x = Trans_B2T_.translation()[0] - 0.0;
  else
    object_pose.position.x = Trans_B2T_.translation()[0] + 0.05;
  if (camera) {
    object_pose.position.y = Trans_B2T_.translation()[1] +
                             (Parameters.radius - 0.15) *
                                 boost::math::sign(Trans_B2T_.translation()[1]);
  } else {
    if (!goDeep && servoing)
      object_pose.position.y =
          Trans_B2T_.translation()[1] +
          (Parameters.radius - 0.15) *
              boost::math::sign(Trans_B2T_.translation()[1]);
    else if (goDeep && !servoing)
      object_pose.position.y =
          Trans_B2T_.translation()[1] +
          (Parameters.radius + 0.4) *
              boost::math::sign(Trans_B2T_.translation()[1]);
  }
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(object_pose);
  collision_object.operation = collision_object.ADD;
  objects.push_back(collision_object);

  planning_scene_interface->addCollisionObjects(objects);
}
void Manipulator::addChargerDynamicPlanningConstraint() const {
  tag_data_mutex.lock_shared();
  Eigen::Affine3d Trans_B2T{getTagPosition(Tags_detected[0].Trans_C2T)};
  tag_data_mutex.unlock_shared();
  std::vector<std::string> object_names{
      planning_scene_interface->getKnownObjectNames()};
  const std::vector<std::string> charger_name{"CHARGER"};
  if (!object_names.empty()) {
    for (auto &object_name : object_names) {
      if (object_name == "CHARGER") {
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
  object_pose.position.x = 0;
  object_pose.position.y = boost::math::sign(Trans_B2T.translation()[1]) *
                           (abs(Trans_B2T.translation()[1]) + 0.03);
  object_pose.position.z = 1;
  object_pose.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(M_PI / 2.0, 0, 0);
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(object_pose);
  collision_object.operation = collision_object.ADD;
  objects.push_back(collision_object);

  planning_scene_interface->addCollisionObjects(objects);
}
void Manipulator::addDangerDynamicPlanningConstraint() {
  tf2::fromMsg(move_group->getCurrentPose().pose, Trans_W2E);
  std::vector<std::string> object_names{
      planning_scene_interface->getKnownObjectNames()};
  const std::vector<std::string> danger_tree_name{"danger_tree"};
  if (!object_names.empty()) {
    for (auto &object_name : object_names) {
      if (object_name == "danger_tree") {
        planning_scene_interface->removeCollisionObjects(danger_tree_name);
      }
    }
  }
  std::vector<moveit_msgs::CollisionObject> objects;
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group->getPlanningFrame();
  collision_object.id = "danger_tree";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = 4;
  primitive.dimensions[1] = 0.02;

  geometry_msgs::Pose object_pose;
  object_pose.position.x = Trans_W2E.translation()[0];
  object_pose.position.y =
      Trans_W2E.translation()[1] +
      (0.02 + 0.005) * boost::math::sign(Trans_W2E.translation()[1]);
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(object_pose);
  collision_object.operation = collision_object.ADD;
  objects.push_back(collision_object);

  planning_scene_interface->addCollisionObjects(objects);
}

void Manipulator::removeAllDynamicConstraint() {
  std::vector<std::string> object_names{
      planning_scene_interface->getKnownObjectNames()};
  std::vector<std::string> dynamic_names;
  if (!object_names.empty()) {
    for (auto &object_name : object_names) {
      if (object_name == "CHARGER" || object_name == TREE_NAME ||
          object_name == "danger_tree") {
        dynamic_names.push_back(object_name);
      }
    }
    planning_scene_interface->removeCollisionObjects(dynamic_names);
  }
  logPublish("Rubber Tapping Done");
  resetTool();
}
void Manipulator::removeDangerDynamicPlanningConstraint() const {
  std::vector<std::string> object_names{
      planning_scene_interface->getKnownObjectNames()};
  std::vector<std::string> dynamic_names;
  if (!object_names.empty()) {
    for (auto &object_name : object_names) {
      if (object_name == "danger_tree") {
        dynamic_names.push_back(object_name);
      }
    }
    planning_scene_interface->removeCollisionObjects(dynamic_names);
  }
}
moveit_msgs::JointConstraint Manipulator::addJointConstraint(
    double tolerance_angle, int index) const {
  // joint constraint
  const std::vector<std::string> &joint_names =
      joint_model_group->getVariableNames();
  moveit_msgs::JointConstraint jcm;
  jcm.joint_name = joint_names[index];
  if (index == 0)
    jcm.position = Parameters.inverse ? -1.5707 : 1.5707;
  else
    jcm.position = 0;
  if (index != 2) {
    jcm.tolerance_above = tolerance_angle * M_PI / 180.0;
    jcm.tolerance_below = tolerance_angle * M_PI / 180.0;
  } else {
    jcm.tolerance_above = tolerance_angle * M_PI / 180.0;
    jcm.tolerance_below = 0;
  }
  jcm.weight = 1.0;
  return jcm;
}
void Manipulator::scale_trajectory_speed(
    moveit::planning_interface::MoveGroupInterface::Plan &plan,
    double scale_factor) const {
  for (auto &point : plan.trajectory_.joint_trajectory.points) {
    point.time_from_start *= 1.0 / scale_factor;
    for (int i = 0; i < plan.trajectory_.joint_trajectory.joint_names.size();
         ++i) {
      point.velocities[i] *= scale_factor;
      point.accelerations[i] *= scale_factor * scale_factor;
    }
  }
}

void Manipulator::setAvgCartesianSpeed(
    moveit::planning_interface::MoveGroupInterface::Plan &plan,
    const double speed) {
  // TODO : WHEN THEN OLD_TIMESTAMP IS BIGGER, WE SHOULD INTERPOLATE IT RATHER
  // THAN GIVE IT UP
  // TODO : OR WE SHOULD INTERPOLATE FIRST THEN ADJUST IT
  robot_state::RobotStatePtr kinematic_state(move_group->getCurrentState());
  kinematic_state->setToDefaultValues();

  int num_waypoints =
      plan.trajectory_.joint_trajectory.points
          .size();  // gets the number of waypoints in the trajectory
  const std::vector<std::string> joint_names =
      plan.trajectory_.joint_trajectory
          .joint_names;  // gets the names of the joints being updated in the
                         // trajectory

  // set joint positions of zeroth waypoint
  kinematic_state->setVariablePositions(
      joint_names, plan.trajectory_.joint_trajectory.points.at(0).positions);

  Eigen::Affine3d current_end_effector_state =
      kinematic_state->getGlobalLinkTransform(EE_NAME);
  Eigen::Affine3d next_end_effector_state;
  double euclidean_distance, new_timestamp, old_timestamp, q1, q2, q3, dt1, dt2,
      v1, v2, a;
  trajectory_msgs::JointTrajectoryPoint *prev_waypoint, *curr_waypoint,
      *next_waypoint;

  for (int i = 0; i < num_waypoints - 1; i++)  // loop through all waypoints
  {
    curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(i);
    next_waypoint = &plan.trajectory_.joint_trajectory.points.at(i + 1);

    // set joints for next waypoint
    kinematic_state->setVariablePositions(joint_names,
                                          next_waypoint->positions);

    // do forward kinematics to get cartesian positions of end effector for next
    // waypoint
    next_end_effector_state = kinematic_state->getGlobalLinkTransform(EE_NAME);

    // get euclidean distance between the two waypoints
    euclidean_distance = (next_end_effector_state.translation() -
                          current_end_effector_state.translation())
                             .norm();

    new_timestamp =
        curr_waypoint->time_from_start.toSec() +
        (euclidean_distance / speed);  // start by printing out all 3 of these!
    old_timestamp = next_waypoint->time_from_start.toSec();

    // update next waypoint timestamp & joint velocities/accelerations if joint
    // velocity/acceleration constraints allow
    if (new_timestamp > old_timestamp)
      next_waypoint->time_from_start.fromSec(new_timestamp);
    else {
      ROS_WARN_STREAM_NAMED("setAvgCartesianSpeed",
                            "Average speed is too fast. Moving as fast as "
                            "joint constraints allow. and "
                                << i);
    }

    // update current_end_effector_state for next iteration
    current_end_effector_state = next_end_effector_state;
  }

  // now that timestamps are updated, update joint velocities/accelerations
  // (used updateTrajectory from iterative_time_parameterization as a reference)
  for (int i = 0; i < num_waypoints; i++) {
    curr_waypoint = &plan.trajectory_.joint_trajectory.points.at(
        i);  // set current, previous & next waypoints
    if (i > 0)
      prev_waypoint = &plan.trajectory_.joint_trajectory.points.at(i - 1);
    if (i < num_waypoints - 1)
      next_waypoint = &plan.trajectory_.joint_trajectory.points.at(i + 1);

    if (i == 0)  // update dt's based on waypoint (do this outside of loop to
                 // save time)
      dt1 = dt2 = next_waypoint->time_from_start.toSec() -
                  curr_waypoint->time_from_start.toSec();
    else if (i < num_waypoints - 1) {
      dt1 = curr_waypoint->time_from_start.toSec() -
            prev_waypoint->time_from_start.toSec();
      dt2 = next_waypoint->time_from_start.toSec() -
            curr_waypoint->time_from_start.toSec();
    } else
      dt1 = dt2 = curr_waypoint->time_from_start.toSec() -
                  prev_waypoint->time_from_start.toSec();

    for (int j = 0; j < joint_names.size();
         j++)  // loop through all joints in waypoint
    {
      if (i == 0)  // first point
      {
        q1 = next_waypoint->positions.at(j);
        q2 = curr_waypoint->positions.at(j);
        q3 = q1;
      } else if (i < num_waypoints - 1)  // middle points
      {
        q1 = prev_waypoint->positions.at(j);
        q2 = curr_waypoint->positions.at(j);
        q3 = next_waypoint->positions.at(j);
      } else  // last point
      {
        q1 = prev_waypoint->positions.at(j);
        q2 = curr_waypoint->positions.at(j);
        q3 = q1;
      }

      if (dt1 == 0.0 || dt2 == 0.0)
        v1 = v2 = a = 0.0;
      else {
        v1 = (q2 - q1) / dt1;
        v2 = (q3 - q2) / dt2;
        a = 2.0 * (v2 - v1) / (dt1 + dt2);
      }

      // actually set the velocity and acceleration
      curr_waypoint->velocities.at(j) = (v1 + v2) / 2;
      curr_waypoint->accelerations.at(j) = a;
    }
  }
  ROS_INFO("SET VELOCITY FINISH");
}
bool Manipulator::linearMoveTo(const Eigen::Vector3d &destination_translation,
                               double velocity_scale) {
  move_group->setGoalTolerance(Parameters.goal_tolerance);
  tf2::fromMsg(move_group->getCurrentPose().pose, Trans_W2E);

  move_group->setMaxVelocityScalingFactor(velocity_scale);
  move_group->setMaxAccelerationScalingFactor(1.0);

  Eigen::Affine3d DesiredMotion = getEndMotion(RIGHT, destination_translation);

  // not go forward
  if (!(Trans_W2E.translation()[1] != 0 &&
        boost::math::sign(Trans_W2E.translation()[1]) ==
            boost::math::sign(DesiredMotion.translation()[1])))
    addDangerDynamicPlanningConstraint();

  std::vector<double> tolerance_pose{
      0.05,
      (DesiredMotion.translation() - Trans_W2E.translation()).norm() * 1.5,
      0.1};
  std::vector<double> tolerance_angle(3, 0.0005);

  moveit_msgs::Constraints path_constraints =
      kinematic_constraints::constructGoalConstraints(
          EE_NAME, move_group->getCurrentPose(), tolerance_pose,
          tolerance_angle);

  moveit_msgs::PositionConstraint &pcm =
      path_constraints.position_constraints[0];
  geometry_msgs::Point center;
  center.x =
      (Trans_W2E.translation()[0] + DesiredMotion.translation()[0]) / 2.0;
  center.y =
      (Trans_W2E.translation()[1] + DesiredMotion.translation()[1]) / 2.0;
  center.z =
      (Trans_W2E.translation()[2] + DesiredMotion.translation()[2]) / 2.0;
  pcm.constraint_region.primitive_poses[0].position = center;
  pcm.constraint_region.primitive_poses[0].orientation =
      move_group->getCurrentPose().pose.orientation;

  // path_constraints.orientation_constraints.push_back(ocm);
  // joint constraint
  path_constraints.joint_constraints.push_back(addJointConstraint(85.0));
  move_group->setPathConstraints(path_constraints);
  Eigen::Isometry3d DesiredMotion1;
  DesiredMotion1.translation() = DesiredMotion.translation();
  DesiredMotion1.linear() = DesiredMotion.rotation();
  move_group->setPoseTarget(DesiredMotion1);
  move_group->setPlanningTime(10.0);
  bool success = move_group->move() ==
                 moveit::planning_interface::MoveItErrorCode::SUCCESS;
  move_group->setPlanningTime(5.0);
  move_group->clearPathConstraints();
  removeDangerDynamicPlanningConstraint();
  return success;
}
bool Manipulator::constantMoveTo(const Eigen::Vector3d &destination,
                                 double velocity_limit) {
  move_group->setMaxVelocityScalingFactor(1.0);
  move_group->setMaxAccelerationScalingFactor(1.0);
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = EE_NAME;
  ocm.header.frame_id = move_group->getPlanningFrame();
  ocm.header.stamp = ros::Time::now();
  ocm.absolute_x_axis_tolerance = 0.0001;
  ocm.absolute_y_axis_tolerance = 0.0001;
  ocm.absolute_z_axis_tolerance = 0.0001;
  ocm.orientation = move_group->getCurrentPose().pose.orientation;
  ocm.weight = 1.0;
  moveit_msgs::Constraints path_constraints;
  path_constraints.orientation_constraints.push_back(ocm);
  path_constraints.joint_constraints.push_back(addJointConstraint(30.0));
  move_group->setPathConstraints(path_constraints);
  {
    Eigen::Affine3d tmp1 = getEndMotion(RIGHT, destination);
    Eigen::Isometry3d tmp2;
    tmp2.linear() = tmp1.rotation();
    tmp2.translation() = tmp1.translation();
    move_group->setPoseTarget(tmp2);
  }
  move_group->setPlanningTime(10.0);
  moveit::planning_interface::MoveGroupInterface::Plan real_plan;
  bool planing_success = move_group->plan(real_plan) ==
                         moveit::planning_interface::MoveItErrorCode::SUCCESS;
  move_group->setPlanningTime(5.0);
  move_group->clearPathConstraints();
  if (!planing_success)
    return false;
  else {
    setAvgCartesianSpeed(real_plan, velocity_limit);
  }

  ros::Time test_before{ros::Time::now()};
  bool a = move_group->execute(real_plan) ==
           moveit::planning_interface::MoveItErrorCode::SUCCESS;
  ROS_INFO_STREAM("THE INTERVAL IS "
                  << (ros::Time::now() - test_before).toSec());
  return a;
}
void Manipulator::updateParameters(int ID) {
  YAML::Node doc = YAML::LoadFile(ros::package::getPath("visual_servo") +
                                  "/config/tagParam.yaml");
  try {
    Parameters.radius = doc["ID" + std::to_string(ID)]["radius"].as<double>();
    Parameters.cutTimes = doc["ID" + std::to_string(ID)]["cutTimes"].as<int>();
    Parameters.cameraXYZ[0] =
        doc["ID" + std::to_string(ID)]["goCameraX"].as<double>();
    Parameters.cameraXYZ[1] =
        doc["ID" + std::to_string(ID)]["goCameraY"].as<double>() +
        Parameters.cutTimes * 0.0015;
    Parameters.cameraXYZ[2] =
        doc["ID" + std::to_string(ID)]["goCameraZ"].as<double>();
    Parameters.compensateP =
        doc["ID" + std::to_string(ID)]["compensateP"].as<double>();
    Parameters.steeringDistance =
        doc["ID" + std::to_string(ID)]["steeringDistance"].as<double>();
  } catch (YAML::InvalidScalar) {
    ROS_ERROR("tagParam.yaml is invalid.");
  }
}
void Manipulator::saveCutTimes(int ID, double cutTimes) {
  if (Parameters.realWork) {
    YAML::Node doc = YAML::LoadFile(ros::package::getPath("visual_servo") +
                                    "/config/tagParam.yaml");
    doc["ID" + std::to_string(ID)]["cutTimes"] = (int)cutTimes;
    std::ofstream output_file(ros::package::getPath("visual_servo") +
                              "/config/tagParam.yaml");
    if (output_file.is_open()) {
      output_file.clear();
      output_file << doc;
      output_file.close();
    }
  }
  if (cutTimes < 1) {
    YAML::Node doc = YAML::LoadFile(ros::package::getPath("visual_servo") +
                                    "/config/tagParam.yaml");
    auto temp_cameraY =
        doc["ID" + std::to_string(ID)]["goCameraY"].as<double>();
    doc["ID" + std::to_string(ID)]["goCameraY"] = temp_cameraY + cutTimes;
    std::ofstream output_file(ros::package::getPath("visual_servo") +
                              "/config/tagParam.yaml");
    if (output_file.is_open()) {
      output_file.clear();
      output_file << doc;
      output_file.close();
    }
  }
}
void Manipulator::resetTool() {
  switch (knife_status) {
    case KNIFE_FORWARD: {
      ros::param::set("/visual_servo/steeringOut", 1.0);
      usleep(800000);
      knife_status = KnifeStatus::KNIFE_BACK;
      // STEERING GO BACK
      ros::param::set("/visual_servo/antiClockGo", 1.0);
      ros::Time start{ros::Time::now()};
      while (true) {
        if ((bool)parameterListener_->parameters()[3]) {
          ros::param::set(PARAMETER_NAMES[3], 0.0);
          knife_status = KnifeStatus::KNIFE_RIGHT;
          break;
        }
        if ((ros::Time::now() - start).toSec() >= 35.0) {
          ROS_ERROR(
              "After 15.0 seconds, the knife can not reach the right position");
          ros::param::set("/visual_servo/knifeUnplug", 1.0);
          break;
        }
        usleep(33333);
      }
      break;
    }
    case KNIFE_OFF: {
      ros::param::set("/visual_servo/antiClockGo", 1.0);
      // check if the knife reach the position, and unplug the knife after 15.0
      // seconds period
      ros::Time start = ros::Time::now();
      while (true) {
        if ((bool)parameterListener_->parameters()[3]) {
          ros::param::set(PARAMETER_NAMES[3], 0.0);
          break;
        }
        if ((ros::Time::now() - start).toSec() >= 35.0) {
          ROS_ERROR(
              "After 15.0 seconds, the knife can not reach the right position");
          ros::param::set("/visual_servo/knifeUnplug", 1.0);
          break;
        }
        usleep(33333);
      }
      knife_status = KnifeStatus::KNIFE_RIGHT;
      break;
    }
    default:
      break;
  }
  knife_status = KnifeStatus ::KNIFE_READY;
}
bool Manipulator::goUp(double velocity_scale) const {
  move_group->setGoalTolerance(Parameters.goal_tolerance);
  move_group->setMaxVelocityScalingFactor(velocity_scale);
  move_group->setMaxAccelerationScalingFactor(1.0);
  if (Parameters.inverse)
    move_group->setNamedTarget("inverseUp");
  else
    move_group->setNamedTarget("up");
  return move_group->move() ==
         moveit::planning_interface::MoveItErrorCode::SUCCESS;
}
bool Manipulator::goHome(double velocity_scale) {
  removeAllDynamicConstraint();
  move_group->setGoalTolerance(Parameters.goal_tolerance);
  move_group->setMaxVelocityScalingFactor(velocity_scale);
  move_group->setMaxAccelerationScalingFactor(1.0);
  std::vector<double> goal;
  if (Parameters.inverse) {
    move_group->setNamedTarget("inverseHome");
    isInverse = true;
  } else {
    move_group->setNamedTarget("home");
    isInverse = false;
  }
  return move_group->move() ==
         moveit::planning_interface::MoveItErrorCode::SUCCESS;
}
double Manipulator::allClose(const std::vector<double> &goal) const {
  std::vector<double> current_joints{move_group->getCurrentJointValues()};

  std::vector<double> auxiliary;

  std::transform(current_joints.begin(), current_joints.end(), goal.begin(),
                 std::back_inserter(auxiliary),
                 [](double element1, double element2) {
                   return pow((element1 - element2), 2);
                 });
  auxiliary.shrink_to_fit();
  return sqrt(std::accumulate(auxiliary.begin(), auxiliary.end(), 0.0));
}
Eigen::Affine3d Manipulator::getEndMotion(MultiplyType multiply_type,
                                          const Eigen::Vector3d &Translation,
                                          const Eigen::Vector3d &RPY) {
  Eigen::Affine3d DesiredMotion;
  DesiredMotion.translation() = Translation;
  DesiredMotion.linear() =
      Eigen::Matrix3d{Eigen::AngleAxisd(RPY[2], Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(RPY[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(RPY[0], Eigen::Vector3d::UnitX())};
  tf2::fromMsg(move_group->getCurrentPose().pose, Trans_W2E);
  // left multiply or right multiply
  switch (multiply_type) {
    case RIGHT:
      return Trans_W2E * DesiredMotion;
    case LEFT:
      return DesiredMotion * Trans_W2E;
    default:
      return Trans_W2E * DesiredMotion;
  }
}
bool Manipulator::goSearchOnce(SearchType search_type, double velocity_scale,
                               double search_angle) {
  move_group->setGoalTolerance(Parameters.goal_tolerance);
  move_group->setMaxVelocityScalingFactor(velocity_scale);
  move_group->setMaxAccelerationScalingFactor(1.0);
  std::vector<double> goal;
  switch (search_type) {
    case FLAT:
      goal = move_group->getCurrentJointValues();
      goal[4] += search_angle;
      move_group->setJointValueTarget(goal);
      if (move_group->move() != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return false;
      goal[4] -= search_angle;
      break;
    case DOWN:
      if (charging) {
        if (!linearMoveTo(
                Eigen::Vector3d(0.0, 0.0, Parameters.chargeDownHeight),
                velocity_scale))
          return false;
        else {
          tag_data_mutex.lock_shared();
          isTagEmpty = Tags_detected.empty();
          tag_data_mutex.unlock_shared();
          return !isTagEmpty;
        }
      } else {
        {
          Eigen::Affine3d tmp1 = getEndMotion(
              RIGHT, Eigen::Vector3d(0.0, 0.0, Parameters.searchDownHeight));
          Eigen::Isometry3d tmp2;
          tmp2.linear() = tmp1.rotation();
          tmp2.translation() = tmp1.translation();
          move_group->setPoseTarget(tmp2);
        }
        if (move_group->move() != moveit_msgs::MoveItErrorCodes::SUCCESS)
          return false;
        else {
          tag_data_mutex.lock_shared();
          isTagEmpty = Tags_detected.empty();
          tag_data_mutex.unlock_shared();
          return !isTagEmpty;
        }
      }
      break;
    case UP:
      //
      {
        Eigen::Affine3d tmp1 =
            getEndMotion(RIGHT, Eigen::Vector3d(0.0, 0.0, 0.0),
                         Eigen::Vector3d(M_PI / 6, 0, 0));
        Eigen::Isometry3d tmp2;
        tmp2.linear() = tmp1.rotation();
        tmp2.translation() = tmp1.translation();
        move_group->setPoseTarget(tmp2);
      }
      if (move_group->move() != moveit_msgs::MoveItErrorCodes::SUCCESS)
        return false;
      goal = move_group->getCurrentJointValues();
      goal[4] += 2 * search_angle;
      break;
    default:
      ROS_ERROR("Wrong search type!!!!");
      return false;
  }
  move_group->setJointValueTarget(goal);
  move_group->setMaxVelocityScalingFactor(0.2);
  move_group->setMaxAccelerationScalingFactor(1.0);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group->plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
    move_group->asyncExecute(my_plan);
  else
    return false;
  double temp_x = 10.0;
  std::vector<double> joint_values{};
  while (RobotAllRight) {
    ROS_INFO_ONCE("Searching tag!!!!!");
    tag_data_mutex.lock_shared();
    isTagEmpty = Tags_detected.empty();
    tag_data_mutex.unlock_shared();
    if (!isTagEmpty) {
      tag_data_mutex.lock_shared();
      if (abs(Tags_detected[0].Trans_C2T.translation()[0]) < temp_x) {
        temp_x = abs(Tags_detected[0].Trans_C2T.translation()[0]);
        joint_values = move_group->getCurrentJointValues();
      }
      tag_data_mutex.unlock_shared();
    }
    if (allClose(goal) <= Parameters.goal_tolerance * 3) {
      ROS_INFO("finished");
      if (temp_x > 0.2)
        return false;
      else
        break;
    }
    // a little bit slower than tag frequency
    usleep(40000);
  }
  ROS_INFO("Already remembered the most center joint values");
  // wait until the robot finished the asyncMove task, can't be 0 because of
  // goal_tolerance
  while (isRobotMoving)
    ;
  move_group->setMaxVelocityScalingFactor(1.0);
  move_group->setMaxAccelerationScalingFactor(1.0);
  move_group->setJointValueTarget(joint_values);
  return (move_group->move() == moveit_msgs::MoveItErrorCodes::SUCCESS);
}
bool Manipulator::goSearch(double velocity_scale, bool underServoing,
                           double search_angle) {
  if (!underServoing) {
    if (goSearchOnce(DOWN, velocity_scale, search_angle)) return true;
  }
  return goSearchOnce(FLAT, velocity_scale, search_angle);
}
bool Manipulator::goServo(double velocity_scale) {
  tf2::fromMsg(move_group->getCurrentPose().pose, Trans_W2E);
  move_group->setPlanningTime(10.0);
  double error = 100.0;
  Destination_t EndDestination;
  static bool tag_empty{true};
  move_group->setGoalTolerance(Parameters.goal_tolerance);
  move_group->setMaxVelocityScalingFactor(velocity_scale);
  move_group->setMaxAccelerationScalingFactor(1.0);
  moveit_msgs::Constraints path_constraints;
  path_constraints.joint_constraints.push_back(addJointConstraint(85.0));
  path_constraints.joint_constraints.push_back(addJointConstraint(90, 2));
  move_group->setPathConstraints(path_constraints);
  while (RobotAllRight && error > Parameters.servoTolerance) {
    tag_data_mutex.lock_shared();
    isTagEmpty = Tags_detected.empty();
    tag_data_mutex.unlock_shared();
    if (!isTagEmpty || goSearch(Parameters.basicVelocity, true)) {
      tag_empty = false;
      addDynamicPlanningConstraint(false, true);
      tag_data_mutex.lock_shared();
      EndDestination = astra->getCameraEE(Tags_detected[0].Trans_C2T, Trans_E2C,
                                          ExpectMatrix);
      tag_data_mutex.unlock_shared();
      error = EndDestination.error;

      if (!Parameters.servoPoseOn)
        EndDestination.EE_Motion.linear() = Eigen::Matrix3d::Identity();
      {
        Eigen::Affine3d tmp1 = Trans_W2E * EndDestination.EE_Motion;
        Eigen::Isometry3d tmp2;
        tmp2.linear() = tmp1.rotation();
        tmp2.translation() = tmp1.translation();
        move_group->setPoseTarget(tmp2);
      }
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group->plan(my_plan) ==
                      moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success) {
        // ROS_INFO("Reachable");
        if (move_group->execute(my_plan) !=
            moveit::planning_interface::MoveItErrorCode::SUCCESS) {
          move_group->clearPathConstraints();
          return false;
        }
        // wait for physic vibration
        usleep(200000);
        // sleep(1);
      } else {
        ROS_ERROR("Failed");
        move_group->clearPathConstraints();
        return false;
      }
      tf2::fromMsg(move_group->getCurrentPose().pose, Trans_W2E);
    } else {
      // in case of light influence
      static ros::Time start_servo{ros::Time::now()};
      if (!tag_empty) start_servo = ros::Time::now();
      tag_empty = true;
      if ((ros::Time::now() - start_servo).toSec() > 2.0) {
        move_group->clearPathConstraints();
        return false;
      } else
        continue;
    }
  }
  // compensate

  if (Parameters.compensateP == 0.0) {
    // new tag, then save the initial value
    Parameters.compensateP = EndDestination.error_log[4];
    YAML::Node doc = YAML::LoadFile(ros::package::getPath("visual_servo") +
                                    "/config/tagParam.yaml");
    doc["ID" + std::to_string(Parameters.ID)]["compensateP"] =
        Parameters.compensateP;
    std::fstream output_file(ros::package::getPath("visual_servo") +
                             "/config/tagParam.yaml");
    if (output_file.is_open()) output_file << doc;
    output_file.close();
  }
  {
    Eigen::Affine3d tmp1 =
        getEndMotion(RIGHT, Eigen::Vector3d(0.0, 0.0, 0.0),
                     Eigen::Vector3d(0.0, Parameters.compensateP, 0.0));
    Eigen::Isometry3d tmp2;
    tmp2.linear() = tmp1.rotation();
    tmp2.translation() = tmp1.translation();
    move_group->setPoseTarget(tmp2);
  }
  ROS_INFO_STREAM(move_group->getCurrentPose());
  bool result = (RobotAllRight &&
                 move_group->move() ==
                     moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group->clearPathConstraints();
  move_group->setPlanningTime(5.0);
  return result;
}
bool Manipulator::goCut(double velocity_scale) {
  knife_status = KnifeStatus::KNIFE_READY;
  bool success{true};
  // clear all sign
  ros::param::set(PARAMETER_NAMES[2], 0.0);
  ros::param::set(PARAMETER_NAMES[3], 0.0);
  // ros::param::set("/visual_servo/antiClockGo", 1.0);
  std::cout << "visual servo anticlockgo" << std::endl;
  usleep(20000);
  // check the right switch
  ros::param::set("/visual_servo/getSwitch", 0.0);
  ros::Time start{ros::Time::now()};
  while (true) {
    if ((bool)parameterListener_->parameters()[3]) {
      ros::param::set(PARAMETER_NAMES[3], 0.0);
      knife_status = KnifeStatus::KNIFE_RIGHT;
      break;
    }
    if ((ros::Time::now() - start).toSec() >= 35.0) {
      ROS_ERROR(
          "After 15.0 seconds, the knife can not reach the right position");
      ros::param::set("/visual_servo/knifeUnplug", 1.0);
      return linearMoveTo(Eigen::Vector3d(0.0, 0.25, 0.0), 0.5);
    }
    usleep(33333);
  }
  // knife go down
  if (!linearMoveTo(Eigen::Vector3d(0.0, 0.0, -0.0015), 1.0)) return false;
  usleep(200000);
  // knife on
  ros::param::set("/visual_servo/knifeOn", 1.0);
  sleep(1);
  knife_status = KnifeStatus::KNIFE_ON;
  // debug test 1221
  {
    std_msgs::Int8 msg;
    msg.data = 303;
    debug_test_pub_.publish(msg);
  }
  // knife go forward
  ros::param::set("/visual_servo/steeringIn", Parameters.steeringDistance);
  sleep(1);
  knife_status = KnifeStatus::KNIFE_FORWARD;
  // knife ready to go anti-clock-wise entire circle
  // ros::param::set("/visual_servo/clockGo", 1.0);
  std::cout << "visual servo clockgo" << std::endl;

  start = ros::Time::now();
  while (true) {
    // ros::param::set("/visual_servo/getSwitch", 0.0);
    if ((bool)parameterListener_->parameters()[2]) {
      ros::param::set(PARAMETER_NAMES[2], 0.0);
      knife_status = KnifeStatus::KNIFE_LEFT;
      break;
    }
    if ((ros::Time::now() - start).toSec() >= 13.0) {  // default 35
      ROS_ERROR(
          "After 15.0 seconds, the knife can not reach the right position");
      ros::param::set("/visual_servo/knifeUnplug", 1.0);
      break;
    }
    usleep(33333);
  }
  // knife go back
  ros::param::set("/visual_servo/steeringOut", 1.0);
  sleep(1);
  knife_status = KnifeStatus::KNIFE_BACK;
  // knife off
  ros::param::set("/visual_servo/knifeOff", 1.0);
  knife_status = KnifeStatus::KNIFE_OFF;
  // try check the clock go
  // ros::param::set("/visual_servo/antiClockGo", 1.0);
  usleep(500000);
  knife_status = KnifeStatus::KNIFE_RIGHT;
  /*
  //knife go left
   ros::param::set("/visual_servo/clockGo",1.0);
//check if the knife reach the position, and unplug the knife after 15.0 seconds
period start=ros::Time::now(); while(true)
{
  if((bool) parameterListener_->parameters()[2])
  {
          ros::param::set(PARAMETER_NAMES[2],0.0);
      break;
   }
  if((ros::Time::now()-start).toSec()>=35.0)
  {
          ROS_ERROR("After 15.0 seconds, the knife can not reach the right
position"); ros::param::set("/visual_servo/knifeUnplug",1.0); return
linearMoveTo(Eigen::Vector3d(0.0,0.25,0.0),0.5);
  }
  usleep(33333);
}
knife_status=KnifeStatus::KNIFE_LEFT;*/
  // knife go shallow;
  if (!linearMoveTo(Eigen::Vector3d(0.0, 0.3, 0.0), 0.5)) {
    linearMoveTo(Eigen::Vector3d(0.0, 0.0, 0.1), 0.5);
    success = linearMoveTo(Eigen::Vector3d(0.0, 0.25, 0.0), 0.5);
  }
  addDynamicPlanningConstraint(false, true);
  return success;
}
bool Manipulator::goCamera(const Eigen::Vector3d &target_array,
                           double velocity_scale) {
  addDynamicPlanningConstraint(false, true, true);
  move_group->clearPathConstraints();
  move_group->setGoalTolerance(Parameters.goal_tolerance);
  move_group->setMaxVelocityScalingFactor(velocity_scale);
  move_group->setMaxAccelerationScalingFactor(1.0);
  tf2::fromMsg(move_group->getCurrentPose().pose, Trans_W2E);

  Trans_W2EP.translation() = Trans_E2C * target_array;
  Trans_W2EP.linear() = Eigen::Matrix3d::Identity();
  // constraints
  move_group->setPlanningTime(10.0);
  moveit_msgs::Constraints path_constraints;
  path_constraints.joint_constraints.push_back(addJointConstraint(60.0));
  path_constraints.joint_constraints.push_back(addJointConstraint(75.0, 1));
  move_group->setPathConstraints(path_constraints);
  {
    Eigen::Affine3d tmp1 = Trans_W2E * Trans_W2EP;
    Eigen::Isometry3d tmp2;
    tmp2.linear() = tmp1.rotation();
    tmp2.translation() = tmp1.translation();
    move_group->setPoseTarget(tmp2);
  }
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  if (move_group->plan(my_plan) ==
      moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_INFO("Naive Reachable");
    move_group->clearPathConstraints();
    return move_group->execute(my_plan) ==
           moveit::planning_interface::MoveItErrorCode::SUCCESS;
  } else {
    ROS_INFO("FAILED");
    move_group->clearPathConstraints();
    return false;
  }
}
Eigen::Affine3d Manipulator::getTagPosition(
    const Eigen::Affine3d &Trans_C2T) const {
  Eigen::Affine3d Trans_B2E;
  tf2::fromMsg(move_group->getCurrentPose().pose, Trans_B2E);
  return Trans_B2E * Trans_E2C * Trans_C2T;
}
bool Manipulator::goCharge(double velocity_scale) {
  move_group->setGoalTolerance(Parameters.goal_tolerance);
  move_group->setMaxVelocityScalingFactor(velocity_scale);
  move_group->setMaxAccelerationScalingFactor(1.0);
  tag_data_mutex.lock_shared();
  Eigen::Affine3d Trans_B2T{getTagPosition(Tags_detected[0].Trans_C2T)};
  tag_data_mutex.unlock_shared();
  Trans_B2T.translation() += Eigen::Vector3d(
      boost::math::sign(Trans_B2T.translation()[1]) * 0.087, -0.015, -0.150);
  Eigen::Affine3d tempMatrix;
  tempMatrix.linear() =
      Eigen::Matrix3d{Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())};
  tempMatrix.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
  Trans_W2EP = Trans_B2T * tempMatrix * Trans_E2CH.inverse();
  {
    Eigen::Affine3d tmp1 = Trans_W2EP;
    Eigen::Isometry3d tmp2;
    tmp2.linear() = tmp1.rotation();
    tmp2.translation() = tmp1.translation();
    move_group->setPoseTarget(tmp2);
  }
  if (move_group->move() !=
      moveit::planning_interface::MoveItErrorCode::SUCCESS)
    return false;

  return (linearMoveTo(Eigen::Vector3d(0.035, 0.00, 0.00), velocity_scale));
}
bool Manipulator::leaveCharge(double velocity_scale) {
  if (!linearMoveTo(Eigen::Vector3d(-0.10, 0.00, 0.00), velocity_scale))
    return false;
  move_group->setNamedTarget("up");
  return (move_group->move() ==
          moveit::planning_interface::MoveItErrorCode::SUCCESS);
}
int Manipulator::executeService(int serviceType) {
  // boost::shared_lock<boost::shared_mutex> read_lock(tag_data_mutex);
  running_ = true;
  int serviceStatus;
  switch (serviceType) {
    case visual_servo::manipulate::Request::CUT:
      if (isInverse) {
        ros::param::set("/user/inverse", 0.0);
        Parameters.inverse = false;
        if (!goHome(Parameters.basicVelocity)) {
          isInverse = true;
          serviceStatus = visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
          break;
        } else
          isInverse = false;
      }
      goUp(1.0);
      // debug test 1221
      {
        std_msgs::Int8 msg;
        msg.data = 301;
        debug_test_pub_.publish(msg);
      }
      tag_data_mutex.lock_shared();
      isTagEmpty = Tags_detected.empty();
      tag_data_mutex.unlock_shared();
      if (isTagEmpty) {
        if (!goSearch(Parameters.basicVelocity)) {
          ROS_INFO("!!!!!!NO TAG SEARCHED!!!!!");
          serviceStatus = visual_servo_namespace::SERVICE_STATUS_NO_TAG;
          goUp(Parameters.basicVelocity);
          if (!goHome(Parameters.basicVelocity))
            serviceStatus = visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
        } else {
          tag_data_mutex.lock_shared();
          Parameters.ID = Tags_detected[0].id;
          tag_data_mutex.unlock_shared();
          updateParameters(Parameters.ID);
          std::string serial_send{"Processing the " +
                                  std::to_string(Parameters.ID) + " th tree"};
          communicator_->send(serial_send.c_str(), serial_send.size());
          // logPublish("INFO: "+serial_send);
          if (!goServo(Parameters.basicVelocity)) {
            serviceStatus = visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED;
            goUp(Parameters.basicVelocity);
            if (!goHome(Parameters.basicVelocity))
              serviceStatus =
                  visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
          } else {
            // addDynamicPlanningConstraint(false,true);
            tag_data_mutex.lock_shared();
            Eigen::Vector3d Center3d{Tags_detected[0].Trans_C2T.translation()};
            tag_data_mutex.unlock_shared();
            ros::Time start = ros::Time::now();
            int times = 1;
            while ((ros::Time::now() - start).toSec() < 1.0) {
              times++;
              tag_data_mutex.lock_shared();
              Center3d += Tags_detected[0].Trans_C2T.translation();
              tag_data_mutex.unlock_shared();
              usleep(50000);
            }
            Center3d /= times;
            Center3d += Parameters.cameraXYZ;
            // logPublish("INFO: the received cameraxYZ is
            // "+std::to_string(Parameters.cameraXYZ[0])+","+std::to_string(Parameters.cameraXYZ[1])+","+std::to_string(Parameters.cameraXYZ[2]));
            // enable cut
            bool real_cut = false;
            if (real_cut) {
              if (!goCamera(Center3d, Parameters.basicVelocity)) {
                serviceStatus =
                    visual_servo_namespace::SERVICE_STATUS_CLOSE_FAILED;
                goUp(Parameters.basicVelocity);
                if (!goHome(Parameters.basicVelocity))
                  serviceStatus =
                      visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
              } else {
                addDynamicPlanningConstraint(true);
                if (linearMoveTo(Eigen::Vector3d(0.0, -0.3, 0.0), 0.5)) {
                  serviceStatus =
                      visual_servo_namespace::SERVICE_STATUS_CUTIN_SUCCEED;
                  if (!Parameters.debugOn) {
                    if (!goCut(Parameters.basicVelocity))
                      serviceStatus =
                          visual_servo_namespace::SERVICE_STATUS_CUT_FAILED;
                    else {
                      serviceStatus =
                          visual_servo_namespace::SERVICE_STATUS_SUCCEED;
                      saveCutTimes(Parameters.ID, Parameters.cutTimes + 1);
                    }
                    goUp(Parameters.basicVelocity);
                    if (!goHome(Parameters.basicVelocity))
                      serviceStatus =
                          visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
                  }
                } else {
                  serviceStatus =
                      visual_servo_namespace::SERVICE_STATUS_CUT_FAILED;
                  goUp(Parameters.basicVelocity);
                  if (!goHome(Parameters.basicVelocity))
                    serviceStatus =
                        visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
                }
              }
            } else {
              goUp(Parameters.basicVelocity);
              if (!goHome(Parameters.basicVelocity))
                serviceStatus =
                    visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
            }
          }
        }
      } else {
        tag_data_mutex.lock_shared();
        Parameters.ID = Tags_detected[0].id;
        tag_data_mutex.unlock_shared();
        updateParameters(Parameters.ID);
        std::string serial_send{"Processing the " +
                                std::to_string(Parameters.ID) + " th tree"};
        communicator_->send(serial_send.c_str(), serial_send.size());
        // logPublish("INFO: "+serial_send);
        if (!goServo(Parameters.basicVelocity)) {
          serviceStatus = visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED;
          goUp(Parameters.basicVelocity);
          if (!goHome(Parameters.basicVelocity))
            serviceStatus = visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
        } else {
          // debug test 1221
          {
            std_msgs::Int8 msg;
            msg.data = 302;
            debug_test_pub_.publish(msg);
          }
          // addDynamicPlanningConstraint(false,true);
          tag_data_mutex.lock_shared();
          Eigen::Vector3d Center3d{Tags_detected[0].Trans_C2T.translation()};
          tag_data_mutex.unlock_shared();
          ros::Time start = ros::Time::now();
          int times = 1;
          while ((ros::Time::now() - start).toSec() < 1.0) {
            times++;
            tag_data_mutex.lock_shared();
            Center3d += Tags_detected[0].Trans_C2T.translation();
            tag_data_mutex.unlock_shared();
            usleep(50000);
          }
          Center3d /= times;
          Center3d += Parameters.cameraXYZ;
          // logPublish("INFO: the received cameraxYZ is
          // "+std::to_string(Parameters.cameraXYZ[0])+","+std::to_string(Parameters.cameraXYZ[1])+","+std::to_string(Parameters.cameraXYZ[2]));
          if (!goCamera(Center3d, Parameters.basicVelocity)) {
            serviceStatus = visual_servo_namespace::SERVICE_STATUS_CLOSE_FAILED;
            goUp(Parameters.basicVelocity);
            if (!goHome(Parameters.basicVelocity))
              serviceStatus =
                  visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
          } else {
            addDynamicPlanningConstraint(true);
            if (linearMoveTo(Eigen::Vector3d(0.0, -0.3, 0.0), 0.5)) {
              serviceStatus =
                  visual_servo_namespace::SERVICE_STATUS_CUTIN_SUCCEED;
              if (!Parameters.debugOn) {
                if (!goCut(Parameters.basicVelocity))
                  serviceStatus =
                      visual_servo_namespace::SERVICE_STATUS_CUT_FAILED;
                else {
                  serviceStatus =
                      visual_servo_namespace::SERVICE_STATUS_SUCCEED;
                  saveCutTimes(Parameters.ID, Parameters.cutTimes + 1);
                }
                goUp(Parameters.basicVelocity);
                if (!goHome(Parameters.basicVelocity))
                  serviceStatus =
                      visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
              }
            } else {
              serviceStatus = visual_servo_namespace::SERVICE_STATUS_CUT_FAILED;
              goUp(Parameters.basicVelocity);
              if (!goHome(Parameters.basicVelocity))
                serviceStatus =
                    visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
            }
          }
        }
      }
      break;
    case visual_servo::manipulate::Request::CUTBACK:
      if (!goCut(Parameters.basicVelocity))
        serviceStatus = visual_servo_namespace::SERVICE_STATUS_CUT_FAILED;
      else {
        serviceStatus = visual_servo_namespace::SERVICE_STATUS_SUCCEED;
        saveCutTimes(Parameters.ID, Parameters.cutTimes + 1);
      }
      goUp(Parameters.basicVelocity);
      if (!goHome(Parameters.basicVelocity))
        serviceStatus = visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
      break;
    case visual_servo::manipulate::Request::SEARCH:
      goUp(0.8);
      serviceStatus = goSearch(Parameters.basicVelocity)
                          ? visual_servo_namespace::SERVICE_STATUS_SUCCEED
                          : visual_servo_namespace::SERVICE_STATUS_NO_TAG;
      break;
    case visual_servo::manipulate::Request::CHARGE:
      if (!isInverse) {
        ros::param::set("/user/inverse", 1.0);
        Parameters.inverse = true;
        if (!goHome(Parameters.basicVelocity)) {
          isInverse = false;
          serviceStatus = visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
          break;
        } else
          isInverse = true;
      }
      goUp(1.0);
      tag_data_mutex.lock_shared();
      isTagEmpty = Tags_detected.empty();
      tag_data_mutex.unlock_shared();
      if (isTagEmpty) {
        if (!goSearch(Parameters.basicVelocity)) {
          ROS_INFO("!!!!!!NO TAG SEARCHED!!!!!");
          serviceStatus = visual_servo_namespace::SERVICE_STATUS_NO_TAG;
          goUp(Parameters.basicVelocity);
          if (!goHome(Parameters.basicVelocity))
            serviceStatus = visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
        } else {
          tag_data_mutex.lock_shared();
          Parameters.ID = Tags_detected[0].id;
          tag_data_mutex.unlock_shared();
          updateParameters(Parameters.ID);
          std::string serial_send{"Processing the " +
                                  std::to_string(Parameters.ID) + " th tree"};
          communicator_->send(serial_send.c_str(), serial_send.size());
          // logPublish("INFO: "+serial_send);
          if (!goServo(Parameters.basicVelocity)) {
            serviceStatus = visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED;
            goUp(Parameters.basicVelocity);
            if (!goHome(Parameters.basicVelocity))
              serviceStatus =
                  visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
          } else {
            // addDynamicPlanningConstraint(false,true);
            tag_data_mutex.lock_shared();
            Eigen::Vector3d Center3d{Tags_detected[0].Trans_C2T.translation()};
            tag_data_mutex.unlock_shared();
            ros::Time start = ros::Time::now();
            int times = 1;
            while ((ros::Time::now() - start).toSec() < 1.0) {
              times++;
              tag_data_mutex.lock_shared();
              Center3d += Tags_detected[0].Trans_C2T.translation();
              tag_data_mutex.unlock_shared();
              usleep(50000);
            }
            Center3d /= times;
            Center3d += Parameters.cameraXYZ;
            // logPublish("INFO: the received cameraxYZ is
            // "+std::to_string(Parameters.cameraXYZ[0])+","+std::to_string(Parameters.cameraXYZ[1])+","+std::to_string(Parameters.cameraXYZ[2]));
            // enable cut
            bool real_cut = false;
            if (real_cut) {
              if (!goCamera(Center3d, Parameters.basicVelocity)) {
                serviceStatus =
                    visual_servo_namespace::SERVICE_STATUS_CLOSE_FAILED;
                goUp(Parameters.basicVelocity);
                if (!goHome(Parameters.basicVelocity))
                  serviceStatus =
                      visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
              } else {
                addDynamicPlanningConstraint(true);
                if (linearMoveTo(Eigen::Vector3d(0.0, -0.3, 0.0), 0.5)) {
                  serviceStatus =
                      visual_servo_namespace::SERVICE_STATUS_CUTIN_SUCCEED;
                  if (!Parameters.debugOn) {
                    if (!goCut(Parameters.basicVelocity))
                      serviceStatus =
                          visual_servo_namespace::SERVICE_STATUS_CUT_FAILED;
                    else {
                      serviceStatus =
                          visual_servo_namespace::SERVICE_STATUS_SUCCEED;
                      saveCutTimes(Parameters.ID, Parameters.cutTimes + 1);
                    }
                    goUp(Parameters.basicVelocity);
                    if (!goHome(Parameters.basicVelocity))
                      serviceStatus =
                          visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
                  }
                } else {
                  serviceStatus =
                      visual_servo_namespace::SERVICE_STATUS_CUT_FAILED;
                  goUp(Parameters.basicVelocity);
                  if (!goHome(Parameters.basicVelocity))
                    serviceStatus =
                        visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
                }
              }
            } else {
              goUp(Parameters.basicVelocity);
              if (!goHome(Parameters.basicVelocity))
                serviceStatus =
                    visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
            }
          }
        }
      } else {
        tag_data_mutex.lock_shared();
        Parameters.ID = Tags_detected[0].id;
        tag_data_mutex.unlock_shared();
        updateParameters(Parameters.ID);
        std::string serial_send{"Processing the " +
                                std::to_string(Parameters.ID) + " th tree"};
        communicator_->send(serial_send.c_str(), serial_send.size());
        // logPublish("INFO: "+serial_send);
        if (!goServo(Parameters.basicVelocity)) {
          serviceStatus = visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED;
          goUp(Parameters.basicVelocity);
          if (!goHome(Parameters.basicVelocity))
            serviceStatus = visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
        } else {
          // addDynamicPlanningConstraint(false,true);
          tag_data_mutex.lock_shared();
          Eigen::Vector3d Center3d{Tags_detected[0].Trans_C2T.translation()};
          tag_data_mutex.unlock_shared();
          ros::Time start = ros::Time::now();
          int times = 1;
          while ((ros::Time::now() - start).toSec() < 1.0) {
            times++;
            tag_data_mutex.lock_shared();
            Center3d += Tags_detected[0].Trans_C2T.translation();
            tag_data_mutex.unlock_shared();
            usleep(50000);
          }
          Center3d /= times;
          Center3d += Parameters.cameraXYZ;
          // logPublish("INFO: the received cameraxYZ is
          // "+std::to_string(Parameters.cameraXYZ[0])+","+std::to_string(Parameters.cameraXYZ[1])+","+std::to_string(Parameters.cameraXYZ[2]));
          if (!goCamera(Center3d, Parameters.basicVelocity)) {
            serviceStatus = visual_servo_namespace::SERVICE_STATUS_CLOSE_FAILED;
            goUp(Parameters.basicVelocity);
            if (!goHome(Parameters.basicVelocity))
              serviceStatus =
                  visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
          } else {
            addDynamicPlanningConstraint(true);
            if (linearMoveTo(Eigen::Vector3d(0.0, -0.3, 0.0), 0.5)) {
              serviceStatus =
                  visual_servo_namespace::SERVICE_STATUS_CUTIN_SUCCEED;
              if (!Parameters.debugOn) {
                if (!goCut(Parameters.basicVelocity))
                  serviceStatus =
                      visual_servo_namespace::SERVICE_STATUS_CUT_FAILED;
                else {
                  serviceStatus =
                      visual_servo_namespace::SERVICE_STATUS_SUCCEED;
                  saveCutTimes(Parameters.ID, Parameters.cutTimes + 1);
                }
                goUp(Parameters.basicVelocity);
                if (!goHome(Parameters.basicVelocity))
                  serviceStatus =
                      visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
              }
            } else {
              serviceStatus = visual_servo_namespace::SERVICE_STATUS_CUT_FAILED;
              goUp(Parameters.basicVelocity);
              if (!goHome(Parameters.basicVelocity))
                serviceStatus =
                    visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
            }
          }
        }
      }
      break;
      // charging = true;
      // Parameters.inverse = true;
      // goUp(1.0);
      // tag_data_mutex.lock_shared();
      // isTagEmpty = Tags_detected.empty();
      // tag_data_mutex.unlock_shared();
      // if (isTagEmpty) {
      //   if (!goSearch(0.5)) {
      //     ROS_INFO("!!!!!!NO TAG SEARCHED!!!!!");
      //     serviceStatus = visual_servo_namespace::SERVICE_STATUS_NO_TAG;
      //     goUp(Parameters.basicVelocity);
      //     goHome(Parameters.basicVelocity);
      //   } else {
      //     addChargerDynamicPlanningConstraint();
      //     bool servo_success = goServo(Parameters.basicVelocity);
      //     if (!servo_success)
      //       serviceStatus =
      //       visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED;
      //     else {
      //       if (!goCharge(Parameters.basicVelocity))
      //         serviceStatus =
      //             visual_servo_namespace::SERVICE_STATUS_CHARGE_FAILED;
      //       else {
      //         sleep(20);
      //         serviceStatus =
      //             leaveCharge(Parameters.basicVelocity)
      //                 ? visual_servo_namespace::SERVICE_STATUS_SUCCEED
      //                 : visual_servo_namespace::
      //                       SERVICE_STATUS_LEAVE_CHARGE_FAILED;
      //       }
      //     }
      // goHome(Parameters.basicVelocity);
      //   }
      // }
      // else {
      //   addChargerDynamicPlanningConstraint();
      //   bool servo_success = goServo(Parameters.basicVelocity);
      //   if (!servo_success)
      //     serviceStatus =
      //     visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED;
      //   else {
      //     if (!goCharge(Parameters.basicVelocity))
      //       serviceStatus =
      //       visual_servo_namespace::SERVICE_STATUS_CHARGE_FAILED;
      //     else {
      //       // to do charge proto
      //       ros::param::set("/visual_servo/isChargingStatusChanged",
      //       (double)true); sleep(20);
      //       ros::param::set("/visual_servo/isChargingStatusChanged",
      //       (double)true); serviceStatus =
      //           leaveCharge(Parameters.basicVelocity)
      //               ? visual_servo_namespace::SERVICE_STATUS_SUCCEED
      //               :
      //               visual_servo_namespace::SERVICE_STATUS_LEAVE_CHARGE_FAILED;
      //     }
      //   }
      //   goHome(Parameters.basicVelocity);
      // }
      // charging = false;
      // break;
    case visual_servo::manipulate::Request::UP:
      serviceStatus = goUp(Parameters.basicVelocity)
                          ? visual_servo_namespace::SERVICE_STATUS_SUCCEED
                          : visual_servo_namespace::SERVICE_STATUS_UP_FAILED;
      break;
    case visual_servo::manipulate::Request::HOME:
      serviceStatus = goHome(Parameters.basicVelocity)
                          ? visual_servo_namespace::SERVICE_STATUS_SUCCEED
                          : visual_servo_namespace::SERVICE_STATUS_HOME_FAILED;
      break;
    case visual_servo::manipulate::Request::LINEAR:
      if (linearMoveTo(ManipulateSrv.transformation, 0.3)) {
        serviceStatus = visual_servo_namespace::SERVICE_STATUS_CUTIN_SUCCEED;
        if (ManipulateSrv.transformation[2] != 0)
          saveCutTimes(Parameters.ID, -1 * ManipulateSrv.transformation[2]);
      } else
        serviceStatus = visual_servo_namespace::SERVICE_STATUS_LINEAR_FAILED;
      break;
    case visual_servo::manipulate::Request::EMPTY:
      ROS_INFO("EMPTY SERVICE");
      // sleep(3);
      serviceStatus = visual_servo_namespace::SERVICE_STATUS_EMPTY;
      break;
    default:
      serviceStatus = visual_servo_namespace::SERVICE_STATUS_EMPTY;
      break;
  }
  if (!RobotAllRight)
    serviceStatus = visual_servo_namespace::SERVICE_STATUS_ROBOT_ABORT;
  ros::param::set("/visual/tagDetectorOn", false);
  usleep(500000);
  ros::param::set(PARAMETER_NAMES[2], 0.0);
  running_ = false;
  std::string serial_send{
      visual_servo_namespace::getServiceStatusString(serviceStatus)};
  communicator_->send(serial_send.c_str(), serial_send.size());
  // logPublish("INFO:
  // "+visual_servo_namespace::getServiceStatusString(serviceStatus));
  return serviceStatus;
}

Listener::Listener() {
  this->tag_info_received = false;
  tag_info_sub =
      nh.subscribe("TagsDetected", 1000, &Listener::tagInfoCallback, this);
  vs_status_sub =
      nh.subscribe("VisualServoStatus", 100, &Listener::statusCallback, this);
  joint_state_sub_ =
      nh.subscribe("joint_states", 100, &Listener::jointStateCallback, this);
  manipulate_callback_t manipulate_callback =
      boost::bind(&Listener::manipulate, this, _1, _2);
  service_ = nh.advertiseService("manipulate", manipulate_callback);
  watchdog_timer_ = nh.createTimer(ros::Duration(WATCHDOG_PERIOD_),
                                   &Listener::watchdog, this, true);
}
Listener::~Listener() { watchdog_timer_.stop(); }

void Listener::tagInfoCallback(const visual_servo::TagsDetection_msg &msg) {
  watchdog_timer_.stop();
  watchdog_timer_.start();
  boost::unique_lock<boost::shared_mutex> write_lock(tag_data_mutex);
  this->tag_info_received = true;
  TagDetectInfo Tag_detected;
  Tags_detected.clear();
  for (auto &tag_information : msg.tags_information) {
    tf2::fromMsg(tag_information.pose, Tag_detected.Trans_C2T);
    Tag_detected.PixelCoef = tag_information.PixelCoef;
    Tag_detected.Center.x = tag_information.center.x;
    Tag_detected.Center.y = tag_information.center.y;
    Tag_detected.id = tag_information.id;
    Tags_detected.push_back(Tag_detected);
  }
}
void Listener::statusCallback(const visual_servo::VisualServoMetaTypeMsg &msg) {
  RobotAllRight = msg.RobotAllRight;
  // std::cout<<"RobotAllRight: "<<(int) RobotAllRight<<"RobotSwitchOn: "<<
  // (int) RobotSwitchOn<<std::endl;
}
void Listener::jointStateCallback(const sensor_msgs::JointState &msg) {
  static bool first_time = true;
  if (first_time) {
    last_received_state_ = msg;
    first_time = false;
  }
  std::vector<double> current_joints(msg.position.size());
  memcpy(current_joints.data(), msg.position.data(),
         sizeof(double) * msg.position.size());
  std::vector<double> last_joints(msg.position.size());
  memcpy(last_joints.data(), last_received_state_.position.data(),
         sizeof(double) * last_received_state_.position.size());
  std::vector<double> auxiliary;

  std::transform(current_joints.begin(), current_joints.end(),
                 last_joints.begin(), std::back_inserter(auxiliary),
                 [](double element1, double element2) {
                   return pow((element1 - element2), 2);
                 });
  auxiliary.shrink_to_fit();

  isRobotMoving =
      sqrt(std::accumulate(auxiliary.begin(), auxiliary.end(), 0.0)) > 0.001;
  last_received_state_ = msg;
}
bool Listener::manipulate(visual_servo::manipulate::Request &req,
                          visual_servo::manipulate::Response &res) {
  manipulate_srv_on = true;
  ManipulateSrv.srv_type = req.type;
  if (!req.transformation.empty()) {
    ManipulateSrv.transformation[0] = req.transformation[0];
    ManipulateSrv.transformation[1] = req.transformation[1];
    ManipulateSrv.transformation[2] = req.transformation[2];
  }
  ros::Time temp;
  while (manipulate_srv_on) {
    /*Warning: Do not delete this line, or it may can not break out*/
    temp = ros::Time::now();
    usleep(500000);
  }
  res.status = ManipulateSrv.srv_status;
  res.status_string =
      visual_servo_namespace::getServiceStatusString(ManipulateSrv.srv_status);
  ManipulateSrv.srv_status = visual_servo_namespace::SERVICE_STATUS_EMPTY;
  ManipulateSrv.srv_type = visual_servo::manipulate::Request::EMPTY;
  return true;
}
void Listener::watchdog(const ros::TimerEvent &e) {
  ROS_WARN("tag info not received for %f seconds, is the camera node drop?",
           WATCHDOG_PERIOD_);
  this->tag_info_received = false;
}
