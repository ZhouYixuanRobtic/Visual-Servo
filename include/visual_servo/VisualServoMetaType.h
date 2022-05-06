//
// Created by xcy on 19-6-27.
//

#ifndef VISUAL_SERVO_VISUALSERVOMETATYPE_H
#define VISUAL_SERVO_VISUALSERVOMETATYPE_H

#include <unistd.h>

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <sstream>
#include <string>
#include <utility>

#include "Eigen/Dense"
#include "Eigen/Geometry"
// #include "cv.h"
#include <atomic>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/thread.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ros/ros.h"
#include "tr1/memory"
#include "visual_servo/TagDetection_msg.h"
#include "visual_servo/TagsDetection_msg.h"
#include "visual_servo/VisualServoMetaTypeMsg.h"
#include "visual_servo/detect_once.h"
#include "visual_servo/manipulate.h"
/**
 * General types
 */
typedef uint8_t boolean;
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int64_t int64;
typedef uint64_t uint64;
typedef float float32;
typedef double float64;

struct TagDetectInfo {
  // the tag pose with respect to camera described as a homogeneous matrix
  Eigen::Affine3d Trans_C2T;
  // the tag id
  int id;
  // the coefficient unit meters per pixel
  double PixelCoef;
  // the pixel point of tag center
  cv::Point2d Center;
};

struct Destination_t {
  // the end effector motion described as a homogeneous matrix
  Eigen::Affine3d EE_Motion;
  // the position error with respect to expected position
  double error;
  Eigen::Matrix<double, 6, 1> error_log;
};

struct manipulateSrv {
  int srv_type;
  Eigen::Vector3d transformation;
  int srv_status;
};

typedef std::vector<TagDetectInfo> tag_detection_info_t;
/*
 * namespace
 */
namespace visual_servo_namespace {
class ServiceCaller {
  ros::NodeHandle nh;
  ros::ServiceClient client;
  visual_servo::manipulate srv{};
  std::tr1::shared_ptr<boost::thread> thread_ptr_;
  std::atomic_bool srvCalling_{}, srvFinished_{};

 public:
  ServiceCaller();
  ~ServiceCaller() = default;
  void worker(int SrvRequestType,
              Eigen::Vector3d transformation = Eigen::Vector3d(0, 0, 0));
  bool callSrv(int SrvRequestType,
               Eigen::Vector3d transformation = Eigen::Vector3d(0, 0, 0));
  const visual_servo::manipulate::Response& getSrvResponseStatus() const {
    return srv.response;
  };
  bool srvCalling() const { return srvCalling_; };
  bool srvFinished() {
    bool temp{srvFinished_};
    srvFinished_ = false;
    return temp;
  };
};

typedef enum {
  ROBOT_STATUS_STARTUP_SUCCEED,  //机械臂启动
  ROBOT_STATUS_STARTUP_FAIL,
  ROBOT_STATUS_MOVE_STOP,  //机械臂停止运动
  ROBOT_STATUS_SWITCH_ON,  //微动触发
  ROBOT_STATUS_ALL_RIGHT,  //机械臂一切正常
  ROBOT_STATUS_LOGIN_FAIL,
  ROBOT_STATUS_LOGIN_SUCCEED,

} RobotStatus;

typedef enum {
  SERVICE_STATUS_SUCCEED,
  SERVICE_STATUS_CUTIN_SUCCEED,
  SERVICE_STATUS_NO_TAG,             //未发现标签
  SERVICE_STATUS_OUT_RANGE,          //超出工作空间
  SERVICE_STATUS_GOAL_IN_COLLISION,  //规划场景中，规划路径上有障碍物
  SERVICE_STATUS_START_STATE_IN_COLLISION,
  SERVICE_STATUS_PLANNING_FAILED,
  SERVICE_STATUS_NO_IK_SOLUTION,
  SERVICE_STATUS_NO_TRAJECTORY,  //无法生成切割轨迹
  SERVICE_STATUS_UP_FAILED,
  SERVICE_STATUS_HOME_FAILED,
  SERVICE_STATUS_FORWARD_FAILED,
  SERVICE_STATUS_BACK_FAILED,
  SERVICE_STATUS_ROBOT_ABORT,
  SERVICE_STATUS_CLOSE_FAILED,
  SERVICE_STATUS_EMPTY,         //无法调用服务
  SERVICE_STATUS_SERVO_FAILED,  //伺服失败
  SERVICE_STATUS_CUT_FAILED,
  SERVICE_STATUS_CHARGE_FAILED,  // CHARGE FILED
  SERVICE_STATUS_LEAVE_CHARGE_FAILED,
  SERVICE_STATUS_LINEAR_FAILED,
} ServiceStatus;

static std::string getServiceStatusString(const int& ServiceStatus) {
  switch (ServiceStatus) {
    case visual_servo_namespace::SERVICE_STATUS_SUCCEED:
      return "Service call succeed and manipulate succeed";
    case visual_servo_namespace::SERVICE_STATUS_CUTIN_SUCCEED:
      return "Service call succeed and cut in succeed";
    case visual_servo_namespace::SERVICE_STATUS_NO_TAG:
      return "Service call failed because no tag searched";
    case visual_servo_namespace::SERVICE_STATUS_CLOSE_FAILED:
      return "Service call failed because can't get close";
    case visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED:
      return "Service call failed because can't servo to right place";
    case visual_servo_namespace::SERVICE_STATUS_CUT_FAILED:
      return "Service call failed because can't cut ";
    case visual_servo_namespace::SERVICE_STATUS_ROBOT_ABORT:
      return "Service call failed because robot abort ";
    case visual_servo_namespace::SERVICE_STATUS_UP_FAILED:
      return "Service call failed because can't get up";
    case visual_servo_namespace::SERVICE_STATUS_HOME_FAILED:
      return "Service call failed because can't get home";
    case visual_servo_namespace::SERVICE_STATUS_CHARGE_FAILED:
      return "Service call failed because cant't charge";
    case visual_servo_namespace::SERVICE_STATUS_LEAVE_CHARGE_FAILED:
      return "Service call failed because cant't leave charge";
    case visual_servo_namespace::SERVICE_STATUS_LINEAR_FAILED:
      return "Service call failed because can't go linear";
    case visual_servo_namespace::SERVICE_STATUS_FORWARD_FAILED:
      return "Service call failed because can't go forward";
    case visual_servo_namespace::SERVICE_STATUS_BACK_FAILED:
      return "Service call failed because can't go back";
    default:
      return "Service call succeed but no response";
  }
}
}  // namespace visual_servo_namespace
visual_servo_namespace::ServiceCaller::ServiceCaller() {
  client = nh.serviceClient<visual_servo::manipulate>("manipulate");
}
void visual_servo_namespace::ServiceCaller::worker(
    int SrvRequestType, Eigen::Vector3d transformation) {
  srvCalling_ = true;
  srv.request.type = SrvRequestType;
  srv.request.transformation = {transformation[0], transformation[1],
                                transformation[2]};
  if (client.call(srv)) {
    std::cout << "The error code is " << srv.response.status << ": "
              << srv.response.status_string << std::endl;
    srvFinished_ = true;
  } else {
    ROS_ERROR("Failed to call service detect_once");
    srv.response.status = visual_servo_namespace::SERVICE_STATUS_EMPTY;
    srvFinished_ = true;
  }
  srvCalling_ = false;
}
bool visual_servo_namespace::ServiceCaller::callSrv(
    int SrvRequestType, Eigen::Vector3d transformation) {
  thread_ptr_.reset(new boost::thread(boost::bind(
      &ServiceCaller::worker, this, SrvRequestType, transformation)));
  thread_ptr_->detach();
}

#endif  // VISUAL_SERVO_VISUALSERVOMETATYPE_H