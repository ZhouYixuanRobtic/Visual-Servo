//
// Created by xcy on 19-6-27.
//

#ifndef VISUAL_SERVO_VISUALSERVOMETATYPE_H
#define VISUAL_SERVO_VISUALSERVOMETATYPE_H

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>
#include  <math.h>
#include <ctime>

#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <Eigen/Core>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/ximgproc.hpp>
#include "cv.h"
#include "highgui.h"

#include "visual_servo/TagDetection_msg.h"
#include "visual_servo/TagsDetection_msg.h"
#include "visual_servo/detect_once.h"
#include "visual_servo/manipulate.h"
#include "visual_servo/VisualServoMetaTypeMsg.h"

/**
 * General types
 */
typedef  uint8_t     boolean;
typedef  int8_t      int8;
typedef  int16_t     int16;
typedef  int32_t     int32;
typedef  uint8_t     uint8;
typedef  uint16_t    uint16;
typedef  uint32_t    uint32;
typedef  int64_t     int64;
typedef  uint64_t    uint64;
typedef  float       float32;
typedef  double      float64;

struct TagDetectInfo{
    //the tag pose with respect to camera described as a homogeneous matrix
    Eigen::Affine3d Trans_C2T;
    //the tag id
    int id;
    //the coefficient unit meters per pixel
    double PixelCoef;
    //the pixel point of tag center
    cv::Point2d Center;
};

struct Destination_t{
    //the end effector motion described as a homogeneous matrix
    Eigen::Affine3d EE_Motion;
    //the position error with respect to expected position
    double error;
};

struct manipulateSrv{
    int srv_type;
    int srv_status;
};

typedef std::vector<TagDetectInfo> tag_detection_info_t;
/*
 * namespace
 */
namespace visual_servo_namespace
{
    typedef enum{
        ROBOT_STATUS_STARTUP_SUCCEED,           //机械臂启动
        ROBOT_STATUS_STARTUP_FAIL,
        ROBOT_STATUS_MOVE_STOP,         //机械臂停止运动
        ROBOT_STATUS_SWITCH_ON,         //微动触发
        ROBOT_STATUS_ALL_RIGHT,         //机械臂一切正常
        ROBOT_STATUS_LOGIN_FAIL,
        ROBOT_STATUS_LOGIN_SUCCEED,

    }RobotStatus;


    typedef enum{
        SERVICE_STATUS_SUCCEED,
        SERVICE_STATUS_NO_TAG,          //未发现标签
        SERVICE_STATUS_OUT_RANGE,       //超出工作空间
        SERVICE_STATUS_GOAL_IN_COLLISION,    //规划场景中，规划路径上有障碍物
        SERVICE_STATUS_START_STATE_IN_COLLISION,
        SERVICE_STATUS_PLANNING_FAILED,
        SERVICE_STATUS_NO_IK_SOLUTION,
        SERVICE_STATUS_NO_TRAJECTORY,   //无法生成切割轨迹
        SERVICE_STATUS_UP_FAILED,
        SERVICE_STATUS_HOME_FAILED,
        SERVICE_STATUS_ROBOT_ABORT,
        SERVICE_STATUS_CLOSE_FAILED,
        SERVICE_STATUS_EMPTY,         //无法调用服务
        SERVICE_STATUS_SERVO_FAILED,      //伺服失败
        SERVICE_STATUS_CUT_FAILED,
        SERVICE_STATUS_CHARGE_FAILED,    //CHARGE FILED
        SERVICE_STATUS_LEAVE_CHARGE_FAILED,
    }ServiceStatus;

    static void printServiceStatus(const int & ServiceStatus)
    {
        switch(ServiceStatus)
        {
            case visual_servo_namespace::SERVICE_STATUS_SUCCEED:
                std::cout<<"Service call succeed and manipulate succeed"<<std::endl;
                break;
            case visual_servo_namespace::SERVICE_STATUS_NO_TAG:
                std::cout<<"Service call failed because no tag searched"<<std::endl;
                break;
            case visual_servo_namespace::SERVICE_STATUS_CLOSE_FAILED:
                std::cout<<"Service call failed because can't get close"<<std::endl;
                break;
            case visual_servo_namespace::SERVICE_STATUS_SERVO_FAILED:
                std::cout<<"Service call failed because can't servo to right place"<<std::endl;
                break;
            case visual_servo_namespace::SERVICE_STATUS_CUT_FAILED:
                std::cout<<"Service call failed because can't cut "<<std::endl;
                break;
            case visual_servo_namespace::SERVICE_STATUS_ROBOT_ABORT:
                std::cout<<"Service call failed because robot abort "<<std::endl;
                break;
            case visual_servo_namespace::SERVICE_STATUS_UP_FAILED:
                std::cout<<"Service call failed because can't get up"<<std::endl;
                break;
            case visual_servo_namespace::SERVICE_STATUS_HOME_FAILED:
                std::cout<<"Service call failed because can't get home"<<std::endl;
                break;
            case visual_servo_namespace::SERVICE_STATUS_CHARGE_FAILED:
                std::cout<<"Service call failed because cant't charge"<<std::endl;
                break;
            case visual_servo_namespace::SERVICE_STATUS_LEAVE_CHARGE_FAILED:
                std::cout<<"Service call failed because cant't leave charge"<<std::endl;
                break;
            default:
                std::cout<<"Service call succeed but no response"<<std::endl;
                break;
        }
    }
}




#endif //VISUAL_SERVO_VISUALSERVOMETATYPE_H