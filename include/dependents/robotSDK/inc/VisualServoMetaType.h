//
// Created by xcy on 19-6-27.
//

#ifndef VISUAL_SERVO_VISUALSERVOMETATYPE_H
#define VISUAL_SERVO_VISUALSERVOMETATYPE_H

#include <iostream>
#include <stdint.h>

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
            default:
                std::cout<<"Service call succeed but no response"<<std::endl;
                break;
        }
    }
}




#endif //VISUAL_SERVO_VISUALSERVOMETATYPE_H
