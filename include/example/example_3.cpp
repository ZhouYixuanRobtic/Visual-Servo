#include "example_3.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include "util.h"


#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 8899

Example_3::Example_3()
{
}

void Example_3::demo()
{
    ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"登录成功."<<std::endl;
    }
    else
    {
        std::cerr<<"登录成功."<<std::endl;
    }

     std::cerr<<"机械臂初始化....."<<std::endl;


    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;

    //工具动力学参数
    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

    ret = robotService.rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                               6        /*碰撞等级*/,
                                               true     /*是否允许读取位姿　默认为true*/,
                                               true,    /*保留默认为true */
                                               1000,    /*保留默认为1000 */
                                               result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"机械臂初始化成功."<<std::endl;
    }
    else
    {
        std::cerr<<"机械臂初始化失败."<<std::endl;
    }

    /** 业务块 **/
    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置关节型运动的最大加速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
    jointMaxAcc.jointPara[0] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[1] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[2] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[3] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[4] = 50.0/180.0*M_PI;
    jointMaxAcc.jointPara[5] = 50.0/180.0*M_PI;   //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);

    /** 接口调用: 设置关节型运动的最大速度 ***/
    aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
    jointMaxVelc.jointPara[0] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[1] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[2] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[3] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[4] = 50.0/180.0*M_PI;
    jointMaxVelc.jointPara[5] = 50.0/180.0*M_PI;   //接口要求单位是弧度
    robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

    /** 机械臂运动到零位姿态 **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    Util::initJointAngleArray(jointAngle, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    ret = robotService.robotServiceJointMove(jointAngle, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动至零位姿态失败.　ret:"<<ret<<std::endl;
    }

    for(int i=0;i<80;i++)
    {
        /** 接口调用: 设置偏移量**/
        aubo_robot_namespace::MoveRelative relativeMoveOnBase;
        relativeMoveOnBase.ena = true;
        relativeMoveOnBase.relativePosition[0] = 0;
        relativeMoveOnBase.relativePosition[1] = 0;
        relativeMoveOnBase.relativePosition[2] = 0.05*(i%4);   //单位米

        relativeMoveOnBase.relativeOri.w=1;
        relativeMoveOnBase.relativeOri.x=0;
        relativeMoveOnBase.relativeOri.y=0;
        relativeMoveOnBase.relativeOri.z=0;
        robotService.robotServiceSetMoveRelativeParam(relativeMoveOnBase);


        /** 路点1运动 **/
        Util::initJointAngleArray(jointAngle, 0.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
        robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动至路点1失败.　ret:"<<ret<<std::endl;
            break;
        }

        /** 路点2运动 **/
        Util::initJointAngleArray(jointAngle, 15.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
        robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动至路点2失败.　ret:"<<ret<<std::endl;
            break;
        }
    }

    /** 机械臂Shutdown **/
    robotService.robotServiceRobotShutdown();

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}
