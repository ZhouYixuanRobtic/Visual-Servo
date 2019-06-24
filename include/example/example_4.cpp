#include "example_4.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>

#include "util.h"


#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 8899

aubo_robot_namespace::wayPoint_S Example_4::s_currentWayPoing;

Example_4::Example_4()
{
}

void Example_4::RealTimeWaypointCallback(const aubo_robot_namespace::wayPoint_S *wayPointPtr, void *arg)
{
    (void)arg;
    s_currentWayPoing = *wayPointPtr;
    //Util::printWaypoint(s_currentWayPoing);
}


void Example_4::demo()
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


    robotService.robotServiceRegisterRealTimeRoadPointCallback(Example_4::RealTimeWaypointCallback, NULL);


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

   /** 运动到初始位姿 **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    Util::initJointAngleArray(jointAngle, 0.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
    ret = robotService.robotServiceJointMove(jointAngle, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动至零位姿态失败.　ret:"<<ret<<std::endl;
    }


    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置末端型运动的最大加速度 　　直线运动属于末端型运动***/
    double lineMoveMaxAcc;
    lineMoveMaxAcc = 4;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(lineMoveMaxAcc);
    robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(lineMoveMaxAcc);

    /** 接口调用: 设置末端型运动的最大速度 直线运动属于末端型运动***/
    double lineMoveMaxVelc;
    lineMoveMaxVelc = 4;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(lineMoveMaxVelc);
    robotService.robotServiceGetGlobalMoveEndMaxAngleVelc(lineMoveMaxVelc);

    for(int i=0;i<20;i++)
    {
        double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
        Util::initJointAngleArray(jointAngle, 0.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
        ret = robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动至路点0失败.　ret:"<<ret<<std::endl;
        }

        Util::initJointAngleArray(jointAngle, 0.0/180.0*M_PI,  0.0/180.0*M_PI,  45.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
        robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动至路点1失败.　ret:"<<ret<<std::endl;
        }

        Util::initJointAngleArray(jointAngle, 30.0/180.0*M_PI,  0.0/180.0*M_PI,  45.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
        robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动至路点2失败.　ret:"<<ret<<std::endl;
        }

        Util::initJointAngleArray(jointAngle, 30.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
        robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动至路点3失败.　ret:"<<ret<<std::endl;
        }
    }


    /** 机械臂Shutdown **/
    robotService.robotServiceRobotShutdown();

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}



void Example_4::demo_relativeOri()
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

    robotService.robotServiceRegisterRealTimeRoadPointCallback(Example_4::RealTimeWaypointCallback, NULL);

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

   /** 运动到初始位姿 **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    Util::initJointAngleArray(jointAngle, 0.0/180.0*M_PI,  0.0/180.0*M_PI,  90.0/180.0*M_PI, 0.0/180.0*M_PI, 90.0/180.0*M_PI, 0.0/180.0*M_PI);
    ret = robotService.robotServiceJointMove(jointAngle, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动至零位姿态失败.　ret:"<<ret<<std::endl;
    }


    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置末端型运动的最大加速度 　　直线运动属于末端型运动***/
    double lineMoveMaxAcc;
    lineMoveMaxAcc = 1.0;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(lineMoveMaxAcc);
    robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(lineMoveMaxAcc);

    /** 接口调用: 设置末端型运动的最大速度 直线运动属于末端型运动***/
    double lineMoveMaxVelc;
    lineMoveMaxVelc = 2.0;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(lineMoveMaxVelc);
    robotService.robotServiceGetGlobalMoveEndMaxAngleVelc(lineMoveMaxVelc);

    //定义一个工具
    aubo_robot_namespace::ToolInEndDesc toolInEndDesc;
    toolInEndDesc.toolInEndPosition.x=0;
    toolInEndDesc.toolInEndPosition.y=0;
    toolInEndDesc.toolInEndPosition.z=0.1;
    toolInEndDesc.toolInEndOrientation.w=1;
    toolInEndDesc.toolInEndOrientation.x=0;
    toolInEndDesc.toolInEndOrientation.y=0;
    toolInEndDesc.toolInEndOrientation.z=0;

    for(int i=0;i<50;i++)
    {
        aubo_robot_namespace::Ori relativeOri;
        aubo_robot_namespace::Rpy relativeRpy;

        relativeRpy.rx=1.0/180.0*M_PI;
        relativeRpy.ry=1.0/180.0*M_PI;
        relativeRpy.rz=0.0/180.0*M_PI;
        robotService.RPYToQuaternion(relativeRpy,relativeOri);  //欧拉角转四元素　　求得偏移量的四元素表示

        aubo_robot_namespace::MoveRelative relativeMoveOnUserCoord;
        aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;

        relativeMoveOnUserCoord.ena = true;
        relativeMoveOnUserCoord.relativePosition[0] = 0;
        relativeMoveOnUserCoord.relativePosition[1] = 0;
        relativeMoveOnUserCoord.relativePosition[2] = 0;
        relativeMoveOnUserCoord.relativeOri = relativeOri;

        userCoord.coordType=aubo_robot_namespace::EndCoordinate;
        userCoord.toolDesc=toolInEndDesc;

        robotService.robotServiceSetMoveRelativeParam(relativeMoveOnUserCoord, userCoord);   //设置偏移
        robotService.robotServiceSetToolKinematicsParam(toolInEndDesc);

        robotService.robotServiceLineMove(s_currentWayPoing.jointpos,true);
    }


    /** 机械臂Shutdown **/
    robotService.robotServiceRobotShutdown();

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}
