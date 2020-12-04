//
// Created by xcy on 19-6-26.
//
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include "AuboSDK.h"

#define SERVER_HOST "192.168.2.13"
#define SERVER_PORT 8899

AuboSDK::AuboSDK()
{
    robotService = new ServiceInterface;
    loginSucceed= false;
    ret = robotService->robotServiceLogin(SERVER_HOST,SERVER_PORT,"aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"登录成功."<<std::endl;
        loginSucceed=true;
    }
    else
    {
        loginSucceed=false;
        std::cerr<<"登录失败."<<std::endl;
    }
}
AuboSDK::~AuboSDK()
{
    delete robotService;
}
bool AuboSDK::robotStartUp()
{

    //工具动力学参数
    robotEventRegister();

    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    ret = robotService->rootServiceRobotStartup(toolDynamicsParam/**工具动力学参数**/,
                                                6        /*碰撞等级*/,
                                                true     /*是否允许读取位姿　默认为true*/,
                                                true,    /*保留默认为true */
                                                1000,    /*保留默认为1000 */
                                                result); /*机械臂初始化*/
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        return true;
        std::cerr<<"机械臂初始化成功."<<std::endl;
    }
    else
    {
        std::cerr<<"机械臂初始化失败."<<std::endl;
        return false;
    }
}
int AuboSDK::robotShutDown()
{
    /** 机械臂Shutdown **/
    robotService->robotServiceRobotShutdown();
    /** 机械臂电源控制 **/
    robotService->robotServicePowerControl(false);
    /** 接口调用: 退出登录　**/
    robotService->robotServiceLogout();

    return aubo_robot_namespace::InterfaceCallSuccCode;
}
void AuboSDK::RealTimeEventInfoCallback(const aubo_robot_namespace::RobotEventInfo *pEventInfo, void *arg)
{
    (void)arg;
    Util::printEventInfo(*pEventInfo);
    if(pEventInfo->eventType==aubo_robot_namespace::RobotEvent_exitSoftEmergency)
    {
        std::cout<<"退出急停，重启机械臂"<<std::endl;
        ExitSoftEmergency=true;
    }
    else if(pEventInfo->eventType==aubo_robot_namespace::RobotEvent_robotStartupDoneResult)
    {
        RobotMoveStop=false;
    }
    else if(pEventInfo->eventType==aubo_robot_namespace::RobotEventMoveEnterStopState
            ||pEventInfo->eventType==aubo_robot_namespace::RobotEvent_robotShutdownDone)
    {
        RobotMoveStop=true;
    }
    else if(pEventInfo->eventType==aubo_robot_namespace::RobotEvent_collision)
    {
        RobotCollisionRecover=false;
        RobotMoveStop=true;
    }
    else if(pEventInfo->eventType==aubo_robot_namespace::RobotEvent_collisionStatusChanged)
    {
        RobotCollisionRecover=true;
    }

}
bool AuboSDK::robotEventRegister()
{
    ret = robotService->robotServiceRegisterRobotEventInfoCallback(AuboSDK::RealTimeEventInfoCallback, NULL);
    return ret == aubo_robot_namespace::InterfaceCallSuccCode;
}
bool AuboSDK::OverturnIOStatus()
{
    double value;
    ret=robotService->robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO,"U_DO_00",value);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
            ret = value==1.0 ? robotService->robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO, "U_DO_00", 0.0)
                             : robotService->robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO, "U_DO_00", 1.0);
    }
    else
        return false;
    return ret == aubo_robot_namespace::InterfaceCallSuccCode;

}
bool AuboSDK::toolStart()
{
    ret=robotService->robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO,"U_DO_00",1.0);
    return ret == aubo_robot_namespace::InterfaceCallSuccCode;
}
bool AuboSDK::toolAllclear()
{
    ret = robotService->robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO,"U_DO_00",0.0);
    return ret == aubo_robot_namespace::InterfaceCallSuccCode;
}
int AuboSDK::robotDiagno()
{
    aubo_robot_namespace::RobotDiagnosis robotDiagnosisInfo;
    robotService->robotServiceGetRobotDiagnosisInfo(robotDiagnosisInfo);
    Util::printRobotDiagnosis(robotDiagnosisInfo);
    return 1;
}
bool AuboSDK::robotFastMoveStop()
{
    ret=robotService->rootServiceRobotControl(aubo_robot_namespace::RobotControlCommand(aubo_robot_namespace::RobotBrake));
    return ret == aubo_robot_namespace::InterfaceCallSuccCode;
}
bool AuboSDK::robotFastMoveRelease()
{
    ret=robotService->rootServiceRobotControl(aubo_robot_namespace::RobotControlCommand(aubo_robot_namespace::RobotRelease));
    return ret == aubo_robot_namespace::InterfaceCallSuccCode;
}

