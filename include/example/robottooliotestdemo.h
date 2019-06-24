#ifndef ROBOTTOOLIOTESTDEMO_H
#define ROBOTTOOLIOTESTDEMO_H

#include "serviceinterface.h"
using namespace aubo_robot_namespace;

class RobotToolIoTestDemo
{
public:
    RobotToolIoTestDemo();

private:
    /** 登录 **/
    int loginAPI();

    /** 退出登录 **/
    int logoutAPI();

    /** 设置工具端电源电压类型 接口 **/
    int setToolPowerVoltageTypeAPI(ToolPowerType type);

    /** 获取工具端电源电压类型 接口  **/
    int getToolPowerVoltageTypeAPI();

    /** 获取工具端电源电压状态 接口  **/
    int getToolPowerVoltageStatusAPI();

    /** 获取工具端数字量IO的状态 接口  **/
    int getAllToolDigitalIOStatusAPI();

    /** 设置工具端数字量IO的状态 接口  **/
    int setToolDigitalIOTypeAPI(aubo_robot_namespace::ToolDigitalIOAddr addr, RobotIoType value);

    /** 设置工具端数字量IO的状态 接口   **/
    int setToolDigitalIOStatusAPI(std::string name, aubo_robot_namespace::IO_STATUS value);

    /** 设置工具端数字量IO的状态 接口   **/
    int setToolDigitalIOStatusAPI(aubo_robot_namespace::ToolDigitalIOAddr addr, aubo_robot_namespace::IO_STATUS value);

    /** 获取工具段所有AI的状态 接口   **/
    int robotServiceGetAllToolAIStatusAPI();


public:
    void example();

    
private:
    ServiceInterface *m_robotService;

};

#endif // ROBOTTOOLIOTESTDEMO_H
