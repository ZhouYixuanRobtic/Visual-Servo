#include "example_toolio.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "AuboRobotMetaType.h"    //机械臂的元数据类型
#include "serviceinterface.h"     //机械臂接口

#define SERVER_HOST "127.0.0.1"
//#define SERVER_HOST "192.168.1.123"
#define SERVER_PORT 8899



Example_ToolIO::Example_ToolIO()
{
}

void Example_ToolIO::demo()
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
        std::cerr<<"登录失败."<<std::endl;
    }

    aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
    memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));
    aubo_robot_namespace::ROBOT_SERVICE_STATE result;
    robotService.rootServiceRobotStartup(toolDynamicsParam, 6, true, true, 1000, result, true);    //机械臂初始化


    /** 设置工具端IO的电源和数字量IO的类型 **/
    ret = robotService.robotServiceSetToolPowerTypeAndDigitalIOType(aubo_robot_namespace::OUT_12V,
                                                              aubo_robot_namespace::IO_OUT, aubo_robot_namespace::IO_IN,
                                                              aubo_robot_namespace::IO_IN, aubo_robot_namespace::IO_OUT);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"设置工具端IO类型SUCC."<<std::endl;
    }
    else
    {
        std::cerr<<"设置工具端IO类型Failed."<<std::endl;
    }

    sleep(2);   //延时
    std::vector<aubo_robot_namespace::RobotIoDesc> statusVector;
    ret = robotService.robotServiceGetAllToolDigitalIOStatus(statusVector);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"Digital IO Count:"<<statusVector.size()<<std::endl;

        for(int i=0;i<(int)statusVector.size();i++)
        {
            std::cout<<"ioAddr:"<<statusVector[i].ioAddr<<"  ioType:"<<statusVector[i].ioType<<"  ioName:"<<statusVector[i].ioName<<std::endl;
        }
    }
    else
    {
        std::cerr<<"设置工具端IO类型Failed."<<std::endl;
    }
}
