#include "robottooliotestdemo.h"
#include <stdlib.h>
#include <unistd.h>

#define SERVER_HOST "127.0.0.1"
//#define SERVER_HOST "192.168.1.5"
#define SERVER_PORT 8899




RobotToolIoTestDemo::RobotToolIoTestDemo()
{
    m_robotService = new ServiceInterface();
}


int RobotToolIoTestDemo::loginAPI()
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 登录 ***/
    ret = m_robotService->robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");

    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"ERROR:登录失败."<<std::endl<<std::endl;
    }

    return ret;
}


int RobotToolIoTestDemo::logoutAPI()
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 退出登录 **/
    ret = m_robotService->robotServiceLogout();

    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"ERROR:退出登录失败."<<std::endl<<std::endl;
    }

    return ret;
}


int RobotToolIoTestDemo::setToolPowerVoltageTypeAPI(ToolPowerType type)
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    ret = m_robotService->robotServiceSetToolPowerVoltageType(type);

    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:设置工具端电压类型成功. 当前设置的类型:"<<type<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:设置工具端电压类型失败."<<std::endl;
    }

    return ret;
}


int RobotToolIoTestDemo::getToolPowerVoltageTypeAPI()
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    ToolPowerType type;

    /** 获取工具端电源类型 **/
    ret = m_robotService->robotServiceGetToolPowerVoltageType(type);

    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:获取电源电压类型成功.   结果为:"<<type<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:设置工具端电压类型失败."<<std::endl;
    }

    return ret;
}

int RobotToolIoTestDemo::getToolPowerVoltageStatusAPI()
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    double ToolPowerVoltageStatus;

    ret = m_robotService->robotServiceGetToolPowerVoltageStatus(ToolPowerVoltageStatus);

    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:获取工具端电源电压成功  电源电压:"<<ToolPowerVoltageStatus<<std::endl;
    }
    else
    {
        std::cout<<"ERROR:获取工具端电源电压失败."<<std::endl;
    }

    return ret;
}


int RobotToolIoTestDemo::getAllToolDigitalIOStatusAPI()
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    std::vector<aubo_robot_namespace::RobotIoDesc> statusVector;

    ret = m_robotService->robotServiceGetAllToolDigitalIOStatus(statusVector);

    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:工具端数字量IO的状态:"<<std::endl;

        for(int i=0;i<(int)statusVector.size();i++)
        {
            std::cout <<"名称:"<<statusVector[i].ioName <<" 类型:"<<statusVector[i].ioType  <<"  地址:"<<statusVector[i].ioAddr  <<"  数据:"<<statusVector[i].ioValue <<std::endl;
        }

        std::cout<<"INFO:获取工具端数字量IO的状态成功."<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:获取工具端数字量IO的状态失败."<<std::endl;
    }

    return ret;
}


int RobotToolIoTestDemo::setToolDigitalIOTypeAPI(ToolDigitalIOAddr addr, RobotIoType value)
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    if( (ret = m_robotService->robotServiceSetToolDigitalIOType(addr, value)) == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:设置工具端数字量IO的类型成功. 当前设置 addr:"<<addr<<"  数据:"<<value<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:设置工具端数字量IO的类型失败. addr:"<<addr<<std::endl;
    }

    return ret;
}

int RobotToolIoTestDemo::setToolDigitalIOStatusAPI(std::string name, IO_STATUS value)
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    if( ( ret = m_robotService->robotServiceSetToolDOStatus(name, value)) == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:设置工具端数字量IO的状态成功. 当前设置 name:"<<name<<"  数据:"<<value<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:设置工具端数字量IO的状态失败. name:"<<name<<std::endl;
    }

    return ret;
}

int RobotToolIoTestDemo::setToolDigitalIOStatusAPI(ToolDigitalIOAddr addr, IO_STATUS value)
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    if( ( ret = m_robotService->robotServiceSetToolDOStatus(addr, value)) == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"INFO:设置工具端数字量IO的状态成功.  当前设置 addr:"<<addr<<"  数据:"<<value<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:设置工具端数字量IO的状态失败. addr:"<<addr<<std::endl;
    }

    return ret;
}

int RobotToolIoTestDemo::robotServiceGetAllToolAIStatusAPI()
{
    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    std::vector<RobotIoDesc> toolAIStatusVector;
    ret = m_robotService->robotServiceGetAllToolAIStatus(toolAIStatusVector);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        for(int i=0;i<(int)toolAIStatusVector.size();i++)
        {
            std::cout <<"名称:"<<toolAIStatusVector[i].ioName <<" 类型:"<<toolAIStatusVector[i].ioType  <<"  地址:"<<toolAIStatusVector[i].ioAddr  <<"  数据:"<<toolAIStatusVector[i].ioValue <<std::endl;
        }

        std::cout<<"INFO:获取工具端所有AI的状态成功."<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:获取工具端所有AI的状态失败."<<std::endl;
    }
    return ret;
}

void RobotToolIoTestDemo::example()
{
    if(loginAPI() == false)
    {
        exit(1);
    }

    setToolPowerVoltageTypeAPI(OUT_0V);
    sleep(1);
    getToolPowerVoltageTypeAPI();

    setToolPowerVoltageTypeAPI(OUT_12V);
    sleep(1);
    getToolPowerVoltageTypeAPI();

    setToolPowerVoltageTypeAPI(OUT_24V);
    sleep(1);
    getToolPowerVoltageTypeAPI();
    std::cout<<"=================================================="<<std::endl<<std::endl;


    setToolDigitalIOTypeAPI(TOOL_DIGITAL_IO_0, RobotToolDI);
    setToolDigitalIOTypeAPI(TOOL_DIGITAL_IO_1, RobotToolDI);
    setToolDigitalIOTypeAPI(TOOL_DIGITAL_IO_2, RobotToolDI);
    setToolDigitalIOTypeAPI(TOOL_DIGITAL_IO_3, RobotToolDI);
    sleep(5);
    getAllToolDigitalIOStatusAPI();
    std::cout<<"=================================================="<<std::endl<<std::endl;


    setToolDigitalIOTypeAPI(TOOL_DIGITAL_IO_0, RobotToolDO);
    setToolDigitalIOTypeAPI(TOOL_DIGITAL_IO_1, RobotToolDO);
    setToolDigitalIOTypeAPI(TOOL_DIGITAL_IO_2, RobotToolDO);
    setToolDigitalIOTypeAPI(TOOL_DIGITAL_IO_3, RobotToolDO);
    sleep(5);
    getAllToolDigitalIOStatusAPI();
    std::cout<<"=================================================="<<std::endl<<std::endl;

    setToolDigitalIOStatusAPI("T_DI/O_00", IO_STATUS_INVALID);
    setToolDigitalIOStatusAPI("T_DI/O_01", IO_STATUS_INVALID);
    setToolDigitalIOStatusAPI("T_DI/O_02", IO_STATUS_INVALID);
    setToolDigitalIOStatusAPI("T_DI/O_03", IO_STATUS_INVALID);
    sleep(5);
    getAllToolDigitalIOStatusAPI();
    std::cout<<"=================================================="<<std::endl<<std::endl;

    setToolDigitalIOStatusAPI(TOOL_DIGITAL_IO_0, IO_STATUS_VALID);
    setToolDigitalIOStatusAPI(TOOL_DIGITAL_IO_1, IO_STATUS_VALID);
    setToolDigitalIOStatusAPI(TOOL_DIGITAL_IO_2, IO_STATUS_VALID);
    setToolDigitalIOStatusAPI(TOOL_DIGITAL_IO_3, IO_STATUS_VALID);
    sleep(5);
    getAllToolDigitalIOStatusAPI();
    std::cout<<"=================================================="<<std::endl<<std::endl;

    robotServiceGetAllToolAIStatusAPI();

    getToolPowerVoltageStatusAPI();
    std::cout<<"=================================================="<<std::endl<<std::endl;

    logoutAPI();
}



