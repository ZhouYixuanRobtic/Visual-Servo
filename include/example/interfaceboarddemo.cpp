#include "interfaceboarddemo.h"

#include <vector>
#include <unistd.h>
#include <stdlib.h>
#include "serviceinterface.h"

//#define SERVER_HOST "127.0.0.1"
#define SERVER_HOST "192.168.1.5"
#define SERVER_PORT 8899

InterfaceBoardDemo::InterfaceBoardDemo()
{

}



void InterfaceBoardDemo::example()
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

        exit(1);
    }


    while(true)
    {
        /** ******************** 获取IO配置　***************** **/
        {
            int ret = aubo_robot_namespace::InterfaceCallSuccCode;
            std::vector<aubo_robot_namespace::RobotIoType> ioType;
            std::vector<aubo_robot_namespace::RobotIoDesc> configVector;
            ioType.push_back(aubo_robot_namespace::RobotBoardControllerDI);
            ioType.push_back(aubo_robot_namespace::RobotBoardControllerDO);
            ioType.push_back(aubo_robot_namespace::RobotBoardControllerAI);
            ioType.push_back(aubo_robot_namespace::RobotBoardControllerAO);
            ioType.push_back(aubo_robot_namespace::RobotBoardUserDI);
            ioType.push_back(aubo_robot_namespace::RobotBoardUserDO);
            ioType.push_back(aubo_robot_namespace::RobotBoardUserAI);
            ioType.push_back(aubo_robot_namespace::RobotBoardUserAO);

            ret = robotService.robotServiceGetBoardIOConfig(ioType, configVector);
            if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
            {
                std::cout<<"获取BoardIOConfig成功"<<std::endl;
            }
            else
            {
                std::cerr<<"ERROR:getBoardIOConfigAPI 失败."<<std::endl;
            }
        }

        std::cout<<std::endl;

        /** ********************* 获取IO状态　***************** **/
        {
            int ret = aubo_robot_namespace::InterfaceCallSuccCode;
            std::vector<aubo_robot_namespace::RobotIoType> ioType;
            std::vector<aubo_robot_namespace::RobotIoDesc> statusVector;
            ioType.push_back(aubo_robot_namespace::RobotBoardControllerDI);
            ioType.push_back(aubo_robot_namespace::RobotBoardControllerDO);
            ioType.push_back(aubo_robot_namespace::RobotBoardControllerAI);
            ioType.push_back(aubo_robot_namespace::RobotBoardControllerAO);
            ioType.push_back(aubo_robot_namespace::RobotBoardUserDI);
            ioType.push_back(aubo_robot_namespace::RobotBoardUserDO);
            ioType.push_back(aubo_robot_namespace::RobotBoardUserAI);
            ioType.push_back(aubo_robot_namespace::RobotBoardUserAO);

            ret = robotService.robotServiceGetBoardIOStatus(ioType, statusVector);
            if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
            {
                std::cout<<"获取BoardIOStatus成功"<<std::endl;
            }
            else
            {
                std::cerr<<"ERROR:getBoardIOStatusAPI 失败."<<std::endl;
            }
        }

        std::cout<<"================================"<<std::endl<<std::endl;

        sleep(5);
    }

    /** 接口调用: 退出登录　**/
    ret = robotService.robotServiceLogout();
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"退出登录失败."<<std::endl;

        exit(1);
    }

}




