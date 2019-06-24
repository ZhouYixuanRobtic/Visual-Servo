#include "example_9.h"


#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>


#include <vector>

#include "AuboRobotMetaType.h"    //机械臂的元数据类型
#include "serviceinterface.h"     //机械臂接口


//#define SERVER_HOST "127.0.0.1"
#define SERVER_HOST "192.168.1.123"
#define SERVER_PORT 8899


Example_9::Example_9()
{
}

void Example_9::demo()
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

//    //获取　数字量IO状态
//    double value;
//    robotService.robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDI, "U_DI_01", value);
//    std::cerr<<"U_DI_01状态:"<<value<<std::endl;

//    std::vector<aubo_robot_namespace::RobotIoType> ioType;
//    std::vector<aubo_robot_namespace::RobotIoDesc> statusVector;

//    ioType.push_back(aubo_robot_namespace::RobotBoardUserAI);
//    robotService.robotServiceGetBoardIOStatus(ioType, statusVector);

//    for(int i=0;i<statusVector.size();i++)
//    std::cerr<<statusVector[i].ioName<<"   "<<statusVector[i].ioValue<<std::endl;


//    //获取　模拟量IO状态
//    robotService.robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotBoardUserAI, "VI0", value);
//    std::cerr<<"---------VI0状态:"<<value<<std::endl;

    //设置IO状态
    ret = robotService.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO, "U_DO_02", 0);
    std::cerr<<"ret:"<<ret<<std::endl;

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}



