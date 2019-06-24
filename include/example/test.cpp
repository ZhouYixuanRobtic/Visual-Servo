#include "test.h"

#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 8899

Test::Test()
{
}

void Test::demo()
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


//    ret = robotService.robotServiceLeaveTcp2CanbusMode();
//    std::cerr<<"LeaveTcp2CanbusMode返回结果."<<ret<<std::endl;


//    ret = robotService.robotServiceEnterTcp2CanbusMode();

//    std::cerr<<"EnterTcp2CanbusMode返回结果."<<ret<<std::endl;

    aubo_robot_namespace::JointStatus jointStatus[6];

    ret = robotService.robotServiceGetRobotJointStatus(jointStatus, 6);

    for(int i=0;i<6;i++)
    {
        std::cout<<"joint"<<i<<":"<<jointStatus[i].jointPosJ<<"    ";
    }

    std::cout<<std::endl;


    std::cout<<"=="<<sizeof(aubo_robot_namespace::JointStatus)<<std::endl;


    if(ret == aubo_robot_namespace::ErrnoSucc)
    {
        std::cerr<<"获取关节状态成功."<<std::endl;
    }
    else
    {
        std::cerr<<"获取关节状态失败."<<std::endl;
    }

}
