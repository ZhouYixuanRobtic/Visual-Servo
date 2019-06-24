#include "conveyortrackexample.h"

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"


#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 8899

ConveyorTrackExample::ConveyorTrackExample()
{

}

void ConveyorTrackExample::demo()
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


//    ret = robotService.robotServiceSetConveyorEncoderReset();
//    ret = robotService.robotServiceSetConveyorStartup();
//    ret = robotService.robotServiceSetConveyorStop();
//    ret=robotService.robotServiceSetConveyorDir(5);

//    ret=robotService.robotServiceSetConveyorVelc(1.23456);
//    ret=robotService.robotServiceSetEncoderValPerMeter(12345);
//    ret=robotService.robotServiceSetStartWindowUpstream(3.21);
//    ret=robotService.robotServiceSetStartWindowDownstream(8.765);
//    ret=robotService.robotServiceSetConveyorTrackDownstream(2367.6543);

//    aubo_robot_namespace::RobotCameraCalib robotCameraCalib;

//    robotCameraCalib.toolInEndOrientation.w=1.1;
//    robotCameraCalib.toolInEndOrientation.x=2.1;
//    robotCameraCalib.toolInEndOrientation.y=3.1;
//    robotCameraCalib.toolInEndOrientation.z=4.1;

//    robotCameraCalib.toolInEndPosition.x=1.23;
//    robotCameraCalib.toolInEndPosition.y=2.34;
//    robotCameraCalib.toolInEndPosition.z=3.45;
//    ret=robotService.robotServiceSetRobotCameraCalib(robotCameraCalib);

    aubo_robot_namespace::Pos pos;
    aubo_robot_namespace::Ori ori;

    pos.x = 1.23;
    pos.y = 2.23;
    pos.z = 3.23;

    ori.w = 4.13;
    ori.x = 5.13;
    ori.y = 6.13;
    ori.z = 7.13;

    int timeStamp=12345678;

    ret=robotService.robotServiceAppendObject2ConveyorTrackQueue(pos, ori, timeStamp);


    ret=robotService.robotServiceEnableConveyorTrack();

    int value;
    robotService.robotServiceGetConveyorEncoderVal(value);


    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"调用接口成功."<<std::endl;
    }
    else
    {
        std::cerr<<"调用接口失败."<<std::endl;
    }

}
