#include "example_5.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include "util.h"

#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 8899

Example_5::Example_5()
{
}

void Example_5::demo()
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

    /** 模拟业务 **/
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


    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置末端型运动的最大加速度 　　直线运动属于末端型运动***/
    double endMoveMaxAcc;
    endMoveMaxAcc = 0.2;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(endMoveMaxAcc);
    robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(endMoveMaxAcc);


    /** 接口调用: 设置末端型运动的最大速度 直线运动属于末端型运动***/
    double endMoveMaxVelc;
    endMoveMaxVelc = 0.2;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(endMoveMaxVelc);
    robotService.robotServiceSetGlobalMoveEndMaxAngleVelc(endMoveMaxVelc);

    double jointAngle[aubo_robot_namespace::ARM_DOF];

    for(int i=0;i<1;i++)
    {
//        //准备点  关节运动属于关节型运动
//        robotService.robotServiceInitGlobalMoveProfile();

//        robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
//        robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);
//        Util::initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
//        ret = robotService.robotServiceJointMove(jointAngle, true);   //关节运动至准备点
//        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
//        {
//            std::cerr<<"JointMove失败.　ret:"<<ret<<std::endl;
//        }


//        //圆弧
//        robotService.robotServiceInitGlobalMoveProfile();

//        robotService.robotServiceSetGlobalMoveEndMaxLineAcc(endMoveMaxAcc);
//        robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(endMoveMaxAcc);
//        robotService.robotServiceSetGlobalMoveEndMaxLineVelc(endMoveMaxVelc);
//        robotService.robotServiceSetGlobalMoveEndMaxAngleVelc(endMoveMaxVelc);
//        Util::initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
//        robotService.robotServiceAddGlobalWayPoint(jointAngle);

//        Util::initJointAngleArray(jointAngle,0.200000, -0.127267, -1.321122, 0.376934, -1.570794, -0.000008);
//        robotService.robotServiceAddGlobalWayPoint(jointAngle);

//        Util::initJointAngleArray(jointAngle,0.600000, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
//        robotService.robotServiceAddGlobalWayPoint(jointAngle);

//        robotService.robotServiceSetGlobalCircularLoopTimes(0);    //圆的圈数
//        ret = robotService.robotServiceTrackMove(aubo_robot_namespace::ARC_CIR, true);

//        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
//        {
//            std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
//        }


//        //准备点
//        robotService.robotServiceInitGlobalMoveProfile();

//        robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
//        robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);
//        Util::initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
//        ret = robotService.robotServiceJointMove(jointAngle, true);   //关节运动至准备点
//        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
//        {
//            std::cerr<<"JointMove失败.　ret:"<<ret<<std::endl;
//        }

//        //圆
//        robotService.robotServiceInitGlobalMoveProfile();

//        robotService.robotServiceSetGlobalMoveEndMaxLineAcc(endMoveMaxAcc);
//        robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(endMoveMaxAcc);
//        robotService.robotServiceSetGlobalMoveEndMaxLineVelc(endMoveMaxVelc);
//        robotService.robotServiceSetGlobalMoveEndMaxAngleVelc(endMoveMaxVelc);
//        Util::initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
//        robotService.robotServiceAddGlobalWayPoint(jointAngle);
//        Util::initJointAngleArray(jointAngle,-0.211675, -0.325189, -1.466753, 0.429232, -1.570794, -0.211680);
//        robotService.robotServiceAddGlobalWayPoint(jointAngle);
//        Util::initJointAngleArray(jointAngle,-0.037186, -0.224307, -1.398285, 0.396819, -1.570796, -0.037191);
//        robotService.robotServiceAddGlobalWayPoint(jointAngle);

//        robotService.robotServiceSetGlobalCircularLoopTimes(1);    //圆的圈数
//        ret = robotService.robotServiceTrackMove(aubo_robot_namespace::ARC_CIR,true);
//        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
//        {
//            std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
//        }


        //准备点
//        robotService.robotServiceInitGlobalMoveProfile();

//        robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
//        robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);
//        Util::initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
//        ret = robotService.robotServiceJointMove(jointAngle, true);   //关节运动至准备点
//        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
//        {
//            std::cerr<<"JointMove失败.　ret:"<<ret<<std::endl;
//        }

        //MoveP
        robotService.robotServiceInitGlobalMoveProfile();

        robotService.robotServiceSetGlobalMoveEndMaxLineAcc(endMoveMaxAcc);
        robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(endMoveMaxAcc);
        robotService.robotServiceSetGlobalMoveEndMaxLineVelc(endMoveMaxVelc);
        robotService.robotServiceSetGlobalMoveEndMaxAngleVelc(endMoveMaxVelc);
        Util::initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
        robotService.robotServiceAddGlobalWayPoint(jointAngle);
        Util::initJointAngleArray(jointAngle,0.100000, -0.147267, -1.321122, 0.376934, -1.570794, -0.000008);
        robotService.robotServiceAddGlobalWayPoint(jointAngle);
        Util::initJointAngleArray(jointAngle,0.200000, -0.167267, -1.321122, 0.376934, -1.570796, -0.000008);
        robotService.robotServiceAddGlobalWayPoint(jointAngle);

        robotService.robotServiceSetGlobalBlendRadius(0.03);                     //交融半径
        ret = robotService.robotServiceTrackMove(aubo_robot_namespace::CARTESIAN_GNUBSPLINEINTP,true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
        }
    }

    /** 机械臂Shutdown **/
    robotService.robotServiceRobotShutdown();

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}
