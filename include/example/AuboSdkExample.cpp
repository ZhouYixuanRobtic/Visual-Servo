#include "AuboSdkExample.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>


#define SERVER_HOST "127.0.0.1"
#define SERVER_PORT 8899


AuboSdkExample::AuboSdkExample()
{
}

/** 实时路点回调函数　**/
void AuboSdkExample::RealTimeWaypointCallback(const aubo_robot_namespace::wayPoint_S *wayPointPtr, void *arg)
{
    (void)arg;
    aubo_robot_namespace::wayPoint_S waypoint = *wayPointPtr;
    printWaypoint(waypoint);
}

/** 实时关节状态回调函数　**/
void AuboSdkExample::RealTimeJointStatusCallback(const aubo_robot_namespace::JointStatus *jointStatus, int size, void *arg)
{
    (void)arg;
    printJointStatus(jointStatus, size);
}


/** 实时末端速度回调函数　**/
void AuboSdkExample::RealTimeEndSpeedCallback(double speed, void *arg)
{
    (void)arg;
    std::cout<<"实时末端速度:"<<speed<<std::endl;
}

/** 实时机械臂事件回调函数　**/
void AuboSdkExample::RealTimeEventInfoCallback(const aubo_robot_namespace::RobotEventInfo *pEventInfo, void *arg)
{
    (void)arg;
    printEventInfo(*pEventInfo);
}





//打印路点信息
void AuboSdkExample::printWaypoint(aubo_robot_namespace::wayPoint_S &wayPoint)
{
    std::cout<<std::endl<<"start-------------路点信息---------------"<<std::endl;
    //位置信息
    std::cout<<"x:"<<wayPoint.cartPos.position.x<<"  ";
    std::cout<<"y:"<<wayPoint.cartPos.position.y<<"  ";
    std::cout<<"z:"<<wayPoint.cartPos.position.z<<std::endl;

    //姿态信息
    std::cout<<"w:"<<wayPoint.orientation.w<<"  ";
    std::cout<<"x:"<<wayPoint.orientation.x<<"  ";
    std::cout<<"y:"<<wayPoint.orientation.y<<"  ";
    std::cout<<"z:"<<wayPoint.orientation.z<<std::endl;

    //    aubo_robot_namespace::Rpy tempRpy;
    //    robotService.quaternionToRPY(wayPoint.orientation,tempRpy);
    //    std::cout<<"RX:"<<tempRpy.rx<<"  RY:"<<tempRpy.ry<<"   RZ:"<<tempRpy.rz<<std::endl;

    //关节信息
    for(int i=0;i<aubo_robot_namespace::ARM_DOF;i++)
    {
        std::cout<<"joint"<<i+1<<":"<<wayPoint.jointpos[i]<<" | "<<wayPoint.jointpos[i]*180.0/M_PI<<std::endl;
    }
}


//打印关节状态信息
void AuboSdkExample::printJointStatus(const aubo_robot_namespace::JointStatus *jointStatus, int len)
{
    std::cout<<std::endl<<"start----------关节状态信息-------" << std::endl;

    for(int i=0; i<len; i++)
    {
        std::cout<<"关节ID:"   <<i<<"  " ;
        std::cout<<"电流:"     <<jointStatus[i].jointCurrentI<<" ";
        std::cout<<"速度:"     <<jointStatus[i].jointSpeedMoto<<" ";
        std::cout<<"关节角:"   <<jointStatus[i].jointPosJ<<" ";
        std::cout<<"电压   :"  <<jointStatus[i].jointCurVol<<" ";
        std::cout<<"温度   :"  <<jointStatus[i].jointCurTemp<<" ";
        std::cout<<"目标电流:"  <<jointStatus[i].jointTagCurrentI<<" ";
        std::cout<<"目标电机速度:" <<jointStatus[i].jointTagSpeedMoto<<" ";
        std::cout<<"目标关节角 :"  <<jointStatus[i].jointTagPosJ<<" ";
        std::cout<<"关节错误   :"  <<jointStatus[i].jointErrorNum <<std::endl;
    }
    std::cout<<std::endl;
}


//打印事件信息
void AuboSdkExample::printEventInfo(const aubo_robot_namespace::RobotEventInfo &eventInfo)
{
    std::cout<<"事件类型:"<<eventInfo.eventType <<"  code:"<<eventInfo.eventCode<<"  内容:"<<eventInfo.eventContent<<std::endl;
}


void AuboSdkExample::printRobotDiagnosis(const aubo_robot_namespace::RobotDiagnosis &robotDiagnosis)
{
    std::cout<<std::endl<<"start----------机械臂统计信息-------" << std::endl;

    std::cout<<std::endl<<"   "<<"CAN通信状态:"<<(int)robotDiagnosis.armCanbusStatus;
    std::cout<<std::endl<<"   "<<"电源当前电流:"<<robotDiagnosis.armPowerCurrent;
    std::cout<<std::endl<<"   "<<"电源当前电压:"<<robotDiagnosis.armPowerVoltage;

    (robotDiagnosis.armPowerStatus)? std::cout<<std::endl<<"   "<<"48V电源状态:开":std::cout<<std::endl<<"   "<<"48V电源状态:关";

    std::cout<<std::endl<<"   "<<"控制箱温度:"<<(int)robotDiagnosis.contorllerTemp;
    std::cout<<std::endl<<"   "<<"控制箱湿度:"<<(int)robotDiagnosis.contorllerHumidity;
    std::cout<<std::endl<<"   "<<"远程关机信号:"<<robotDiagnosis.remoteHalt;
    std::cout<<std::endl<<"   "<<"机械臂软急停:"<<robotDiagnosis.softEmergency;
    std::cout<<std::endl<<"   "<<"远程急停信号:"<<robotDiagnosis.remoteEmergency;
    std::cout<<std::endl<<"   "<<"碰撞检测位:"<<robotDiagnosis.robotCollision;
    std::cout<<std::endl<<"   "<<"进入力控模式标志位:"<<robotDiagnosis.forceControlMode;
    std::cout<<std::endl<<"   "<<"刹车状态:"<<robotDiagnosis.brakeStuats;
    std::cout<<std::endl<<"   "<<"末端速度:"<<robotDiagnosis.robotEndSpeed;
    std::cout<<std::endl<<"   "<<"最大加速度:"<<robotDiagnosis.robotMaxAcc;
    std::cout<<std::endl<<"   "<<"上位机软件状态位:"<<robotDiagnosis.orpeStatus;
    std::cout<<std::endl<<"   "<<"位姿读取使能位:"<<robotDiagnosis.enableReadPose;
    std::cout<<std::endl<<"   "<<"安装位置状态:"<<robotDiagnosis.robotMountingPoseChanged;
    std::cout<<std::endl<<"   "<<"磁编码器错误状态:"<<robotDiagnosis.encoderErrorStatus;
    std::cout<<std::endl<<"   "<<"静止碰撞检测开关:"<<robotDiagnosis.staticCollisionDetect;
    std::cout<<std::endl<<"   "<<"关节碰撞检测:"<<robotDiagnosis.jointCollisionDetect;
    std::cout<<std::endl<<"   "<<"光电编码器不一致错误:"<<robotDiagnosis.encoderLinesError;
    std::cout<<std::endl<<"   "<<"关节错误状态:"<<robotDiagnosis.jointErrorStatus;
    std::cout<<std::endl<<"   "<<"奇异点过速警告:"<<robotDiagnosis.singularityOverSpeedAlarm;
    std::cout<<std::endl<<"   "<<"电流错误警告:"<<robotDiagnosis.robotCurrentAlarm;
    std::cout<<std::endl<<"   "<<"tool error:"<<(int)robotDiagnosis.toolIoError;
    std::cout<<std::endl<<"   "<<"安装位置错位:"<<robotDiagnosis.robotMountingPoseWarning;
    std::cout<<std::endl<<"   "<<"mac缓冲器长度:"<<robotDiagnosis.macTargetPosBufferSize;
    std::cout<<std::endl<<"   "<<"mac缓冲器有效数据长度:"<<robotDiagnosis.macTargetPosDataSize;
    std::cout<<std::endl<<"   "<<"mac数据中断:"<<robotDiagnosis.macDataInterruptWarning;

    std::cout<<std::endl<<"----------------------------------end."<<std::endl;
}

void AuboSdkExample::initJointAngleArray(double *array, double joint0, double joint1, double joint2, double joint3, double joint4, double joint5)
{
    array[0] = joint0;
    array[1] = joint1;
    array[2] = joint2;
    array[3] = joint3;
    array[4] = joint4;
    array[5] = joint5;

}




/**
 * ************************************  AuboSdkExample:机械臂信息获取
 * 测试功能:获取机械臂关节状态和诊断状态
 * 注意：诊断状态必须连接真实机械臂
 *
 *
 *
 *
 */

void AuboSdkExample::example_0()
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


    /** 接口调用: 获取真实臂是否存在 **/
    bool IsRealRobotExist = false;
    ret = robotService.robotServiceGetIsRealRobotExist(IsRealRobotExist);

    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        (IsRealRobotExist)? std::cout<<"真实臂存在."<<std::endl:std::cout<<"真实臂不存在."<<std::endl;
        std::cout<<std::endl;
    }
    else
    {
        std::cerr<<"ERROR:获取机械臂真实臂是否存在失败."<<std::endl;
    }


    /** 接口调用: 主动获取关节状态 **/
    aubo_robot_namespace::JointStatus jointStatus[6];
    ret = robotService.robotServiceGetRobotJointStatus(jointStatus, 6);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        printJointStatus(jointStatus, 6);
    }
    else
    {
        std::cerr<<"ERROR:获取关节状态失败."<<"ret:"<<ret<<std::endl;
    }

    /** 接口调用: 机械臂诊断信息 **/
    aubo_robot_namespace::RobotDiagnosis robotDiagnosisInfo;
    ret = robotService.robotServiceGetRobotDiagnosisInfo(robotDiagnosisInfo);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        printRobotDiagnosis(robotDiagnosisInfo);
    }
    else
    {
        std::cerr<<"ERROR:获取机械臂诊断信息失败."<<std::endl;
    }
    sleep(120);

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}






/**
 * ************************************  AuboSdkExample:关节运动
 * 测试功能:以关节运动的方式运动四个点，循环一次偏移一定的位置
 * setp1：登录
 * step2: 机械臂初始化
 * setp2：初始化运动属性
 * setp3：设置关节型运动的最大加速度
 * setp4：设置关节型运动的最大速度
 * setp5：机械臂运动到零位姿态
 * setp6: 设置偏移量
 * setp7：机械臂以关节运动的方式循环8次运动四个点，每循环一次后每个点位置会像z轴偏移
 * setp8: 退出登录
 */

void AuboSdkExample::example_JointMove()
{

}







/**
 * ************************************  AuboSdkExample:直线运动
 *测试功能：直线运动
 *setp1：登录
 *setp2：运动到初始位姿
 *setp3：初始化运动属性
 *setp4：设置直线型运动的最大加速度
 *setp5：设置直线型运动的最大速度
 *setp6：机械臂直线运动四个位点
 *退出登录
 *
*/

void AuboSdkExample::example_LineMove()
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

    /** 运动到初始位姿 **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    jointAngle[0] = 0.0/180.0*M_PI;
    jointAngle[1] = 0.0/180.0*M_PI;
    jointAngle[2] = 90.0/180.0*M_PI;
    jointAngle[3] = 0.0/180.0*M_PI;
    jointAngle[4] = 90.0/180.0*M_PI;
    jointAngle[5] = 0.0/180.0*M_PI;
    ret = robotService.robotServiceJointMove(jointAngle, true);   //关节运动至准备点
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动0失败.　ret:"<<ret<<std::endl;
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

    for(int i=0;i<8;i++)
    {
        double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
        jointAngle[0] = 0.0/180.0*M_PI;
        jointAngle[1] = 0.0/180.0*M_PI;
        jointAngle[2] = 90.0/180.0*M_PI;
        jointAngle[3] = 0.0/180.0*M_PI;
        jointAngle[4] = 90.0/180.0*M_PI;
        jointAngle[5] = 0.0/180.0*M_PI;
        ret = robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动0失败.　ret:"<<ret<<std::endl;
        }

        jointAngle[0] = 0.0/180.0*M_PI;
        jointAngle[1] = 0.0/180.0*M_PI;
        jointAngle[2] = 45.0/180.0*M_PI;
        jointAngle[3] = 0.0/180.0*M_PI;
        jointAngle[4] = 90.0/180.0*M_PI;
        jointAngle[5] = 0.0/180.0*M_PI;
        robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动1失败.　ret:"<<ret<<std::endl;
        }

        jointAngle[0] = 30.0/180.0*M_PI;
        jointAngle[1] = 0.0/180.0*M_PI;
        jointAngle[2] = 45.0/180.0*M_PI;
        jointAngle[3] = 0.0/180.0*M_PI;
        jointAngle[4] = 90.0/180.0*M_PI;
        jointAngle[5] = 0.0/180.0*M_PI;
        robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动2失败.　ret:"<<ret<<std::endl;
        }

        jointAngle[0] = 30.0/180.0*M_PI;
        jointAngle[1] = 0.0/180.0*M_PI;
        jointAngle[2] = 90.0/180.0*M_PI;
        jointAngle[3] = 0.0/180.0*M_PI;
        jointAngle[4] = 90.0/180.0*M_PI;
        jointAngle[5] = 0.0/180.0*M_PI;
        robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动3失败.　ret:"<<ret<<std::endl;
        }
    }

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}







/**
 * ************************************ AuboSdkExample:轨迹运动
 * 测试功能：轨迹运动之圆弧，圆，movep
 *setp1：登录
 *setp2：运动到初始位姿
 *setp3：初始化关节运动属性
 *setp4：初始化末端型运动属性
 *setp5：循环  准备点  （1）设置关节最大加速度，速度
                     （2）添加关节路点
                     （3）关节运动到该路点

 *setp6：循环  圆弧    （1）设置短短运动最大加速度，速度
                     （2）添加轨迹路点
                     （3）圆的圈数设置为0
                     （4）轨迹运动一个圆弧

 *setp7：循环  准备点  （1）设置关节最大加速度，速度
                     （2）添加关节路点
                     （3）关节运动到该路点

 *setp8：循环  圆     （1）设置短短运动最大加速度，速度
                     （2）添加轨迹路点
                     （3）圆的圈数设置为1
                     （4）轨迹运动一个圆

 *setp9：循环  准备点  （1）设置关节最大加速度，速度
                     （2）添加关节路点
                     （3）关节运动到该路点

 *setp10：循环  movep  （1）设置短短运动最大加速度，速度
                      （2）添加轨迹路点
                      （3）设置交融半径为0.03
                      （4）轨迹运动一个圆弧
 *setp11：退出登录
 *
 *
 *
 *
 */
void AuboSdkExample::example_TrackMove()
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

    for(int i=0;i<10;i++)
    {
        //准备点  关节运动属于关节型运动
        robotService.robotServiceInitGlobalMoveProfile();

        robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
        robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);
        initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
        ret = robotService.robotServiceJointMove(jointAngle, true);   //关节运动至准备点
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"JointMove失败.　ret:"<<ret<<std::endl;
        }


        //圆弧
        robotService.robotServiceInitGlobalMoveProfile();

        robotService.robotServiceSetGlobalMoveEndMaxLineAcc(endMoveMaxAcc);
        robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(endMoveMaxAcc);
        robotService.robotServiceSetGlobalMoveEndMaxLineVelc(endMoveMaxVelc);
        robotService.robotServiceSetGlobalMoveEndMaxAngleVelc(endMoveMaxVelc);
        initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
        robotService.robotServiceAddGlobalWayPoint(jointAngle);
        initJointAngleArray(jointAngle,0.200000, -0.127267, -1.321122, 0.376934, -1.570794, -0.000008);
        robotService.robotServiceAddGlobalWayPoint(jointAngle);
        initJointAngleArray(jointAngle,0.600000, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
        robotService.robotServiceAddGlobalWayPoint(jointAngle);

        robotService.robotServiceSetGlobalCircularLoopTimes(0);    //圆的圈数
        ret = robotService.robotServiceTrackMove(aubo_robot_namespace::ARC_CIR, true);

        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
        }


        //准备点
        robotService.robotServiceInitGlobalMoveProfile();

        robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
        robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);
        initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
        ret = robotService.robotServiceJointMove(jointAngle, true);   //关节运动至准备点
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"JointMove失败.　ret:"<<ret<<std::endl;
        }

        //圆
        robotService.robotServiceInitGlobalMoveProfile();

        robotService.robotServiceSetGlobalMoveEndMaxLineAcc(endMoveMaxAcc);
        robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(endMoveMaxAcc);
        robotService.robotServiceSetGlobalMoveEndMaxLineVelc(endMoveMaxVelc);
        robotService.robotServiceSetGlobalMoveEndMaxAngleVelc(endMoveMaxVelc);
        initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
        robotService.robotServiceAddGlobalWayPoint(jointAngle);
        initJointAngleArray(jointAngle,-0.211675, -0.325189, -1.466753, 0.429232, -1.570794, -0.211680);
        robotService.robotServiceAddGlobalWayPoint(jointAngle);
        initJointAngleArray(jointAngle,-0.037186, -0.224307, -1.398285, 0.396819, -1.570796, -0.037191);
        robotService.robotServiceAddGlobalWayPoint(jointAngle);

        robotService.robotServiceSetGlobalCircularLoopTimes(1);    //圆的圈数
        ret = robotService.robotServiceTrackMove(aubo_robot_namespace::ARC_CIR,true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
        }


        //准备点
        robotService.robotServiceInitGlobalMoveProfile();

        robotService.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
        robotService.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);
        initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
        ret = robotService.robotServiceJointMove(jointAngle, true);   //关节运动至准备点
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"JointMove失败.　ret:"<<ret<<std::endl;
        }

        //MoveP
        robotService.robotServiceInitGlobalMoveProfile();

        robotService.robotServiceSetGlobalMoveEndMaxLineAcc(endMoveMaxAcc);
        robotService.robotServiceSetGlobalMoveEndMaxAngleAcc(endMoveMaxAcc);
        robotService.robotServiceSetGlobalMoveEndMaxLineVelc(endMoveMaxVelc);
        robotService.robotServiceSetGlobalMoveEndMaxAngleVelc(endMoveMaxVelc);
        initJointAngleArray(jointAngle,-0.000003, -0.127267, -1.321122, 0.376934, -1.570796, -0.000008);
        robotService.robotServiceAddGlobalWayPoint(jointAngle);
        initJointAngleArray(jointAngle,0.100000, -0.147267, -1.321122, 0.376934, -1.570794, -0.000008);
        robotService.robotServiceAddGlobalWayPoint(jointAngle);
        initJointAngleArray(jointAngle,0.200000, -0.167267, -1.321122, 0.376934, -1.570796, -0.000008);
        robotService.robotServiceAddGlobalWayPoint(jointAngle);

        robotService.robotServiceSetGlobalBlendRadius(0.03);                     //交融半径
        ret = robotService.robotServiceTrackMove(aubo_robot_namespace::CARTESIAN_MOVEP,true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"TrackMove failed.　ret:"<<ret<<std::endl;
        }
}
    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}







/**
 * ************************************  AuboSdkExample:保持当前位姿通过直线运动的方式运动到目标位置
 *
 *setp1:初始化关节最大加速度，最大速度
 *setp2:机械臂以关节运动的方式到初始位姿
 *setp3:初始化末端运动最大加速度，最大速度
 *setp4:获取机械臂当前路点信息
 *setp5:设置坐标系参数
 *setp6:设置工具端参数
 *setp7:设置位置参数（循环）
 *setp8:保持当前运动姿态以直线运动方式运动到目标位置（循环）
 *
 */

void AuboSdkExample::example_MoveLtoPosition()
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

    /** 机械臂运动到准备位置 **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    jointAngle[0] = 0.0/180.0*M_PI;
    jointAngle[1] = 0.0/180.0*M_PI;
    jointAngle[2] = 90.0/180.0*M_PI;
    jointAngle[3] = 0.0/180.0*M_PI;
    jointAngle[4] = 90.0/180.0*M_PI;
    jointAngle[5] = 0.0/180.0*M_PI;
    ret = robotService.robotServiceJointMove(jointAngle, true);   //关节运动至准备点
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动0失败.　ret:"<<ret<<std::endl;
    }


    /** 接口调用: 初始化运动属性 ***/
    robotService.robotServiceInitGlobalMoveProfile();

    /** 接口调用: 设置末端型运动的最大加速度 　　直线运动属于末端型运动***/
    double lineMoveMaxAcc;
    lineMoveMaxAcc = 2;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineAcc(lineMoveMaxAcc);

    /** 接口调用: 设置末端型运动的最大速度 直线运动属于末端型运动 ***/
    double lineMoveMaxVelc;
    lineMoveMaxVelc = 2;   //单位米每秒
    robotService.robotServiceSetGlobalMoveEndMaxLineVelc(lineMoveMaxVelc);

    aubo_robot_namespace::wayPoint_S currentWaypoint;
    ret = robotService.robotServiceGetCurrentWaypointInfo(currentWaypoint);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"获取当前路点信息失败.　ret:"<<ret<<std::endl;
    }
    /** 设置坐标系参数 **/
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;
    userCoord.coordType = aubo_robot_namespace::BaseCoordinate;

    /** 设置工具端参数 **/
    aubo_robot_namespace::ToolInEndDesc toolDesc;
    toolDesc.toolInEndPosition.x = 0;
    toolDesc.toolInEndPosition.y = 0;
    toolDesc.toolInEndPosition.z = 0;

    toolDesc.toolInEndOrientation.w=1;
    toolDesc.toolInEndOrientation.x=0;
    toolDesc.toolInEndOrientation.y=0;
    toolDesc.toolInEndOrientation.z=0;


    for(int i=0;i<10;i++)
    {

        aubo_robot_namespace::Pos position;
        double offsetY = (i%3 > 1)? 0.0:(0.18*(i%3));
        double offsetZ = (i%3 > 1)? 0.0:(0.18);
        position.x = currentWaypoint.cartPos.position.x;
        position.y = currentWaypoint.cartPos.position.y+offsetY;
        position.z = currentWaypoint.cartPos.position.z+offsetZ;

        //保持当前位姿通过直线运动的方式运动到目标位置
        ret = robotService.robotMoveLineToTargetPosition(userCoord, position, toolDesc, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"robotMoveLineToTargetPosition.　ret:"<<ret<<std::endl;
        }

    }

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}








/**
 * ************************************  AuboSdkExample:保持当前位姿通过关节运动的方式运动到目标位置
 *
 *setp1:初始化关节最大加速度，最大速度
 *setp2:机械臂以关节运动的方式到初始位姿
 *setp3:获取机械臂当前路点信息
 *setp4:设置坐标系参数
 *setp5:设置工具端参数
 *setp6:设置位置参数（循环）
 *setp7:保持当前运动姿态以关节运动方式运动到目标位置（循环）
 */
void AuboSdkExample::example_MoveJtoPosition()
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

    /** 机械臂运动到准备位置 **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    jointAngle[0] = 0.0/180.0*M_PI;
    jointAngle[1] = 0.0/180.0*M_PI;
    jointAngle[2] = 90.0/180.0*M_PI;
    jointAngle[3] = 0.0/180.0*M_PI;
    jointAngle[4] = 90.0/180.0*M_PI;
    jointAngle[5] = 0.0/180.0*M_PI;
    ret = robotService.robotServiceJointMove(jointAngle, true);   //关节运动至准备点
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动0失败.　ret:"<<ret<<std::endl;
    }

    aubo_robot_namespace::wayPoint_S currentWaypoint;
    ret = robotService.robotServiceGetCurrentWaypointInfo(currentWaypoint);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"获取当前路点信息失败.　ret:"<<ret<<std::endl;
    }
    /** 设置坐标系参数 **/
    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool userCoord;
    userCoord.coordType = aubo_robot_namespace::BaseCoordinate;
    /** 设置工具端参数 **/
    aubo_robot_namespace::ToolInEndDesc toolDesc;
    toolDesc.toolInEndPosition.x = 0;
    toolDesc.toolInEndPosition.y = 0;
    toolDesc.toolInEndPosition.z = 0;

    for(int i=0;i<10;i++)
    {

        aubo_robot_namespace::Pos position;
        double offsetY = (i%3 > 1)? 0.0:(0.18*(i%3));
        double offsetZ = (i%3 > 1)? 0.0:(0.18);
        position.x = currentWaypoint.cartPos.position.x;
        position.y = currentWaypoint.cartPos.position.y+offsetY;
        position.z = currentWaypoint.cartPos.position.z+offsetZ;

        //保持当前位姿通过直线运动的方式运动到目标位置
        ret = robotService.robotMoveJointToTargetPosition(userCoord, position, toolDesc, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"robotMoveLineToTargetPosition.　ret:"<<ret<<std::endl;
        }

    }

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}

void AuboSdkExample::example_quaternion_RPY_Conversion()
{
    ServiceInterface robotService;
    double jointAngle[6]={0};
    aubo_robot_namespace::Ori ori;
    aubo_robot_namespace::Rpy rpy;
    aubo_robot_namespace::wayPoint_S wayPoint;

    //该位置为机械臂的初始位置
    jointAngle[0]= -0.000172/180*M_PI;
    jointAngle[1]= -7.291862/180*M_PI;
    jointAngle[2]=-75.694718/180*M_PI;
    jointAngle[3]= 21.596727/180*M_PI;
    jointAngle[4]=-89.999982/180*M_PI;
    jointAngle[5]=  -0.00458/180*M_PI;

    //正解
    robotService.robotServiceRobotFk(jointAngle, 6, wayPoint);

    ori.w = wayPoint.orientation.w;
    ori.x = wayPoint.orientation.x;
    ori.y = wayPoint.orientation.y;
    ori.z = wayPoint.orientation.z;

    robotService.quaternionToRPY(ori, rpy);
    std::cout<<"第一次结果:"<<"RX:"<<rpy.rx<<"  RY:"<<rpy.ry<<"  RZ:"<<rpy.rz<<std::endl;

    robotService.RPYToQuaternion(rpy, ori);
    robotService.quaternionToRPY(ori, rpy);
    std::cout<<"第二次结果:"<<"RX:"<<rpy.rx<<"  RY:"<<rpy.ry<<"  RZ:"<<rpy.rz<<std::endl;
}



void AuboSdkExample::example_ToolIO()
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


    sleep(2);   //延时

    /** 设置工具端IO的电源和数字量IO的类型 **/
    ret = robotService.robotServiceSetToolPowerTypeAndDigitalIOType(aubo_robot_namespace::OUT_12V,
                                                              aubo_robot_namespace::IO_IN, aubo_robot_namespace::IO_IN,
                                                              aubo_robot_namespace::IO_IN, aubo_robot_namespace::IO_IN);
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"设置工具端IO类型SUCC."<<std::endl;
    }
    else
    {
        std::cerr<<"设置工具端IO类型Failed."<<std::endl;
    }

    sleep(2);   //延时
    statusVector.clear();
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






/**
 * ************************************  AuboSdkExample:测试离线轨迹
 *setp1：登录
 *setp2：清除路点
 *setp3：设置路点集
 *setp4：追加离线轨迹路点
 *setp5：离线轨迹启动
 *
 *
 *注意：离线轨迹文件需要添加
 */
void AuboSdkExample::testOfflineTrack()
{

    ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 1:接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"登录成功."<<std::endl;
    }
    else
    {
        std::cerr<<"登录失败."<<std::endl;

        return ;
    }
    /** 清除路点 **/
    robotService.robotServiceOfflineTrackWaypointClear();
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cout<<"robotServiceOfflineTrackWaypointClear failed"<<std::endl;
        return ;
    }

    /** 追加离线轨迹路点 **/
    ret = robotService.robotServiceOfflineTrackWaypointAppend("./track.txt");

    /** 启动离线轨迹 **/
    if(ret == aubo_robot_namespace::ErrnoSucc)
    {
        std::cout<<"upload waypoint success."<<std::endl;

        ret = robotService.robotServiceOfflineTrackMoveStartup(true);

        if(ret != aubo_robot_namespace::ErrnoSucc)
        {
            std::cout<<"failed."<<std::endl;
        }
    }
    else
    {
        std::cout<<"upload waypoint failed."<<std::endl;
    }
}






/**
 * ************************************  AuboSdkExample:机械臂仿真模式,真实模式
 *setp1：登录
 *setp2：获取机械臂状态
 *
 *
 */


void AuboSdkExample::testWorkMode()
{
    ServiceInterface robotService;

    int ret = aubo_robot_namespace::InterfaceCallSuccCode;

    /** 1:接口调用: 登录 ***/
    ret = robotService.robotServiceLogin(SERVER_HOST, SERVER_PORT, "aubo", "123456");
    if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"登录成功."<<std::endl;
    }
    else
    {
        std::cerr<<"登录失败."<<std::endl;

        return ;
    }

    for(int i=0;i<10;i++)
    {
        aubo_robot_namespace::RobotWorkMode mode = aubo_robot_namespace::RobotModeSimulator;

        ret = robotService.robotServiceGetRobotWorkMode(mode);

        if(ret == aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cout<<"mode:"<<mode<<std::endl;
        }
        else
        {
            std::cout<<"ERROR"<<std::endl;
        }
    }
}






/**
  * ************************************  AuboSdkExample:跟随模式之提前到位　当前仅适用于关节运动
  *setp1：登录
  *setp2：初始化关节最大速度，最大加速度
  *setp3:循环->两点间运动（切换模式：是否为跟随模式）
  *
  *
  *
  *
  *
  */
void AuboSdkExample::testArriveAhead()
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

    /** 机械臂运动到零位姿态 **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    jointAngle[0] = 0.0/180.0*M_PI;
    jointAngle[1] = 0.0/180.0*M_PI;
    jointAngle[2] = 0.0/180.0*M_PI;
    jointAngle[3] = 0.0/180.0*M_PI;
    jointAngle[4] = 0.0/180.0*M_PI;
    jointAngle[5] = 0.0/180.0*M_PI;
    ret = robotService.robotServiceJointMove(jointAngle, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动0失败.　ret:"<<ret<<std::endl;
    }

    for(int i=0;i<10;i++)
    {
        if(i%2==0)
        {
            /** 跟随模式之提前到位　当前仅适用于关节运动 **/
            robotService.robotServiceSetArrivalAheadDistanceMode(0.2);
        }
        else
        {
            robotService.robotServiceSetNoArrivalAhead();
        }


        /** 接口调用: 设置偏移量**/
        jointAngle[0] = 20.0/180.0*M_PI;
        jointAngle[1] = 0.0/180.0*M_PI;
        jointAngle[2] = 90.0/180.0*M_PI;
        jointAngle[3] = 0.0/180.0*M_PI;
        jointAngle[4] = 90.0/180.0*M_PI;
        jointAngle[5] = 0.0/180.0*M_PI;
        robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动1失败.　ret:"<<ret<<std::endl;
            break;
        }
        else
        {
            std::cerr<<"运动1success. i:"<<i<<std::endl;
        }

        jointAngle[0] = 50.0/180.0*M_PI;
        jointAngle[1] = 40.0/180.0*M_PI;
        jointAngle[2] = 78.0/180.0*M_PI;
        jointAngle[3] = 20.0/180.0*M_PI;
        jointAngle[4] = 66.0/180.0*M_PI;
        jointAngle[5] = 0.0/180.0*M_PI;
        robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动2失败.　ret:"<<ret<<std::endl;
             break;
        }
        else
        {
            std::cerr<<"运动2success. i:"<<i<<std::endl;
        }
    }

    std::cout<<"------------------"<<std::endl;
}






/**
 * ************************************  AuboSdkExample:获取机械臂IO
 * setp1:登录
 * setp2:获取机械臂输入DI_01状态
 * setp3:设置机械臂输出DO_01状态
 *
 *
 *
 *
 *
 *
 */

void AuboSdkExample::testIo()
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

    double value;

    robotService.robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDI, "U_DI_01", value);

    std::cerr<<"DI_01状态:"<<value<<std::endl;

    robotService.robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotBoardUserDO, "U_DO_02", 0);
}







/**
 * @brief AuboSdkExample::Example_0
 *
 * 第一步：登录　　　　　　　登录接口调用成功是其他接口能够被调用的前提
 * 第二步：机械臂初始化　　　主要对机械臂上电－释放刹车－设置碰撞等级－设置工具参数等
 * 第三步：业务操作...
 * 第四步：机械臂shutdown
 * 第五步：Logout　　　断开网络连接
 *
 */
void AuboSdkExample::Example_0()
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

    sleep(5);   //模拟业务

    /** 机械臂Shutdown **/
    robotService.robotServiceRobotShutdown();

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}




void AuboSdkExample::Example_1()
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


    robotService.robotServiceRegisterRealTimeRoadPointCallback(AuboSdkExample::RealTimeWaypointCallback, NULL);

    robotService.robotServiceRegisterRealTimeEndSpeedCallback(AuboSdkExample::RealTimeEndSpeedCallback, NULL);

    robotService.robotServiceRegisterRobotEventInfoCallback(AuboSdkExample::RealTimeEventInfoCallback, NULL);


    sleep(100);


    /** 机械臂Shutdown **/
    robotService.robotServiceRobotShutdown();

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}



void AuboSdkExample::Example_3()
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

    /** 机械臂运动到零位姿态 **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    jointAngle[0] = 0.0/180.0*M_PI;
    jointAngle[1] = 0.0/180.0*M_PI;
    jointAngle[2] = 0.0/180.0*M_PI;
    jointAngle[3] = 0.0/180.0*M_PI;
    jointAngle[4] = 0.0/180.0*M_PI;
    jointAngle[5] = 0.0/180.0*M_PI;             //接口要求单位是弧度
    ret = robotService.robotServiceJointMove(jointAngle, true);
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动至零位姿态失败.　ret:"<<ret<<std::endl;
    }


    for(int i=0;i<8;i++)
    {
        /** 接口调用: 设置偏移量**/
        aubo_robot_namespace::MoveRelative relativeMoveOnBase;
        relativeMoveOnBase.ena = true;
        relativeMoveOnBase.relativePosition[0] = 0;
        relativeMoveOnBase.relativePosition[1] = 0;
        relativeMoveOnBase.relativePosition[2] = 0.05*(i%4);   //单位米
        robotService.robotServiceSetMoveRelativeParam(relativeMoveOnBase);


        /** 路点1运动 **/
        jointAngle[0] = 0.0/180.0*M_PI;
        jointAngle[1] = 0.0/180.0*M_PI;
        jointAngle[2] = 90.0/180.0*M_PI;
        jointAngle[3] = 0.0/180.0*M_PI;
        jointAngle[4] = 90.0/180.0*M_PI;
        jointAngle[5] = 0.0/180.0*M_PI;
        robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动至路点1失败.　ret:"<<ret<<std::endl;
            break;
        }

        /** 路点2运动 **/
        jointAngle[0] = 15.0/180.0*M_PI;
        jointAngle[1] = 0.0/180.0*M_PI;
        jointAngle[2] = 90.0/180.0*M_PI;
        jointAngle[3] = 0.0/180.0*M_PI;
        jointAngle[4] = 90.0/180.0*M_PI;
        jointAngle[5] = 0.0/180.0*M_PI;
        robotService.robotServiceJointMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动至路点2失败.　ret:"<<ret<<std::endl;
            break;
        }
    }

    /** 机械臂Shutdown **/
    robotService.robotServiceRobotShutdown();

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}

void AuboSdkExample::Example_4()
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


    /** 业务块 **/
    /** 运动到初始位姿 **/
    double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
    jointAngle[0] = 0.0/180.0*M_PI;
    jointAngle[1] = 0.0/180.0*M_PI;
    jointAngle[2] = 90.0/180.0*M_PI;
    jointAngle[3] = 0.0/180.0*M_PI;
    jointAngle[4] = 90.0/180.0*M_PI;
    jointAngle[5] = 0.0/180.0*M_PI;
    ret = robotService.robotServiceJointMove(jointAngle, true);   //关节运动至准备点
    if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    {
        std::cerr<<"运动0失败.　ret:"<<ret<<std::endl;
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

    for(int i=0;i<8;i++)
    {
        double jointAngle[aubo_robot_namespace::ARM_DOF] = {0};
        jointAngle[0] = 0.0/180.0*M_PI;
        jointAngle[1] = 0.0/180.0*M_PI;
        jointAngle[2] = 90.0/180.0*M_PI;
        jointAngle[3] = 0.0/180.0*M_PI;
        jointAngle[4] = 90.0/180.0*M_PI;
        jointAngle[5] = 0.0/180.0*M_PI;
        ret = robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动0失败.　ret:"<<ret<<std::endl;
        }

        jointAngle[0] = 0.0/180.0*M_PI;
        jointAngle[1] = 0.0/180.0*M_PI;
        jointAngle[2] = 45.0/180.0*M_PI;
        jointAngle[3] = 0.0/180.0*M_PI;
        jointAngle[4] = 90.0/180.0*M_PI;
        jointAngle[5] = 0.0/180.0*M_PI;
        robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动1失败.　ret:"<<ret<<std::endl;
        }

        jointAngle[0] = 30.0/180.0*M_PI;
        jointAngle[1] = 0.0/180.0*M_PI;
        jointAngle[2] = 45.0/180.0*M_PI;
        jointAngle[3] = 0.0/180.0*M_PI;
        jointAngle[4] = 90.0/180.0*M_PI;
        jointAngle[5] = 0.0/180.0*M_PI;
        robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动2失败.　ret:"<<ret<<std::endl;
        }

        jointAngle[0] = 30.0/180.0*M_PI;
        jointAngle[1] = 0.0/180.0*M_PI;
        jointAngle[2] = 90.0/180.0*M_PI;
        jointAngle[3] = 0.0/180.0*M_PI;
        jointAngle[4] = 90.0/180.0*M_PI;
        jointAngle[5] = 0.0/180.0*M_PI;
        robotService.robotServiceLineMove(jointAngle, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
        {
            std::cerr<<"运动3失败.　ret:"<<ret<<std::endl;
        }
    }


    /** 机械臂Shutdown **/
    robotService.robotServiceRobotShutdown();

    /** 接口调用: 退出登录　**/
    robotService.robotServiceLogout();
}

void AuboSdkExample::Example_5()
{

}
