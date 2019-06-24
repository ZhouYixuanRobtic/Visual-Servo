#include "util.h"

#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <sstream>
#include <fstream>

Util::Util()
{
}


//打印路点信息
void Util::printWaypoint(aubo_robot_namespace::wayPoint_S &wayPoint)
{
    std::cout<<std::endl<<"start-------------路点信息---------------"<<std::endl;
    //位置信息
    std::cout<<"位置信息: ";
    std::cout<<"x:"<<wayPoint.cartPos.position.x<<"  ";
    std::cout<<"y:"<<wayPoint.cartPos.position.y<<"  ";
    std::cout<<"z:"<<wayPoint.cartPos.position.z<<std::endl;

    //姿态信息
    std::cout<<"姿态信息: ";
    std::cout<<"w:"<<wayPoint.orientation.w<<"  ";
    std::cout<<"x:"<<wayPoint.orientation.x<<"  ";
    std::cout<<"y:"<<wayPoint.orientation.y<<"  ";
    std::cout<<"z:"<<wayPoint.orientation.z<<std::endl;

    //    aubo_robot_namespace::Rpy tempRpy;
    //    robotService.quaternionToRPY(wayPoint.orientation,tempRpy);
    //    std::cout<<"RX:"<<tempRpy.rx<<"  RY:"<<tempRpy.ry<<"   RZ:"<<tempRpy.rz<<std::endl;

    //关节信息
    std::cout<<"关节信息: "<<std::endl;
    for(int i=0;i<aubo_robot_namespace::ARM_DOF;i++)
    {
        std::cout<<"joint"<<i+1<<": "<<wayPoint.jointpos[i]<<" ~ "<<wayPoint.jointpos[i]*180.0/M_PI<<std::endl;
    }
}


//打印关节状态信息
void Util::printJointStatus(const aubo_robot_namespace::JointStatus *jointStatus, int len)
{
    std::cout<<std::endl<<"start----------关节状态信息-------" << std::endl;

    for(int i=0; i<len; i++)
    {
        std::cout<<"关节ID:"   <<i<<"  " ;
        std::cout<<"电流:"     <<jointStatus[i].jointCurrentI<<" ";
        std::cout<<"速度:"     <<jointStatus[i].jointSpeedMoto<<" ";
        std::cout<<"关节角:"   <<jointStatus[i].jointPosJ<<" "<<" ~ "<<jointStatus[i].jointPosJ*180.0/M_PI;
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
void Util::printEventInfo(const aubo_robot_namespace::RobotEventInfo &eventInfo)
{
    std::cout<<"事件类型:"<<eventInfo.eventType <<"  code:"<<eventInfo.eventCode<<"  内容:"<<eventInfo.eventContent<<std::endl;
}


void Util::printRobotDiagnosis(const aubo_robot_namespace::RobotDiagnosis &robotDiagnosis)
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

void Util::initJointAngleArray(double *array, double joint0, double joint1, double joint2, double joint3, double joint4, double joint5)
{
    array[0] = joint0;
    array[1] = joint1;
    array[2] = joint2;
    array[3] = joint3;
    array[4] = joint4;
    array[5] = joint5;
}

