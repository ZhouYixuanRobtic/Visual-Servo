//
// Created by ZYX on 19-6-26.
//
/*
 * AUBO 机械臂SDK 应用
 * 主要功能：
 * 1. 启动关闭机械臂。 2. 监听机械臂事件特别是过速,碰撞，急停。
 * 3. 响应机械臂事件并恢复。 4. 机械臂停止运动
 * 5. 读取IO状态
 */
#ifndef VISUAL_SERVO_AUBOSDK_H
#define VISUAL_SERVO_AUBOSDK_H

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"
#include "util.h"
bool ExitSoftEmergency=false;
bool RobotMoveStop=false;
bool RobotCollisionRecover=false;
class AuboSDK{
private:
    ServiceInterface *robotService;
    int ret;

    /** 如果是连接真实机械臂，需要对机械臂进行初始化　**/
    aubo_robot_namespace::ROBOT_SERVICE_STATE result{};
    static void RealTimeEventInfoCallback(const aubo_robot_namespace::RobotEventInfo *pEventInfo, void *arg);
public:
    AuboSDK();
    bool loginSucceed;
    virtual ~AuboSDK();
    bool robotStartUp();
    int robotShutDown();
    bool robotEventRegister();
    bool OverturnIOStatus();
    bool robotFastMoveStop();
    bool robotFastMoveRelease();
    bool toolStart();
    bool toolAllclear();
    int robotDiagno();


};


#endif //VISUAL_SERVO_AUBOSDK_H
