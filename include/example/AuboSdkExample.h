#ifndef AUBO_SDK_EXAMPLE_H
#define AUBO_SDK_EXAMPLE_H

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"


/** 要写清楚demo运行步骤
   * step1: robotServiceOfflineTrackWaypointClear
   * step2: robotServiceOfflineTrackWaypointAppend
   * step3: robotServiceOfflineTrackMoveStartup
   */



class AuboSdkExample
{
public:
    AuboSdkExample();

public:
    /** 打印路点信息 **/
    static void printWaypoint(aubo_robot_namespace::wayPoint_S &wayPoint);

    /** 打印关节状态信息 **/
    static void printJointStatus(const aubo_robot_namespace::JointStatus *jointStatus, int len);

    /** 打印事件信息 **/
    static void printEventInfo(const aubo_robot_namespace::RobotEventInfo &eventInfo);

    /** 打印诊断信息 **/
    static void printRobotDiagnosis(const aubo_robot_namespace::RobotDiagnosis &robotDiagnosis);


private:


    /** 实时关节状态回调函数　**/
    static void RealTimeJointStatusCallback (const aubo_robot_namespace::JointStatus *jointStatus, int size, void *arg);




private:
    static void initJointAngleArray(double *array, double joint0,double joint1,double joint2,double joint3,double joint4,double joint5);


public:
    /** 关于一些信息获取的示例　**/
    void example_0();

    void example_JointMove();

    void example_LineMove();

    void example_TrackMove();

    void example_MoveLtoPosition();

    void example_MoveJtoPosition();

public:
    /** 四元素和欧拉角相互转化示例 **/
    void example_quaternion_RPY_Conversion();


public:  //关于工具端的实例函数
    void example_ToolIO();

public:

    void testOfflineTrack();        //测试离线轨迹

    void testWorkMode();            //机械臂仿真模式,真实模式

    void testArriveAhead();         //跟随模式测试
    //类似于交融半径，几个点之间运动到某一个为至点后不停留

    void testIo();                  //IO状态测试（需要连接实际的机器人）




public:

    /**
     * @brief Example_0
     *
     * 使用SDK构建一个最简单的机械臂的控制工程
     *
     */
    void  Example_0();



    static void RealTimeWaypointCallback (const aubo_robot_namespace::wayPoint_S *wayPointPtr, void *arg); //用于获取实时路点回调函数

    static void RealTimeEndSpeedCallback (double speed, void *arg);  //获取实时末端速度回调函数

    static void RealTimeEventInfoCallback(const aubo_robot_namespace::RobotEventInfo *pEventInfo, void *arg); //获取实时机械臂事件回调函数

    /**
     * @brief Example_1
     *
     * 回调函数的方式获取实时路点，末端速度，机械臂的事件
     *
     *
     */
    void  Example_1();


    /**
     * @brief Example_３
     *
     * 关节运动
     */
    void  Example_3();

    /**
     * @brief Example_4
     *
     * 直线运动
     */
    void  Example_4();

    /**
     * @brief Example_5
     *
     * 轨迹运动
     */
    void  Example_5();




};

#endif // AUBO_SDK_EXAMPLE_H
