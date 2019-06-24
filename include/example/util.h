#ifndef UTIL_H
#define UTIL_H

#include "AuboRobotMetaType.h"
#include "serviceinterface.h"

class Util
{
public:
    Util();

    /** 打印路点信息 **/
    static void printWaypoint(aubo_robot_namespace::wayPoint_S &wayPoint);

    /** 打印关节状态信息 **/
    static void printJointStatus(const aubo_robot_namespace::JointStatus *jointStatus, int len);

    /** 打印事件信息 **/
    static void printEventInfo(const aubo_robot_namespace::RobotEventInfo &eventInfo);

    /** 打印诊断信息 **/
    static void printRobotDiagnosis(const aubo_robot_namespace::RobotDiagnosis &robotDiagnosis);


    static void initJointAngleArray(double *array, double joint0,double joint1,double joint2,double joint3,double joint4,double joint5);
};

#endif // UTIL_H
