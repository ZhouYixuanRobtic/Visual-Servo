#ifndef VISUAL_SERVO_JOYTELEOP_H
#define VISUAL_SERVO_JOYTELEOP_H
#include <utility>

#include"ros/ros.h"
#include "sensor_msgs/Joy.h"
namespace JOYTELEOP
{
    enum ControlTrigger{
        Default,
        NavOn,
        NavPause,
        ManipulateOn,
        ChargeOn,
        UpOn,
        HomeOn,
        AntiClockGo,
        ClockGo,
        KnifeOn,
        KnifeOff,
        KnifeUnplug,
        MappingOn,
        MappingOff,
        RobotArmOn,
        NavigationOn,
        ShutDown,
        SaveCutPoint,
        SaveChargePoint,
        SaveNavPoint,
        SaveTurnPoint,
    };
    class JoyTeleop {
    private:
        const std::string TOPIC_NAME_;
        const double WATCHDOG_PERIOD_=2.0;

        ros::NodeHandle nh_;
        ros::Timer watchdog_timer_;
        ros::Subscriber Joy_sub_;
        bool joy_alive_{};
        ControlTrigger control_trigger_;
    public:
        explicit JoyTeleop(std::string topic_name);
        ~JoyTeleop();
        void JoyCallback(const sensor_msgs::JoyConstPtr & msg);
        void watchdog(const ros::TimerEvent &e);
        ControlTrigger getControlTrigger()  {ControlTrigger temp{control_trigger_};control_trigger_=Default;return temp;};
    };
}



#endif //VISUAL_SERVO_JOYTELEOP_H
