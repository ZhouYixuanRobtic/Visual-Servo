#ifndef VISUAL_SERVO_JOYTELEOP_H
#define VISUAL_SERVO_JOYTELEOP_H
#include <utility>

#include"ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include "atomic"
namespace CONTROLTELEOP
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
        SingleAntiClockGo,
        SingleClockGo,
        KnifeOn,
        KnifeOff,
        KnifeUnplug,
        LightOn,
        LightOff,
		SteeringIn,
		SteeringOut,
        MappingOn,
        MappingOff,
        RobotArmOn,
        NavigationOn,
        ShutDown,
        SaveCutPoint,
        SaveChargePoint,
        SaveNavPoint,
        SaveTurnPoint,
        ArmEmergencyChange,
		LinearUp,
		LinearDown,
		LinearForward,
		LinearBack,
		CutBack
    };
    class ControlTeleop {
    private:
        const double WATCHDOG_PERIOD_=2.0;
        ros::NodeHandle nh_;
        ros::Timer joy_watchdog_timer_,bl_watchdog_timer_;
        ros::Subscriber Joy_sub_,bl_content_sub_;
        ros::Publisher joy_vel_pub;
        std::atomic_bool joy_alive_{},bl_alive_{};
        bool publish_vel_{};
        ControlTrigger control_trigger_{};
        double max_linear_velocity_{};
        double max_angular_velocity_{};
        mutable boost::shared_mutex control_trigger_mutex_;
    public:
        explicit ControlTeleop(bool publish_vel=false,double max_linear_velocity=0.8,double max_angular_velocity=0.5);
        ~ControlTeleop() = default;
        void JoyCallback(const sensor_msgs::JoyConstPtr & msg);
        void BLCallback(const std_msgs::StringConstPtr & msg);
        void Joywatchdog(const ros::TimerEvent &e);
        void BLwatchdog(const ros::TimerEvent &e);
        ControlTrigger getControlTrigger()
        {
            boost::unique_lock<boost::shared_mutex> writLock(control_trigger_mutex_);
            ControlTrigger temp{control_trigger_};
            control_trigger_=Default;
            return temp;
        };
    };
}



#endif //VISUAL_SERVO_JOYTELEOP_H
