#ifndef VISUAL_SERVO_JOYTELEOP_H
#define VISUAL_SERVO_JOYTELEOP_H
#include <utility>

#include"ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <ros/callback_queue.h>
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
        toolClear,
        toolStart,
        toolWork,
        toolReset,
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
        void registerAutoThread();
        const ControlTrigger & getControlTrigger() const { return  control_trigger_;};
        void resetControlTrigger(const ControlTrigger& value){control_trigger_=value;};
    };
}



#endif //VISUAL_SERVO_JOYTELEOP_H
