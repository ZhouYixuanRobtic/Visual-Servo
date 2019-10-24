#include "JoyTeleop.h"
using namespace JOYTELEOP;

JoyTeleop::JoyTeleop(std::string topic_name): TOPIC_NAME_(std::move(topic_name))
{
    control_trigger_=Default;
    Joy_sub_  = nh_.subscribe(TOPIC_NAME_,100,&JoyTeleop::JoyCallback,this);
    watchdog_timer_ = nh_.createTimer(ros::Duration(WATCHDOG_PERIOD_), &JoyTeleop::watchdog, this, true);
    watchdog_timer_.start();
}
JoyTeleop::~JoyTeleop()
{

}
void JoyTeleop::watchdog(const ros::TimerEvent &e)
{
    ROS_WARN("joy not received for %f seconds, is the joy node drop?", WATCHDOG_PERIOD_);
    this->joy_alive_=false;
}
void JoyTeleop::JoyCallback(const sensor_msgs::JoyConstPtr &msg)
{
    watchdog_timer_.stop();
    watchdog_timer_.start();
    if(msg->axes[2]==-1&&msg->buttons[0]) //A
    {
        control_trigger_=ManipulateOn;
    }
    else if(msg->axes[2]==-1&&msg->buttons[1]) //B
    {
        control_trigger_=ChargeOn;
    }
    else if(msg->axes[2]==-1&&msg->buttons[2]) //X
    {
        control_trigger_=HomeOn;
    }
    else if(msg->axes[2]==-1&&msg->buttons[3]) //Y
    {
        control_trigger_=UpOn;
    }
    else if(msg->axes[5]==-1&&msg->buttons[0]) //A
    {
        control_trigger_=toolClear;
    }
    else if(msg->axes[5]==-1&&msg->buttons[1]) //B
    {
        control_trigger_=toolStart;
    }
    else if(msg->axes[5]==-1&&msg->buttons[2]) //X
    {
        control_trigger_=toolWork;
    }
    else if(msg->axes[5]==-1&&msg->buttons[3]) //Y
    {
        control_trigger_=toolReset;
    }
    else if(msg->buttons[4]) //L1
    {
        control_trigger_=NavOn;
    }
    else if(msg->buttons[5]) //R1
    {
        control_trigger_=NavPause;
    }
    else
        control_trigger_ = Default;
}