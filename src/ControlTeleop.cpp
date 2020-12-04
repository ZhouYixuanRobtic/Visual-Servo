#include "ControlTeleop.h"
using namespace CONTROLTELEOP;

ControlTeleop::ControlTeleop(bool publish_vel,double max_linear_velocity,double max_angular_velocity)
{
    control_trigger_=Default;
    publish_vel_=publish_vel;
    Joy_sub_  = nh_.subscribe("joy",100,&ControlTeleop::JoyCallback,this);
    bl_content_sub_ = nh_.subscribe("bl_content",100,&ControlTeleop::BLCallback,this);
    if(publish_vel)
    {
        max_angular_velocity_=max_angular_velocity;
        max_linear_velocity_ = max_linear_velocity;
        joy_vel_pub = nh_.advertise<geometry_msgs::Twist>("/joy_vel", 1);
    }
    joy_watchdog_timer_ = nh_.createTimer(ros::Duration(WATCHDOG_PERIOD_), &ControlTeleop::Joywatchdog, this, true);
    joy_watchdog_timer_.start();
    bl_watchdog_timer_ = nh_.createTimer(ros::Duration(WATCHDOG_PERIOD_), &ControlTeleop::BLwatchdog, this, true);
    bl_watchdog_timer_.start();
}

void ControlTeleop::Joywatchdog(const ros::TimerEvent &e)
{
    ROS_WARN("joy not received for %f seconds, is the joy node drop?", WATCHDOG_PERIOD_);
    this->joy_alive_=false;
}
void ControlTeleop::BLwatchdog(const ros::TimerEvent &e)
{
    ROS_WARN("joy not received for %f seconds, is the joy node drop?", WATCHDOG_PERIOD_);
    this->joy_alive_=false;
}
void ControlTeleop::JoyCallback(const sensor_msgs::JoyConstPtr &msg)
{
    joy_watchdog_timer_.stop();
    joy_watchdog_timer_.start();
    boost::unique_lock<boost::shared_mutex> writLock(control_trigger_mutex_);
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
	else if(msg->axes[2]==-1&&msg->axes[6]==1)
	{
		control_trigger_=LinearBack;
	}
	else if(msg->axes[2]==-1&&msg->axes[6]==-1)
	{
		control_trigger_=LinearForward;
	}
	else if(msg->axes[2]==-1&&msg->axes[7]==1)
	{
		control_trigger_=LinearUp;
	}
	else if(msg->axes[2]==-1&&msg->axes[7]==-1)
	{
		control_trigger_=LinearDown;
	}
    else if(msg->axes[2]==-1&&msg->buttons[5])//
    {
        control_trigger_=ArmEmergencyChange;
    }
	else if(msg->axes[2]==-1&&msg->buttons[4])
	{
		control_trigger_=CutBack;
	}
    else if(msg->axes[2]==-1&&msg->axes[5]==-1)
    {
        control_trigger_=KnifeUnplug;
    }
    /*else if(msg->axes[2]==-1&&msg->axes[7]==1)//up
    {
        control_trigger_ = LightOn;
    }
    else if(msg->axes[2]==-1&&msg->axes[7]==-1)//down
    {
        control_trigger_ = LightOff;
    }*/
    else if(msg->axes[5]==-1&&msg->buttons[0]) //A
    {
        control_trigger_=KnifeOff;
    }
    else if(msg->axes[5]==-1&&msg->buttons[1]) //B
    {
        control_trigger_=ClockGo;
    }
    else if(msg->axes[5]==-1&&msg->buttons[2]) //X
    {
        control_trigger_=AntiClockGo;
    }
    else if(msg->axes[5]==-1&&msg->axes[6]==1) //left
    {
        control_trigger_=SingleAntiClockGo;
    }
    else if(msg->axes[5]==-1&&msg->axes[6]==-1) //right
    {
        control_trigger_=SingleClockGo;
    }
    else if(msg->axes[5]==-1&&msg->buttons[3]) //Y
    {
        control_trigger_=KnifeOn;
    }
    else if(msg->axes[5]==-1&&msg->buttons[10])
    {
        control_trigger_=ShutDown;
    }
	else if(msg->axes[5]==-1&&msg->buttons[4])
	{
		control_trigger_=SteeringIn;
	}
	else if(msg->axes[5]==-1&&msg->buttons[5])
	{
		control_trigger_=SteeringOut;
	}
    else if(msg->buttons[0])
    {
        control_trigger_=SaveCutPoint;
    }
    else if(msg->buttons[1])
    {
        control_trigger_=SaveChargePoint;
    }
    else if(msg->buttons[2])
    {
        control_trigger_=SaveNavPoint;
    }
    else if(msg->buttons[3])
    {
        control_trigger_=SaveTurnPoint;
    }
    else if(msg->buttons[4]) //L1
    {
        control_trigger_=NavOn;
    }
    else if(msg->buttons[5]) //R1
    {
        control_trigger_=NavPause;
    }
    else if(msg->buttons[6])
    {
        control_trigger_=MappingOff;
    }
    else if(msg->buttons[7])
    {
        control_trigger_=MappingOn;
    }
    else if(msg->buttons[9])
    {
        control_trigger_=RobotArmOn;
    }
    else if(msg->buttons[10])
    {
        control_trigger_=NavigationOn;
    }
    else
        control_trigger_ = Default;

    if(publish_vel_)
    {
        geometry_msgs::Twist vel{};

        if(msg->axes[0]||msg->axes[1]  )
        {
            vel.angular.z = msg->axes[0] * max_angular_velocity_;
            vel.linear.x = msg->axes[1] * max_linear_velocity_;
            if(msg->axes[3]||msg->axes[4])
            {
                vel.angular.z = msg->axes[0] * max_angular_velocity_ + msg->axes[3] * max_angular_velocity_ / 2.0;
                vel.linear.x = msg->axes[1] * max_linear_velocity_ + msg->axes[4] * max_linear_velocity_ / 2.0;
            }
        }
        else if(msg->axes[3]||msg->axes[4])
        {
            vel.angular.z = msg->axes[3] * max_angular_velocity_ / 2.0;
            vel.linear.x = msg->axes[4] * max_linear_velocity_ / 2.0;
            if(msg->axes[0]||msg->axes[1])
            {
                vel.angular.z = msg->axes[0] * max_angular_velocity_ + msg->axes[3] * max_angular_velocity_ / 2.0;
                vel.linear.x = msg->axes[1] * max_linear_velocity_ + msg->axes[4] * max_linear_velocity_ / 2.0;
            }
        }
        joy_vel_pub.publish(vel);
    }
}
void ControlTeleop::BLCallback(const std_msgs::StringConstPtr &msg)
{
    /*
     * Code Principle
     * first 3 character
     * ARM ~ robot arm
     * BAE ~ Base
     * Nav ~ Navigation
     * SYS ~ System
     * Then 2 characters
     * 01 ~ 99 peculiar movements;
     * Then 3 character
     * VEL ~ Velocity
     * Then 3 character
     * vx *100
     * Then 3 character
     * vz *100
     */
    /* Warning!!!!!!!!!!!!!!
     * strict code order
     * may change to scan mode;
     * */
    bl_watchdog_timer_.stop();
    bl_watchdog_timer_.start();
    boost::unique_lock<boost::shared_mutex> writLock(control_trigger_mutex_);
    if(msg->data.length()>=11)
    {
        std::string category = msg->data.substr(0,3);
        int MoveNumber = std::stoi(msg->data.substr(3,2));
        printf("already recieve msg\r\n");
        if(category == "ARM")
        {
            if (MoveNumber == 1)
                control_trigger_=ManipulateOn;
            else if(MoveNumber == 2)
                control_trigger_=ChargeOn;
            else if(MoveNumber == 3)
                control_trigger_=HomeOn;
            else if(MoveNumber == 4)
                control_trigger_=UpOn;
            else if(MoveNumber == 5)
                control_trigger_=LinearBack;
            else if(MoveNumber == 6)
                control_trigger_=LinearForward;
            else if(MoveNumber == 7)
                control_trigger_=LinearUp;
            else if(MoveNumber == 8)
                control_trigger_=LinearDown;
            else if(MoveNumber == 9)
                control_trigger_=ArmEmergencyChange;
            else if(MoveNumber == 10)
                control_trigger_=CutBack;
            else
                control_trigger_=Default;
        }
        else if(category == "BAE")
        {
            if (MoveNumber == 1)
                control_trigger_=KnifeUnplug;
            else if(MoveNumber == 2)
                control_trigger_=KnifeOff;
            else if(MoveNumber == 3)
                control_trigger_=ClockGo;
            else if(MoveNumber == 4)
                control_trigger_=AntiClockGo;
            else if(MoveNumber == 5)
                control_trigger_=SingleAntiClockGo;
            else if(MoveNumber == 6)
                control_trigger_=SingleClockGo;
            else if(MoveNumber == 7)
                control_trigger_=KnifeOn;
            else if(MoveNumber == 8)
                control_trigger_=SteeringIn;
            else if(MoveNumber == 9)
                control_trigger_=SteeringOut;
            else
                control_trigger_=Default;
        }
        else if(category == "NAV")
        {
            if (MoveNumber == 1)
                control_trigger_=SaveCutPoint;
            else if(MoveNumber == 2)
                control_trigger_=SaveChargePoint;
            else if(MoveNumber == 3)
                control_trigger_=SaveNavPoint;
            else if(MoveNumber == 4)
                control_trigger_=SaveTurnPoint;
            else if(MoveNumber == 5)
                control_trigger_=NavOn;
            else if(MoveNumber == 6)
                control_trigger_=NavPause;
            else
                control_trigger_=Default;
        }
        else if(category == "SYS")
        {
            printf("SYS\r\n");
            if (MoveNumber == 1)
                control_trigger_=ShutDown;
            else if(MoveNumber == 2)
            {
                printf("MappingOn\r\n");
                control_trigger_=MappingOn;
            }
            else if(MoveNumber == 3)
                control_trigger_=MappingOff;
            else if(MoveNumber == 4)
                control_trigger_=RobotArmOn;
            else if(MoveNumber == 5)
                control_trigger_=NavigationOn;
            else
                control_trigger_=Default;
        }
        else
            control_trigger_=Default;

        //when having joystick, do not publish bluetooth vel
        geometry_msgs::Twist vel{};
        if(msg->data.substr(5,3) == "VEL")
        {
            printf("vel\r\n");
            vel.angular.z = std::stoi(msg->data.substr(11,3))/100.0;
            vel.linear.x =  std::stoi(msg->data.substr(8,3))/100.0;
            std::cout<<"x "<<vel.linear.x<<" z"<<vel.angular.z<<std::endl;
            joy_vel_pub.publish(vel);
        }
    }

}