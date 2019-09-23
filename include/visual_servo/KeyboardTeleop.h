#ifndef _KEYBOARDTELEOP_H_
#define _KEYBOARDTELEOP_H_

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>

class KeyboardTeleop
{
private:
    int kfd;

    struct termios cooked, raw;

    bool done;
    boost::thread thread_;

public:
    bool moveChange;
    bool navOn=false;
    bool pause=false;
    bool maOn=false;
    bool chargeOn=false;
    bool goUpOn = false;
    bool goHomeOn = false;
    KeyboardTeleop();
    virtual ~KeyboardTeleop();
    void keyboardLoop();
};


#endif