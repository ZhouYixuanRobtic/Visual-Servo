//
// Created by xcy on 2019/9/2.
//
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>


class ParameterListener
{
private:
    ros::NodeHandle nh_;
    ros::Rate * loopRate_;
    std::vector<boost::thread> threads_;
public:

    ParameterListener();
    virtual ~ParameterListener();
    void ParameterLoop(std::string parameterName,int index,bool isString);
    void registerParameterCallback(const std::vector<std::string> parameterNames,bool isString);
    std::vector<double> parameters;
    std::vector<std::string> stringParameters;

};
