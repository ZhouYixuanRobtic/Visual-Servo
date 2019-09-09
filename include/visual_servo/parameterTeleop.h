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
    std::vector<std::string> NUMBER_PARAMETER_NAMES;
    std::vector<std::string> STRING_PARAMETER_NAMES;
    std::vector<double> parameters_;
    std::vector<std::string> stringParameters_;
public:
    const std::vector<double>& parameters() const {return parameters_;}
    const std::vector<std::string>& stringParameters() const {return stringParameters_;};
    ParameterListener();
    virtual ~ParameterListener();
    void ParameterLoop(std::string parameterName,int index,bool isString);
    void registerParameterCallback(const std::vector<std::string> parameterNames,bool isString);
    bool getParameterValueViaName(const std::string parameterName,double &value);
    bool getParameterValueViaName(const std::string parameterName,std::string &value);
    bool getParameterValueViaName(const std::string parameterName,int &value);
    bool getParameterValueViaName(const std::string parameterName,bool &value);
    double getNumberParameterValueViaIndex(int index);
    std::string getStringParameterValueViaIndex(int index);


};
