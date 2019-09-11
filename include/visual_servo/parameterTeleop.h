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
    void ParameterLoop(const std::string & parameterName,int index,bool isString);
    void registerParameterCallback(const std::vector<std::string> & parameterNames,bool isString);
    template<typename ANY_TYPE>
    bool getParameterValueViaName(const std::string & parameterName, ANY_TYPE & value);
    bool getParameterValueViaName(const std::string & parameterName, std::string & value);

    double getNumberParameterValueViaIndex(int index);
    std::string getStringParameterValueViaIndex(int index);


};
template<typename ANY_TYPE>
bool ParameterListener::getParameterValueViaName(const std::string & parameterName, ANY_TYPE & value)
{
    for(int i=0;i<NUMBER_PARAMETER_NAMES.size();++i)
    {
        if(NUMBER_PARAMETER_NAMES[i] == parameterName)
        {
            value = (ANY_TYPE) parameters_[i];
            return true;
        }
    }
    return false;
}