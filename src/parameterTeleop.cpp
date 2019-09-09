#include "../include/visual_servo/parameterTeleop.h"
ParameterListener::ParameterListener()
{
    loopRate_ = new ros::Rate(30);
}
ParameterListener::~ParameterListener()
{
    delete loopRate_;
    for(int i=0;i<threads_.size();++i)
    {
        threads_[i].interrupt();
        threads_[i].join();
    }
}
void ParameterListener::registerParameterCallback(const std::vector <std::string> parameterNames, bool isString)
{
    if(isString)
    {
        for(auto & parameterName : parameterNames)
        {
            STRING_PARAMETER_NAMES.push_back(parameterName);
        }
    }
    else
    {
        for(auto & parameterName : parameterNames)
        {
            NUMBER_PARAMETER_NAMES.push_back(parameterName);
        }
    }

    for(int i=0;i<parameterNames.size();++i)
    {
        parameters_.push_back(0.0);
        stringParameters_.push_back("a");
        threads_.push_back(boost::thread(boost::bind(&ParameterListener::ParameterLoop,this,parameterNames[i],i,isString)));
    }
}
void ParameterListener::ParameterLoop(std::string parameterName,int index,bool isString)
{
    while(ros::ok())
    {
        boost::this_thread::interruption_point();
        if(isString)
            ros::param::get(parameterName,stringParameters_[index]);
        else
            ros::param::get(parameterName,parameters_[index]);
        loopRate_->sleep();
    }
}

bool ParameterListener::getParameterValueViaName(const std::string parameterName,double &value)
{
    double temp_value = value;
    for(int i=0; i<NUMBER_PARAMETER_NAMES.size(); )
    {
        if(NUMBER_PARAMETER_NAMES[i] == parameterName)
        {
            value=parameters_[i];
            return true;
        }
    }
    value=temp_value;
    return false;
}
bool ParameterListener::getParameterValueViaName(const std::string parameterName,std::string &value)
{
    std::string temp_value=value;
    for(int i=0; i<STRING_PARAMETER_NAMES.size(); )
    {
        if(STRING_PARAMETER_NAMES[i] == parameterName)
        {
            value=stringParameters_[i];
            return true;
        }
    }
    value=temp_value;
    return false;

}
bool ParameterListener::getParameterValueViaName(const std::string parameterName,int &value)
{
    int temp_value=value;
    for(int i=0; i<NUMBER_PARAMETER_NAMES.size(); )
    {
        if(NUMBER_PARAMETER_NAMES[i] == parameterName)
        {
            value=(int)parameters_[i];
            return true;
        }
    }
    value=temp_value;
    return false;

}
bool ParameterListener::getParameterValueViaName(const std::string parameterName,bool &value)
{
    bool temp_value=value;
    for(int i=0; i<NUMBER_PARAMETER_NAMES.size(); )
    {
        if(NUMBER_PARAMETER_NAMES[i] == parameterName)
        {
            value=(bool)parameters_[i];
            return true;
        }
    }
    value=temp_value;
    return false;

}
double ParameterListener::getNumberParameterValueViaIndex(int index)
{
    if(index>=0&&index<parameters_.size())
        return parameters_[index];
    else
    {
        ROS_ERROR("WRONG INDEX");
        return 0.0;
    }

}
std::string ParameterListener::getStringParameterValueViaIndex(int index)
{
    if(index>=0&&index<stringParameters_.size())
        return stringParameters_[index];
    else
    {
        ROS_ERROR("WRONG INDEX");
        return "\0";
    }
}
