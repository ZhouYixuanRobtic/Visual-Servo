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

    for(int i=0;i<parameterNames.size();++i)
    {
        parameters.push_back(0.0);
        stringParameters.push_back("a");
        threads_.push_back(boost::thread(boost::bind(&ParameterListener::ParameterLoop,this,parameterNames[i],i,isString)));
    }
}
void ParameterListener::ParameterLoop(std::string parameterName,int index,bool isString)
{
    while(ros::ok())
    {
        boost::this_thread::interruption_point();
        if(isString)
            ros::param::get(parameterName,stringParameters[index]);
        else
            ros::param::get(parameterName,parameters[index]);
        loopRate_->sleep();
    }
}
