#include "../include/visual_servo/parameterTeleop.h"
ParameterListener::ParameterListener(int rate, int num_per_thread)
{
    NUM_PER_THREAD_ = num_per_thread<=0 ? 8 :num_per_thread;
    RATE_ = rate <=0 ? 30 : rate;
}
ParameterListener::~ParameterListener()
{
    for(auto & thread_ptr : thread_ptrs_)
    {
        thread_ptr->interrupt();
        thread_ptr->join();
    }
}
void ParameterListener::registerParameterCallback(const std::vector <std::string> & parameterNames, bool isString)
{
    registered_threads_num_ = ceil(static_cast<float>(parameterNames.size())/NUM_PER_THREAD_);
    min_threads_num_ = floor(static_cast<float>(parameterNames.size())/NUM_PER_THREAD_);
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

    parameters_.resize(static_cast<int>(parameterNames.size()));
    stringParameters_.resize(static_cast<int>(parameterNames.size()));
    thread_ptrs_.resize(registered_threads_num_);
    for(int i=0;i<registered_threads_num_;++i)
    {
        thread_ptrs_[i]=shared_ptr<boost::thread>();
        thread_ptrs_[i].reset(new boost::thread(boost::bind(&ParameterListener::ParameterLoop,this,i,isString)));
    }
}
void ParameterListener::ParameterLoop(int thread_index,bool isString)
{
    try
    {
        boost::this_thread::interruption_enabled();
        while(true)
        {
            boost::this_thread::interruption_point();
            if(ros::ok())
            {
                if (isString)
                {
                    if(thread_index < min_threads_num_)
                    {
                        for (int i=0+thread_index*NUM_PER_THREAD_; i<NUM_PER_THREAD_*(thread_index+1); ++i)
                        {
                            ros::param::get(STRING_PARAMETER_NAMES[i],stringParameters_[i]);
                        }
                    }
                    else
                    {
                        for (int i=0+thread_index*NUM_PER_THREAD_; i<stringParameters_.size(); ++i)
                        {
                            ros::param::get(STRING_PARAMETER_NAMES[i],stringParameters_[i]);
                        }
                    }

                }
                else
                {

                    if(thread_index < min_threads_num_)
                    {
                        for (int i=0+thread_index*NUM_PER_THREAD_; i<NUM_PER_THREAD_*(thread_index+1); ++i)
                        {
                            ros::param::get(NUMBER_PARAMETER_NAMES[i],parameters_[i]);
                        }
                    }
                    else
                    {
                        for (int i=0+thread_index*NUM_PER_THREAD_; i<parameters_.size(); ++i)
                        {
                            ros::param::get(NUMBER_PARAMETER_NAMES[i],parameters_[i]);
                        }
                    }

                }

            }
            boost::this_thread::sleep(boost::posix_time::microseconds((int)1E6/RATE_));
        }
    }
    catch (boost::thread_interrupted&e )
    {
        //std::cout<<"close the "<<index<<"th thread and the parameter name is "<<parameterName<<std::endl;
    }
}
bool ParameterListener::getParameterValueViaName(const std::string & parameterName, std::string & value)
{
    for(int i=0;i<STRING_PARAMETER_NAMES.size();++i)
    {
        if(STRING_PARAMETER_NAMES[i] == parameterName)
        {
            value =  stringParameters_[i];
            return true;
        }
    }
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