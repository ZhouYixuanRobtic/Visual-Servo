//
// Created by xcy on 2019/9/2.
//
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include "tr1/memory"
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include "atomic"
using std::tr1::shared_ptr;
class ParameterListener
{
private:
    int NUM_PER_THREAD_;
    int RATE_;
    std::vector< shared_ptr<boost::thread> > thread_ptrs_;
    std::vector<std::string> NUMBER_PARAMETER_NAMES;
    std::vector<std::string> STRING_PARAMETER_NAMES;
    std::vector<double> parameters_{};
    std::vector<std::string> stringParameters_{};
    int registered_threads_num_{};
    int min_threads_num_{};
    boost::shared_mutex data_mutex_{};
public:
    const std::vector<double>& parameters()
    {
        boost::shared_lock<boost::shared_mutex> readLock(data_mutex_);
        return parameters_;
    }
    const std::vector<std::string>& stringParameters()
    {
        boost::shared_lock<boost::shared_mutex> readLock(data_mutex_);
        return stringParameters_;
    };
    const std::vector<std::string>& parameterNames() const {return NUMBER_PARAMETER_NAMES;}
    const std::vector<std::string>& stringParameterNames() const {return STRING_PARAMETER_NAMES;};
    ParameterListener(int rate, int num_per_thread);
    virtual ~ParameterListener();
    void ParameterLoop(int thread_index,bool isString);
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
    boost::shared_lock<boost::shared_mutex> readLock(data_mutex_);
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