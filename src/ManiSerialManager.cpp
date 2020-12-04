//
// Created by xcy on 2019/12/9.
//
#include "SerialManager.h"
#include "ManiSerialManager.h"

#include <utility>
#include "ros/ros.h"
ManiSerialManager::ManiSerialManager(std::string serial_addr, unsigned int baudrate): SerialManager(std::move(serial_addr), baudrate)
{

}
ManiSerialManager::ManiSerialManager(const SerialManager & serialManager): SerialManager(serialManager)
{

}
ManiSerialManager::~ManiSerialManager()
{
    if(thread_registered_)
    {
        thread_ptr_->interrupt();
        thread_ptr_->join();
    }
}
void ManiSerialManager::registerAutoReadThread(int rate)
{
    thread_ptr_.reset(new boost::thread(boost::bind(&ManiSerialManager::readWorker, this, rate)));
    thread_registered_=true;
}
void ManiSerialManager::readWorker(int rate)
{
    static ros::Rate loop_rate(rate);
    try
    {
        boost::this_thread::interruption_enabled();
        while(true)
        {
            //boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::universal_time();
            boost::this_thread::interruption_point();
            this->receive();
            //boost::posix_time::ptime endTime = boost::posix_time::microsec_clock::universal_time();
            //boost::this_thread::sleep(boost::posix_time::microseconds((int)1E6/rate - (endTime - startTime).total_microseconds()));
            loop_rate.sleep();
        }
    }
    catch (boost::thread_interrupted&e )
    {
        std::cout<<"now quit the read thread"<<std::endl;
    }
}
void ManiSerialManager::receive()
{
    boost::unique_lock<boost::shared_mutex> writLock(queue_mutex_);
    int receiveNumbers=read(m_dFd,&read_buffer,BUFFER_SIZE);
    if(receiveNumbers>0)
    {
        serial_alive_ =true;
        ReadResult temp{};
        memcpy(&temp.read_result, &read_buffer, RESULT_SIZE);
        temp.read_bytes=receiveNumbers;
        queue_mutex_.lock();
        read_result_queue.push(temp);
        queue_mutex_.unlock();
        memset(read_buffer,0,BUFFER_SIZE);
    }
    else if(receiveNumbers<0)
    {
        serial_alive_ =false;
        memset(read_buffer,0,BUFFER_SIZE);
    }
}