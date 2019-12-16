//
// Created by xcy on 2019/12/9.
//

#include "SerialManager.h"
#include "ros/ros.h"
SerialManager::SerialManager(std::string serial_addr, unsigned int baudrate):SERIAL_ADDR_(std::move(serial_addr)), BAUDRATE_(std::move(baudrate)) {
    struct termios m_stNew{};
    struct termios m_stOld{};
    isOpen_ = false;
    m_dFd = open(SERIAL_ADDR_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    isOpen_ = -1 != m_dFd;

    if ((fcntl(m_dFd, F_SETFL, FNDELAY)) < 0) {
        perror("Fcntl F_SETFL Error!\n");
        isOpen_ = false;
    }
    if (tcgetattr(m_dFd, &m_stOld) != 0) {
        perror("tcgetattr error!\n");
        isOpen_ = false;
    }
    m_stNew = m_stOld;
    cfmakeraw(&m_stNew);
    cfsetispeed(&m_stNew, baudrate);
    cfsetospeed(&m_stNew, baudrate);

    m_stNew.c_cflag |= (CLOCAL | CREAD);
    m_stNew.c_cflag &= ~CSIZE;
    m_stNew.c_cflag |= CS8;

    m_stNew.c_cflag &= ~PARENB;
    m_stNew.c_iflag &= ~INPCK;

    //set stopbits
    m_stNew.c_cflag &= ~CSTOPB;
    m_stNew.c_cc[VTIME] = 0;
    m_stNew.c_cc[VMIN] = 1;

    tcflush(m_dFd, TCIFLUSH);
    if (tcsetattr(m_dFd, TCSANOW, &m_stNew) != 0) {
        perror("tcsetattr Error!\n");
        isOpen_ = false;
    }
    serial_mutex_.lock();
    serial_alive_ = (read(m_dFd,read_buffer,BUFFER_SIZE)>=0);
    serial_mutex_.unlock();
}
SerialManager::~SerialManager()
{
    close(m_dFd);
    thread_ptr_->interrupt();
    thread_ptr_->join();
}
void SerialManager::registerAutoReadThread(int rate)
{
    thread_ptr_.reset(new boost::thread(boost::bind(&SerialManager::readWorker,this,rate)));
}
void SerialManager::readWorker(int rate)
{
    static ros::Rate loop_rate(rate);
    try
    {
        boost::this_thread::interruption_enabled();
        while(true)
        {
            //boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::universal_time();
            boost::this_thread::interruption_point();
            receive();
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
void SerialManager::receive()
{
    serial_mutex_.lock();
    int receiveNumbers=read(m_dFd,&read_buffer[read_used_bytes],BUFFER_SIZE);
    serial_mutex_.unlock();
    if(receiveNumbers>0)
    {
        serial_alive_ =true;
        read_used_bytes +=receiveNumbers;
        if(read_used_bytes%COMMAND_SIZE==0)
        {
            ReadResult temp{};
            for(int i=0;i<read_used_bytes;i+=COMMAND_SIZE)
            {
                //WARNING, MAY IGNORE SOME DATA
                if(i<=RESULT_SIZE-COMMAND_SIZE)
                {
                    memcpy(&temp.read_result[i], &read_buffer[i], COMMAND_SIZE);
                    temp.read_bytes += COMMAND_SIZE;
                }
            }
            read_result_queue.push(temp);
            memset(read_buffer,0,BUFFER_SIZE);
            read_used_bytes = 0;
        }
        if(read_used_bytes>=BUFFER_SIZE-COMMAND_SIZE)
        {
            read_used_bytes=0;
            memset(read_buffer,0,BUFFER_SIZE);
        }
    }
    else if(receiveNumbers<0)
    {
        serial_alive_ =false;
        read_used_bytes= 0;
        memset(read_buffer,0,BUFFER_SIZE);
    }

}
void SerialManager::send(const void *src,int size)
{
    if(isOpen_)
    {
        send_mutex_.lock();
        memset(write_buffer,0,BUFFER_SIZE);
        memcpy(write_buffer,src,size);
        send_mutex_.unlock();
        serial_mutex_.lock();
        serial_alive_ = (write(m_dFd,write_buffer,size)>=0);
        serial_mutex_.unlock();
    }
    else
        printf("the serial is not opened!!!!\r\n");
}