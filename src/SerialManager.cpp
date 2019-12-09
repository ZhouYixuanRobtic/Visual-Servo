//
// Created by xcy on 2019/12/9.
//

#include "SerialManager.h"

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
    if(isOpen_)
        registerReadThread(60);
    else
        printf("the serial is not opened!!!!\r\n");
}
SerialManager::~SerialManager()
{
    thread_ptr_->interrupt();
    thread_ptr_->join();
}
void SerialManager::registerReadThread(int rate)
{
    thread_ptr_.reset(new boost::thread(boost::bind(&SerialManager::readWorker,this,rate)));
}
void SerialManager::readWorker(int rate)
{
    try
    {
        boost::this_thread::interruption_enabled();
        while(true)
        {
            boost::this_thread::interruption_point();
            boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();
            receive();
            boost::posix_time::ptime endTime = boost::posix_time::microsec_clock::local_time();
            boost::this_thread::sleep(boost::posix_time::microseconds((int)1E6/rate - (endTime - startTime).total_microseconds()));
        }
    }
    catch (boost::thread_interrupted&e )
    {
        std::cout<<"now quit the read thread"<<std::endl;
    }
}
void SerialManager::receive()
{
    receiveNumbers=read(m_dFd,read_buffer,BUFFER_SIZE);
    if(receiveNumbers>0)
    {
        memcpy(read_result, read_buffer, receiveNumbers * sizeof(char));
        printf("back: %s\r\n",read_result);
    }
    else
        memset(read_result,0,BUFFER_SIZE);
}
void SerialManager::send(const char *src,int size)
{
    if(isOpen_)
    {
        memset(write_buffer,0,BUFFER_SIZE);
        memcpy(write_buffer,src,size);
        write(m_dFd,write_buffer,size);
        printf("send: %s\r\n",write_buffer);
    }
    else
        printf("the serial is not opened!!!!\r\n");
}