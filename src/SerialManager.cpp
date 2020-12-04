//
// Created by xcy on 2019/12/16.
//

#include "SerialManager.h"

SerialManager::SerialManager(std::string serial_addr, unsigned int baudrate): SERIAL_ADDR_(std::move(serial_addr)), BAUDRATE_(baudrate)
{

}
SerialManager::SerialManager(const SerialManager & serialManager): SERIAL_ADDR_(serialManager.SERIAL_ADDR_),BAUDRATE_(serialManager.BAUDRATE_)
{

}
SerialManager::~SerialManager()
{
    if(isOpen_)
        close(m_dFd);
}
bool SerialManager::openSerial()
{
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
    cfsetispeed(&m_stNew, BAUDRATE_);
    cfsetospeed(&m_stNew, BAUDRATE_);

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
    //serial_mutex_.lock();
    serial_alive_ = (read(m_dFd,read_buffer,BUFFER_SIZE)>=0);
    //serial_mutex_.unlock();
    return  isOpen_;
}
void SerialManager::receive()
{
    memset(read_buffer,0,BUFFER_SIZE);
    int receiveNumbers=read(m_dFd,&read_buffer,BUFFER_SIZE);
    if(receiveNumbers<0)
        serial_alive_ =false;
}
void SerialManager::send(const void * src, int size)
{
    if(isOpen_)
    {
        memset(write_buffer,0,BUFFER_SIZE);
        memcpy(write_buffer,src,size);
        serial_alive_ = (write(m_dFd,write_buffer,size)>=0);
    }
    else
        printf("the serial is not opened!!!!\r\n");
}
