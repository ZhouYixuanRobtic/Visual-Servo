//
// Created by xcy on 2019/12/16.
//

#ifndef VISUAL_SERVO_SERIALMANAGER_H
#define VISUAL_SERVO_SERIALMANAGER_H

#include <termios.h>
#include <fcntl.h>
#include <boost/thread/thread.hpp>
#include "tr1/memory"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstdlib>
#include <utility>
#include <mutex>
#include <queue>


#define BUFFER_SIZE 400

class SerialManager {
    typedef int file_descriptor_t;
private:
    const unsigned int BAUDRATE_;
    const std::string SERIAL_ADDR_;

protected:
    std::mutex send_mutex_;
    //std::mutex serial_mutex_;
    file_descriptor_t m_dFd;
    bool serial_alive_{};
    bool isOpen_{};
    char read_buffer[BUFFER_SIZE]{};
    char write_buffer[BUFFER_SIZE]{};

public:
    SerialManager(std::string serial_addr, unsigned int baudrate);
    SerialManager(const SerialManager & serialManager);
    virtual ~SerialManager();
    unsigned int getBaudrate(){return BAUDRATE_;};
    std::string getSerialAddr(){return SERIAL_ADDR_;};
    bool isOpen(){ return isOpen_;};
    bool isSerialAlive(){return serial_alive_;};
    bool openSerial();
    void send(const void* src,int size);
    virtual void receive();
};


#endif //VISUAL_SERVO_SERIALMANAGER_H
