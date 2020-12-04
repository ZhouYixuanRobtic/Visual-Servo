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
#include <atomic>
#define BUFFER_SIZE 400

class SerialManager {
    typedef int file_descriptor_t;
private:
    const unsigned int BAUDRATE_;
    const std::string SERIAL_ADDR_;

protected:
    file_descriptor_t m_dFd{};
    std::atomic_bool serial_alive_{};
    std::atomic_bool isOpen_{};
    std::atomic_char read_buffer[BUFFER_SIZE]{};
    std::atomic_char write_buffer[BUFFER_SIZE]{};

public:
    explicit SerialManager(std::string serial_addr, unsigned int baudrate);
    SerialManager(const SerialManager & serialManager);

    virtual ~SerialManager();
    unsigned int getBaudrate() const {return BAUDRATE_;};
    std::string getSerialAddr() const{return SERIAL_ADDR_;};
    bool isOpen() const{ return isOpen_;};
    bool isSerialAlive() const {return serial_alive_;};
    bool openSerial();
    virtual void send(const void* src,int size);
    virtual void receive();
};


#endif //VISUAL_SERVO_SERIALMANAGER_H
