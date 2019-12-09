//
// Created by xcy on 2019/12/9.
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

using std::tr1::shared_ptr;
#define BUFFER_SIZE 120
class SerialManager {
typedef int file_descriptor_t;
private:
    const std::string SERIAL_ADDR_;
    file_descriptor_t m_dFd;
    const unsigned int BAUDRATE_;
    char read_buffer[BUFFER_SIZE];
    char read_result[BUFFER_SIZE];
    char write_buffer[BUFFER_SIZE];
    bool isOpen_{};
    int receiveNumbers;
    shared_ptr<boost::thread> thread_ptr_;
    void registerReadThread(int rate);
    void readWorker(int rate);
public:
    SerialManager(std::string serial_addr, unsigned int baudrate);
    ~SerialManager();
    void send(const char* src,int size);
    void receive();
    const char* getReadResult(){return read_result;};
};


#endif //VISUAL_SERVO_SERIALMANAGER_H
