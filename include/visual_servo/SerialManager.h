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
#include <mutex>
#include <queue>

using std::tr1::shared_ptr;
#define COMMAND_SIZE 8

#define BUFFER_SIZE COMMAND_SIZE*30
#define RESULT_SIZE COMMAND_SIZE*10
class SerialManager {
    typedef int file_descriptor_t;
public:
    struct ReadResult{
        char read_result[RESULT_SIZE];
        int read_bytes;
    };

private:
    const std::string SERIAL_ADDR_;
    std::mutex send_mutex_;
    std::mutex serial_mutex_;

    file_descriptor_t m_dFd;

    const unsigned int BAUDRATE_;
    char read_buffer[BUFFER_SIZE];
    char write_buffer[BUFFER_SIZE];

    std::queue<ReadResult> read_result_queue{};
    ReadResult read_results_;

    int read_used_bytes{};
    bool isOpen_{};
    bool serial_alive_{};

    shared_ptr<boost::thread> thread_ptr_;
    void readWorker(int rate);
public:
    SerialManager(std::string serial_addr, unsigned int baudrate);
    ~SerialManager();
    bool isOpen(){ return isOpen_;};
    bool isSerialAlive(){return serial_alive_;};
    void registerAutoReadThread(int rate);
    void send(const void* src,int size);
    void receive();
    ReadResult & getReadResult()
    {
        if(!read_result_queue.empty())
        {
            read_results_ = read_result_queue.front();
            read_result_queue.pop();
        }
        else
            memset(&read_results_,0, sizeof(ReadResult));
        return read_results_;
    };
};


#endif //VISUAL_SERVO_SERIALMANAGER_H
