//
// Created by xcy on 2019/12/9.
//

#ifndef VISUAL_SERVO_MANISERIALMANAGER_H
#define VISUAL_SERVO_MANISERIALMANAGER_H

#include "SerialManager.h"

#define RESULT_SIZE BUFFER_SIZE/8
class ManiSerialManager : public SerialManager{
public:
    struct ReadResult{
        char read_result[RESULT_SIZE];
        int read_bytes;
    };

private:
    std::queue<ReadResult> read_result_queue{};
    ReadResult read_results_;
    std::tr1::shared_ptr<boost::thread> thread_ptr_;
    std::mutex queue_mutex_;
    void readWorker(int rate);
    bool thread_registered_{};
public:
    ManiSerialManager(std::string serial_addr, unsigned int baudrate);
    explicit ManiSerialManager(const SerialManager & serialManager);
    ~ManiSerialManager() override;
    void registerAutoReadThread(int rate);
    void receive() override;
    ReadResult & getReadResult()
    {
        queue_mutex_.lock();
        if(!read_result_queue.empty())
        {
            read_results_ = read_result_queue.front();
            read_result_queue.pop();
        }
        else
            memset(&read_results_,0, sizeof(ReadResult));
        queue_mutex_.unlock();
        return read_results_;
    };
};


#endif //VISUAL_SERVO_MANISERIALMANAGER_H
