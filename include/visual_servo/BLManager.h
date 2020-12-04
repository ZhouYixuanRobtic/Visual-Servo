//
// Created by xcy on 2020/9/28.
//

#ifndef VISUAL_SERVO_BLMANAGER_H
#define VISUAL_SERVO_BLMANAGER_H

#include <stdio.h>
#include <unistd.h>
#include <cstring>
#include <csignal>
#include <iostream>

#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <fcntl.h>
#include <cerrno>
#include <sys/types.h>
#include <netinet/in.h>

#include <boost/thread/thread.hpp>
#include "tr1/memory"
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include "queue"

#define _BUFFER_SIZE_ 1024

using std::tr1::shared_ptr;
static constexpr bdaddr_t _BDADDR_ANY = {0, 0, 0, 0, 0, 0};
const std::string response_letter{"success"};
class BLManager {
private:
    /* 在sockFd上进行监听，newFd 接受新的连接 */
    int serverFd_{}, newFd_{},tempFd_{};
    /* 自己的地址信息 */
    struct sockaddr_rc serverAddr_{0},receiveAddr_{0};

    fd_set  server_set{},serverNew_set{};
    socklen_t opt{};

    /* 从buffer中读取的位数 */
    int receivedBytes_{};
    /* buffer */
    char * buffer_;

public:
    bool isConnectionOk{};
    explicit BLManager();
    ~BLManager();
    bool initialize();
    bool waitForConnection();
    bool write(const void *data,int dataSize);
    std::string read();
};



#endif //VISUAL_SERVO_BLMANAGER_H
