//
// Created by xcy on 2020/9/28.
//

#include "visual_servo/BLManager.h"
BLManager::BLManager()
{
    buffer_=new char[_BUFFER_SIZE_];
    opt=sizeof(receiveAddr_);
    if(this->initialize())
    {
        this->waitForConnection();
    }
}
BLManager::~BLManager()
{
    close(newFd_);
    delete [] buffer_;
}
bool BLManager::initialize()
{
    /* 如果调用 socket() 出错，则退出 */
    if((serverFd_ = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM)) == -1)
    {
        /* 输出错误提示并退出 */
        printf(" socket fail !\r\n");
        return false;
    }
    printf("socket ok !\r\n");

    /* 主机字节顺序 */
    serverAddr_.rc_family = AF_BLUETOOTH;
    /* 将运行程序机器的IP填充入RC_bdaddr */
    bacpy(&serverAddr_.rc_bdaddr, &_BDADDR_ANY);
    serverAddr_.rc_channel = (uint8_t) 1;

    if(bind(serverFd_, (struct sockaddr*) &serverAddr_, sizeof(struct sockaddr)) == -1)
    {
        /* 如果调用bind()失败，则给出错误提示，退出 */
        printf(" bind fail!\r\n ");
        return false;
    }
    printf("bind ok !\r\n");
    /* 这里是我们一直强调的错误检查！！ */
    if(listen(serverFd_, 1) == -1)
    {
        /* 如果调用 listen 失败，则给出错误提示，退出 */
        printf("listen fail!\r\n");
        return false;
    }
    printf("listen ok!\r\n");
    FD_ZERO(&server_set);
    FD_ZERO(&serverNew_set);
    FD_SET(serverFd_, &serverNew_set);
    tempFd_=serverFd_;
    return true;
}
bool BLManager::waitForConnection()
{
    printf("Waiting for new client connect\r\n");
    server_set=serverNew_set;
    int nready = select(tempFd_ + 1, &server_set, nullptr, nullptr, nullptr);
    if(nready<=0)
    {
        isConnectionOk=false;
        return false;
    }
    else if(FD_ISSET(serverFd_,&server_set))
    {
        newFd_ = accept(serverFd_, (struct sockaddr*) &receiveAddr_, &opt);
        /* 若accept返回为-1，则连接错误，重新等待连接 */
        if(newFd_ == -1)
        {
            printf(" server accept fail!!!\r\n ");
            isConnectionOk=false;
        }
        else
        {
            FD_SET(newFd_, &serverNew_set);
            if(newFd_ > tempFd_)
                tempFd_=newFd_;
            char ip[1024] = {0};
            ba2str(&receiveAddr_.rc_bdaddr,ip);
            printf("Client@%s connected\n", ip);
            isConnectionOk=true;
            fcntl(newFd_, F_SETFL, O_NONBLOCK);
            close(serverFd_);
        }
        return  isConnectionOk;
    }
}
bool BLManager::write(const void *data, int dataSize)
{
    if(isConnectionOk)
    {
        if(send(newFd_, data, dataSize, 0) == -1)
        {
            isConnectionOk=false;
            close(newFd_);
            printf("write fail!\r\n");
        }
    }
    return isConnectionOk;
}
std::string BLManager::read()
{
    std::string content{};
    if(FD_ISSET(newFd_,&serverNew_set))
    {
        if(-1==(receivedBytes_ = recv(newFd_,buffer_,_BUFFER_SIZE_,0)))
        {
            if(!(errno == EINTR || errno == EWOULDBLOCK || errno == EAGAIN))
            {
                isConnectionOk=false;
                FD_CLR(newFd_, &serverNew_set);
                close(newFd_);
                printf("read fail!!!\r\n");
                return content;
            }
        }
        else
        {
            if(receivedBytes_ == 0)
            {
                static bool first_time=true;
                if(first_time)
                {
                    printf("received empty\r\n");
                    first_time=false;
                }
                return content;
            }
            else
            {
                /*
                 * Preserved for further process
                 *
                 */
                content = buffer_;
                printf("receive: %s\r\n",buffer_);
                this->write(response_letter.c_str(),response_letter.size());
            }
        }
        return content;
    }
}
