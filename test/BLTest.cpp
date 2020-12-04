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


#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/forwards.h>
#include <ros/single_subscriber_publisher.h>
#include "ros/callback_queue.h"
#include <ros/package.h>

#include "visual_servo/manipulate.h"
#include "VisualServoMetaType.h"
#include "ControlTeleop.h"
#include "queue"
#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> ( const YAML::Node& node, T& i )
{
    i = node.as<T>();
}
#endif

#define _BUFFER_SIZE_ 1024


using std::tr1::shared_ptr;
static constexpr bdaddr_t _BDADDR_ANY = {0, 0, 0, 0, 0, 0};

using namespace CONTROLTELEOP;
class Listener
{
    typedef void(*sighandler_t)(int);
private:
    ros::NodeHandle nh;
    ros::Subscriber tag_info_sub;
    int tagID_;
    void tagInfoCallback(const visual_servo::TagsDetection_msg &msg);
public:
    explicit Listener(int tag_id);
    virtual ~Listener();
    visual_servo_namespace::ServiceCaller* serviceCaller;
    static int pox_system(const char* commands);
    double getParameter();
};
Listener::Listener(int tag_id):tagID_(tag_id)
{
    serviceCaller = new visual_servo_namespace::ServiceCaller();
    tag_info_sub=nh.subscribe("TagsDetected",10, &Listener::tagInfoCallback,this);
}
Listener::~Listener()
{
    delete serviceCaller;
}
void Listener::tagInfoCallback(const visual_servo::TagsDetection_msg &msg)
{
    if(!msg.tags_information.empty())
    {
        for(auto & tag_information : msg.tags_information)
        {
            tagID_=tag_information.id;
        }
    }

}
int Listener::pox_system(const char *commands)
{
    int ret=0;
    sighandler_t old_handler;
    old_handler = signal(SIGCHLD,SIG_DFL);
    ret = system(commands);
    signal(SIGCHLD,old_handler);
    return ret;
}
double Listener::getParameter()
{
    double result=190.0;
    YAML::Node doc = YAML::LoadFile(ros::package::getPath("visual_servo")+"/config/tagParam.yaml");
    try
    {
        result = doc["ID"+std::to_string(tagID_)]["steeringDistance"].as<double>();
    }
    catch (YAML::InvalidScalar)
    {
        ROS_ERROR("tagParam.yaml is invalid.");
    }
    ROS_INFO_STREAM("the tag id is "<<tagID_<<" the steeringDistance is "<<result);
    return result;
}



class BLSocketServer{
private:

    /* 在sockFd上进行监听，newFd 接受新的连接 */
    int serverFd_{}, newFd_{},tempFd_{};
    /* 自己的地址信息 */
    struct sockaddr_rc serverAddr_{0},receiveAddr_{0};

    fd_set  server_set{},serverNew_set{};
    int sinSize_{};
    socklen_t opt{};
    /* 从buffer中读取的位数 */
    int receivedBytes_{};
    /* buffer */
    char * buffer_;
    char * write_buffer_;
    boost::mutex read_mutex_{},write_mutex_{};


    bool isReadRegistered{};
    shared_ptr<boost::thread> ServerReadThread_;

public:
    bool isConnectionOk{};
    explicit BLSocketServer();
    ~BLSocketServer();
    bool initialize();
    bool waitForConnection();
    bool write(const void *data,int dataSize);
    bool read();
    void serverReadThread(int rate);
    void registerServerReadThread(int rate);

    void closeServer();
};


BLSocketServer::BLSocketServer()
{
    buffer_=new char[_BUFFER_SIZE_];
    write_buffer_=new char[_BUFFER_SIZE_];
    memset(write_buffer_,0,_BUFFER_SIZE_);
    opt=sizeof(receiveAddr_);
    if(this->initialize())
    {
        this->waitForConnection();
    }


}
BLSocketServer::~BLSocketServer()
{
    if(isReadRegistered)
    {
        ServerReadThread_->interrupt();
        ServerReadThread_->join();
    }
    close(newFd_);
    delete [] buffer_;
    delete[] write_buffer_;
}
bool BLSocketServer::initialize()
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
bool BLSocketServer::waitForConnection()
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
bool BLSocketServer::write(const void *data, int dataSize)
{
    if(isConnectionOk)
    {
        if(send(newFd_, data, dataSize, 0) == -1)
        {
            isConnectionOk=false;
            close(newFd_);
            printf("write fail!\r\n");
            if(isReadRegistered)
            {
                ServerReadThread_->interrupt();
                isReadRegistered=false;
            }

        }
    }
    return isConnectionOk;
}
bool BLSocketServer::read()
{
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
                return false;
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
                return false;
            }
            else
            {
                /*
                 * Preserved for further process
                 *
                 */
                printf("receive: %s\r\n",buffer_);
            }
        }
        return true;
    }

}

void BLSocketServer::closeServer()
{
    close(newFd_);
}
void BLSocketServer::serverReadThread(int rate)
{
    boost::this_thread::interruption_enabled();
    try
    {
        while(!isConnectionOk)
            boost::this_thread::sleep(boost::posix_time::microseconds((int)1E6/rate));
        while (isConnectionOk)
        {
            boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();
            boost::this_thread::interruption_point();
            read_mutex_.lock();
            read();
            read_mutex_.unlock();
            boost::posix_time::ptime endTime = boost::posix_time::microsec_clock::local_time();
            boost::this_thread::sleep(boost::posix_time::microseconds((int)1E6/rate - (endTime - startTime).total_microseconds()));
        }
    }
    catch (boost::thread_interrupted&e )
    {
        std::cout<<"client thread interrupted!"<<std::endl;
    }
}
void BLSocketServer::registerServerReadThread(int rate)
{
    ServerReadThread_.reset(new boost::thread(boost::bind(&BLSocketServer::serverReadThread,this,rate)));
    isReadRegistered=true;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"BLServer");
    ros::NodeHandle nh_("~");
    Listener listener(400);
    double max_linear_velocity,max_angular_velocity;
    nh_.param("max_linear_velocity",max_linear_velocity,(double)0.8);
    nh_.param("max_angular_velocity",max_angular_velocity,(double)0.5);
    CONTROLTELEOP::ControlTeleop controlTeleop(true,max_linear_velocity,max_angular_velocity);
    /* launch base_only
    ros::V_string AllNodes;
    ros::master::getNodes(AllNodes);
    for(auto & node_name:AllNodes)
    {
        if(node_name=="/base_only")
        {
            listener.pox_system(("rosnode kill " + node_name).c_str());
            sleep(10);
        }
    }
    listener.pox_system("gnome-terminal -x bash -c \"roslaunch rubber_navigation baseOnly.launch publish_robot_source_odom_tf:=true\";exit;exec bash");
    */
    BLSocketServer blSocketServer;
    blSocketServer.registerServerReadThread(10);
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        switch(controlTeleop.getControlTrigger())
        {
            case ManipulateOn:
                if(!listener.serviceCaller->srvCalling())
                    listener.serviceCaller->callSrv(visual_servo::manipulate::Request::CUT);
                break;
            case ChargeOn:
                if(!listener.serviceCaller->srvCalling())
                    listener.serviceCaller->callSrv(visual_servo::manipulate::Request::CHARGE);
                break;
            case UpOn:
                if(!listener.serviceCaller->srvCalling())
                    listener.serviceCaller->callSrv(visual_servo::manipulate::Request::UP);
                break;
            case HomeOn:
                if(!listener.serviceCaller->srvCalling())
                    listener.serviceCaller->callSrv(visual_servo::manipulate::Request::HOME);
                break;
            case CutBack:
                if(!listener.serviceCaller->srvCalling())
                    listener.serviceCaller->callSrv(visual_servo::manipulate::Request::CUTBACK);
                break;
            case LinearUp:
                if(!listener.serviceCaller->srvCalling())
                    listener.serviceCaller->callSrv(visual_servo::manipulate::Request::LINEAR,Eigen::Vector3d(0.0,0.0,0.0015));
                break;
            case LinearDown:
                if(!listener.serviceCaller->srvCalling())
                    listener.serviceCaller->callSrv(visual_servo::manipulate::Request::LINEAR,Eigen::Vector3d(0.0,0.0,-0.0015));
                break;
            case LinearForward:
                if(!listener.serviceCaller->srvCalling())
                    listener.serviceCaller->callSrv(visual_servo::manipulate::Request::LINEAR,Eigen::Vector3d(0.0,-0.002,0.0));
                break;
            case LinearBack:
                if(!listener.serviceCaller->srvCalling())
                    listener.serviceCaller->callSrv(visual_servo::manipulate::Request::LINEAR,Eigen::Vector3d(0.0,0.002,0.0));
                break;
            case ClockGo:
                ros::param::set("/visual_servo/clockGo",1.0);
                break;
            case AntiClockGo:
                ros::param::set("/visual_servo/antiClockGo",1.0);
                break;
            case SingleAntiClockGo:
                ros::param::set("/visual_servo/singleAntiClockGo",1.0);
                break;
            case SingleClockGo:
                ros::param::set("/visual_servo/singleClockGo",1.0);
                break;
            case KnifeOn:
                ros::param::set("/visual_servo/knifeOn",1.0);
                break;
            case KnifeOff:
                ros::param::set("/visual_servo/knifeOff",1.0);
                break;
            case KnifeUnplug:
                ros::param::set("/visual_servo/knifeUnplug",1.0);
                break;
            case SteeringIn:
                ros::param::set("/visual_servo/steeringIn",listener.getParameter());
                break;
            case SteeringOut:
                ros::param::set("visual_servo/steeringOut",1.0);
                break;
            case LightOn:
                ros::param::set("/visual_servo/lightOn",1.0);
                break;
            case LightOff:
                ros::param::set("/visual_servo/lightOff",1.0);
                break;
            case RobotArmOn:
            {
                ros::V_string AllNodes;
                ros::master::getNodes(AllNodes);
                for(auto & node_name:AllNodes)
                {
                    if(node_name=="/aubo_gazebo_driver"||node_name=="/rviz")
                    {
                        listener.pox_system(("rosnode kill " + node_name).c_str());
                        sleep(20);
                    }
                }
                listener.pox_system("gnome-terminal -x bash -c \"roslaunch visual_servo visual_servo_real.launch\";exit;exec bash;");
                break;
            }
            case NavigationOn:
            {
                ros::V_string AllNodes;
                ros::master::getNodes(AllNodes);
                for(auto & node_name:AllNodes)
                {
                    if(node_name=="/rubber_navigation"||node_name=="/auboRobot"||node_name=="/base_only")
                    {
                        listener.pox_system(("rosnode kill " + node_name).c_str());
                    }
                }
                sleep(10);
                listener.pox_system("gnome-terminal -x bash -c \"roslaunch rubber_navigation rubber_navigation.launch\";exit;exec bash");
                break;
            }
            case ShutDown:
            {
                ros::V_string AllNodes;
                ros::master::getNodes(AllNodes);
                for(auto & node_name:AllNodes)
                {
                    if(node_name!="/joystick"&&node_name!="/maTest"&&node_name!="/rosout")
                    {
                        listener.pox_system(("rosnode kill " + node_name).c_str());
                    }
                }
                sleep(1);
                listener.pox_system("gnome-terminal -x bash -c \"shutdown\"; exec bash;");
            }
            case MappingOn:
            {
                ros::V_string AllNodes;
                ros::master::getNodes(AllNodes);
                for(auto & node_name:AllNodes)
                {
                    if(node_name=="/rviz")
                    {
                        listener.pox_system(("rosnode kill " + node_name).c_str());
                        sleep(5);
                        break;
                    }
                }
                listener.pox_system("gnome-terminal -x bash -c \"roslaunch cartographer_ros demo_velodyne2d.launch\";exit;exec bash;");
                break;
            }
            case MappingOff:
            {
                ros::V_string AllNodes;
                ros::master::getNodes(AllNodes);
                for(auto & node_name:AllNodes)
                {
                    if(node_name=="/rviz")
                    {
                        listener.pox_system("rosservice call /finish_trajectory 0");
                        sleep(1);
                        std::string temp{ros::package::getPath("rubber_navigation")+"/config/map/map.bag.pbstream"};
                        std::string final_temp="gnome-terminal -x bash -c \"rosservice call /write_state \"filename: \'"+temp+"\' include_unfinished_submaps: \'true\'\"\";exit;exec bash;";
                        listener.pox_system(final_temp.c_str());
                        sleep(2);
                        break;
                    }
                }
                break;
            }
            default:
                break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}