#include "KeyboardTeleop.h"

KeyboardTeleop::KeyboardTeleop()
{
    this->moveChange=false;
    kfd=0;
    thread_=boost::thread(boost::bind(&KeyboardTeleop::keyboardLoop,this));

}
KeyboardTeleop::~KeyboardTeleop()
{
    tcsetattr(kfd, TCSANOW, &cooked);
    thread_.interrupt();
    thread_.join();
}
void KeyboardTeleop::keyboardLoop()
{
      char c;  
      bool dirty = false;  
      
      tcgetattr(kfd, &cooked);  
      memcpy(&raw, &cooked, sizeof(struct termios));  
      raw.c_lflag &=~ (ICANON | ECHO);  
      raw.c_cc[VEOL] = 1;  
      raw.c_cc[VEOF] = 2;  
      tcsetattr(kfd, TCSANOW, &raw);  

      puts("Reading from keyboard");  
      puts("Use D key to stop robot");
      puts("Use q key to enable navigation mode");
      puts("Use p key to pause the robot");
      puts("Use c key to continue the robot");
      puts("Use m key to enable cut mode");
      puts("Use n key to enable charging mode");
      struct pollfd ufd;
      ufd.fd = kfd;  
      ufd.events = POLLIN;
      while(true)

      {  

        boost::this_thread::interruption_point();
        // get the next event from the keyboard  
        int num;  
        if ((num = poll(&ufd, 1, 250)) < 0)  
        {  
            perror("poll():");  
            return;  
        }
        else if(num > 0)  
        {  
            if(read(kfd, &c, 1) < 0)  
            {
                perror("read():");  
                return;  
            }  
        }  
        else  
        {  
            if (dirty == true)  
            {  
                    /*
                        stop do something when key up
                    */
                    moveChange=!moveChange;
                    dirty = false;  
            }  
            continue;  
        }
          switch(c)
          {
              case 'w':
                  /*
                  * preserve some space for further requests
                  */
                  break;
              case 'D':
                  dirty = true;
                  break;
              case 'q':
                  navOn=true;
                  dirty=true;
                  std::cout<<"navon"<<std::endl;
                  break;
              case 'p':
                  dirty=true;
                  pause=true;
                  std::cout<<"pause"<<std::endl;
                  break;
              case 'c':
                  dirty=true;
                  pause=false;
                  std::cout<<"go on"<<std::endl;
                  break;
              case 'm':
                  dirty=true;
                  maOn=true;
                  std::cout<<"ma on"<<std::endl;
                  break;
              case 'n':
                  dirty=true;
                  chargeOn=true;
                  std::cout<<"charge on"<<std::endl;
              default:
                  dirty = false;
                  break;
          }
        usleep(100000);
      }
}
