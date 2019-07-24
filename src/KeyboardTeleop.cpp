#include "KeyboardTeleop.h"

KeyboardTeleop::KeyboardTeleop()
{
    this->teachModeOn=false;
    kfd=0;
}
KeyboardTeleop::~KeyboardTeleop()
{

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
      puts("Use D_CAP key to enable teach mode");  
     
      struct pollfd ufd;  
      ufd.fd = kfd;  
      ufd.events = POLLIN;  
      
      for(;;)  

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
                    if(teachModeOn)
                    {
                        teachModeOn=false;
                        std::cout<<"!!!!!teach mode stop!!!!!!!!!!!"<<std::endl;
                    }
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
                std::cout<<"under teach mode"<<std::endl;
                teachModeOn=true; 
                dirty = true;
                break; 
            default:  
                teachModeOn=false; 
                dirty = false;  
                break;
       }  
    }  
}
