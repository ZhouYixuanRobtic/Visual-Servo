#include <iostream>

#include <stdio.h>
#include <unistd.h>


#include "example_3.h"



int main()
{
    //案例0:使用SDK构建一个最简单的机械臂的控制工程
    //Example_0::demo();

    //案例1:回调函数的方式获取实时路点，末端速度，机械臂的事件  关于机械臂相关状态的获取
//    Example_1::demo();
//    Example_1::getJointStatus();

    //案例3:关节运动
    Example_3::demo();

    //案例4:直线运动
//   Example_4::demo_relativeOri();

    //案例5:轨迹运动
//    Example_5::demo();

    //案例6:运动至目标位置
//    Example_6::demo();

    //案例8:正逆解
//    Example_8::demo();

    //关于io的使用案例
//    Example_9::demo();


    return 0;
}

