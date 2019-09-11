# Visual-Servo

​	This is a visual servo application using MoveIt and AprilTag

## Install

- **Dependencies**

​	The main  application depends on [OpenCv 3.4.5](https://opencv.org), [MoveIt](http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html) (Prebuilt Binaries), [ros-kinetic](http://wiki.ros.org/Documentation), [AprilTag](https://april.eecs.umich.edu/software/apriltag), [Sophus](https://github.com/strasdat/Sophus) [CV_bridge](http://wiki.ros.org/cv_bridge), [aubo_robot](<https://github.com/ZhouYixuanRobtic/aubo_robot_realsense>) and [realsense](<https://github.com/IntelRealSense/realsense-ros>) . For simulation you should install [Gazebo](http://gazebosim.org/) too.

​	see [Environment.md](https://github.com/ZhouYixuanRobtic/Visual-Servo/blob/master/Environment.md) for details install commands. 

- **Packages**

  Under your workspace source folder, use commands below, then use `catkin_make`

  ```
  git clone https://github.com/ZhouYixuanRobtic/aubo_robot_realsense.git
  git clone https://github.com/ZhouYixuanRobtic/Visual-Servo.git
  ```

## Usage

- **Simulation**

  ```
  roslaunch visual_servo visual_servo_sim.launch 
  ```

  ​	After the simulation nodes launched clearly, then use

  ```
  rosrun visual_servo visual_servo 
  rosrun visual_servo maTest
  ```

- **real robot**

  ​	Check the robot's IP, if it is not `192.168.2.13` then modify the `visual_servo_real.launch` file. Make sure all sensors are ready and the **EMERGENCY STOP** is as near as you can reach. Then run  

  ```
  roslaunch visual_servo visual_servo_real.launch 
  ```

  ​	After it's launched clearly, run 

  ```
  rosrun visual_servo maTest
  ```