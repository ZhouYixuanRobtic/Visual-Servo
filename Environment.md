## Environment

#### ROS

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### MoveIt

```
rosdep update
sudo apt-get update
sudo apt-get dist-upgrade
sudo apt-get install ros-kinetic-catkin python-catkin-tools
sudo apt install ros-kinetic-moveit
sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin
```

### Gazebo

```
sudo apt-get remove gazebo7*
curl -sSL http://get.gazebosim.org | sh
sudo apt-get install ros-kinetic-gazebo9-ros-pkgs ros-kinetic-gazebo9-ros-control
sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
```

### OpenCV

```
 cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D ENABLE_CXX11=ON \
    -D INSTALL_C_EXAMPLES=OFF \
    -D OPENCV_EXTRA_MODULES_PATH=~/Cspace/opencv_contrib-3.4.5/modules \
    -D PYTHON_EXECUTABLE=~/.virtualenvs/cv/bin/python \
    -D BUILD_EXAMPLES=ON ..
    
```

### Sophus

```
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build
cd build
cmake ..
sudo make install
```

### Apriltag

```
git clone https://github.com/AprilRobotics/apriltag.git
cmake .
sudo make install
```

### Aubo_Robot

```
cd ${HOME}/WorkSpace/src
git clone https://github.com/ZhouYixuanRobtic/aubo_robot_realsense.git
cd ..
catkin_make
```

### RealSense

```
https://github.com/IntelRealSense/realsense-ros
```

