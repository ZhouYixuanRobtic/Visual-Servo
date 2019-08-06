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
sudo apt-get install ros-kinetic-moveit-visual-tools
sudo apt-get install ros-kinetic-industrial-core

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
 sudo apt-get update
 sudo apt-get upgrade
 sudo apt-get install build-essential cmake pkg-config
 sudo apt-get install libjpeg8-dev libtiff5-dev libjasper-dev libpng12-dev
 sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
 sudo apt-get install libxvidcore-dev libx264-dev
 sudo apt-get install libgtk-3-dev
 sudo apt-get install libatlas-base-dev gfortran
 sudo apt-get install python2.7-deb python3.5-dev
 wget https://github.com/Itseez/opencv/archive/3.4.5.zip
 unzip 3.4.5.zip
 sudo rm -rf 3.4.5.zip
 wget https://github.com/Itseez/opencv_contrib/archive/3.4.5.zip
 unzip 3.4.5.zip
 sudo rm -rf 3.4.5.zip
 wget https://bootstrap.pypa.io/get-pip.py
 sudo python get-pip.py
 sudo pip install virtualenv virtualenvwrapper
 sudo rm -rf ~/get-pip.py ~/.cache/pip/
 export WORKON_HOME=$HOME/.virtualenvs
 source /usr/local/bin/virtualenvwrapper.sh
 echo -e "\n# virtualenv and virtualenvwrapper" >> ~/.bashrc
 echo "export WORKON_HOME=$HOME/.virtualenvs" >> ~/.bashrc
 echo "source /usr/local/bin/virtualenvwrapper.sh" >> ~/.bashrc
 mkvirtualenv cv -p python2
 workon cv
 pip install numpy
 cd ~/opecv-3.4.5/
 mkdir build
 cd build 
 cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D ENABLE_CXX11=ON \
    -D INSTALL_C_EXAMPLES=OFF \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.4.5/modules \
    -D PYTHON_EXECUTABLE=~/.virtualenvs/cv/bin/python \
    -D BUILD_EXAMPLES=ON ..
 make -j8
 make clean
 make 
 sudo make install
 sudo ldconfig
 cd ~/.virtualenvs/cv/lib/python2.7/site-packages/
 ln -s /usr/local/lib/python2.7/site-packages/cv2/python-2.7/cv2.so cv2.so
 
 
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
cd ../..
sudo rm -rf Sophus
```

### Apriltag

```
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
cmake .
sudo make install
cd ..
sudo rm -rf apriltag
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
sudo apt install ros-kinetic-rgbd-launch
```

