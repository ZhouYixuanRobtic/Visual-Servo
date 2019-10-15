cd ~
#moveit
rosdep update
sudo apt-get update
sudo apt-get --yes dist-upgrade
sudo apt-get --yes install ros-kinetic-catkin python-catkin-tools
sudo apt --yes install ros-kinetic-moveit
sudo apt-get --yes install ros-kinetic-trac-ik-kinematics-plugin
sudo apt-get --yes install ros-kinetic-moveit-visual-tools
sudo apt-get --yes install ros-kinetic-industrial-core
sudo apt-get update
#opencv
sudo apt-get --yes upgrade
sudo apt-get --yes install build-essential cmake pkg-config
sudo apt-get --yes install libjpeg8-dev libtiff5-dev libjasper-dev libpng12-dev
sudo apt-get --yes install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get --yes install libxvidcore-dev libx264-dev
sudo apt-get --yes install libgtk-3-dev
sudo apt-get --yes install libatlas-base-dev gfortran
sudo apt-get --yes install python2.7-deb python3.5-dev
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
cd ~/opencv-3.4.5/
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
cd ~
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build
cd build
cmake ..
sudo make install
cd ../..
sudo rm -rf Sophus
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
cmake .
sudo make install
cd ..
sudo rm -rf apriltag
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main"
sudo apt-get update -qq
sudo apt-get --yes install librealsense2-dkms --allow-unauthenticated -y
sudo apt-get --yes install librealsense2-utils --allow-unauthenticated -y
sudo apt-get --yes install librealsense2-dev --allow-unauthenticated -y
sudo apt-get --yes install librealsense2-dbg --allow-unauthenticated -y
sudo apt-get --yes install ros-kinetic-diagnostic-updater -y
sudo apt-get --yes install ros-kinetic-ddynamic-reconfigure -y
sudo apt-get update
sudo apt-get --yes upgrade
sudo apt-get --yes install ros-kinetic-rgbd-launch
mkdir -p ~/catkin_realsense_ws/src
cd ~/catkin_realsense_ws/src/
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
cd ..
catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source ~/catkin_realsense_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
