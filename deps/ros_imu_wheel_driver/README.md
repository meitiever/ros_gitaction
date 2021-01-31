# Sineva Camera Imu Synchronization #

* This is the visual SLAM Synchronization for sineva clean machine robot.

-------

### Features ###
* imu camera sync triggered by mivii S2 Pro

-------

### Requirements ###
### Install framework dependencies ###
* sudo apt install autotools-dev ccache doxygen git liblapack-dev libblas-dev libgtest-dev libreadline-dev libssh2-1-dev pylint clang-format-3.8 python-autopep8 python-catkin-tools python-pip python-git python-setuptools python-termcolor python-wstool libatlas3-base --yes

* find usb port name in /dev. - /dev/ttyUSB0
* enable open and read privilege of the port specified above. - sudo chmod 777 /dev/ttyUSB4 /dev/miivii_sync_in_a  
* fill in configuration into the launch file - sync_start.launch
	* portname / bandwidth(460800) / frequency(100) / topic and tolerance(10 ms)
* install driver of CH341 -  sudo insmod ch341.ko

### Config workapace ###
* create workspace
* cd workspace
* git clone XXXX
* copy dependencies to local folder, to help you compile faster. 
* export ROS_VERSION=kinetic #(Ubuntu 16.04: kinetic, Ubuntu 18.04: melodic, make sure type the right version)
* export CATKIN_WS=~/sync_ws
* mkdir -p $CATKIN_WS/src
* cd $CATKIN_WS
* catkin init
* catkin config --merge-devel # Necessary for catkin_tools >= 0.4.
* catkin config --extend /opt/ros/$ROS_VERSION
* catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

### Copy dependencies to local folder ###
* sudo mkdir -p /opt/sineva/sineva_zips
* sudo cp -r dependencies/opt/sineva/sineva_zips/* /opt/sineva/sineva_zips
* verify zip files copied manually.

### build dependencies ###
* catkin build gflags_catkin
* catkin build glog_catkin

### build program ###
* catkin build ros_imu_cam_sync 

-------

### Example ###
* roslaunch ros_imu_cam_sync sync_start.launch

### Tests ###
* OS: 16.04 / 18.04
* Platform: x86, arm 
