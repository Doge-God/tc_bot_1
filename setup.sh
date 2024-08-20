#!/bin/bash

## MODIFIED
####################################################
### COMPSYS 732: ROS setup script                ###
### Author: Mahla Nejati                         ###
####################################################

SETUPFILE="/opt/ros/noetic/setup.bash"

notice() {
    printf "\033[1;35m$*\033[0m\n"
}

notice "*************************Installing ROS Noetic*************************"

notice "*******Setup your sources.list******************"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

notice "*******Install curl, gedit and git******************"
sudo apt install -y curl gedit git

notice "*******Setup your keys******************"
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

notice "*******Upadate and install ROS noetic******************"
sudo apt update
sudo apt install -y ros-noetic-desktop-full


notice "*************************Modifying ~/.bashrc file*************************"
if [ -e ~/.bashrc ]; then
    grep -q -E "^source\s+$SETUPFILE\s*$" ~/.bashrc || echo "source $SETUPFILE" >> ~/.bashrc
else
    echo "source $SETUPFILE" >> ~/.bashrc

notice "*************************Setup dependecies for building packages*************************"
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install -y python3-rosdep

notice "*****Initialize rosdep*****"
sudo rosdep init
rosdep update

notice "*************************Installing noetic Kobuki*************************"
sudo ln -s /usr/bin/python3 /usr/bin/python
sudo apt-get update
sudo apt-get install -y ros-noetic-kobuki-ftdi

make deps
make -f Makefile

notice "***********Installing libuvc**********"
sudo apt-get install -y ros-noetic-libuvc-camera ros-noetic-libuvc-ros
notice "***********Compiling camera packages**********"
catkin_make --pkg astra_camera
catkin_make --pkg astra_launch
echo export TURTLEBOT_BASE=kobuki >> ~/.bashrc
echo export TURTLEBOT_3D_SENSOR=astra >> ~/.bashrc
source ~/.bashrc

notice ".. DONE .."
