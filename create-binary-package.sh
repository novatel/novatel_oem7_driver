#!/bin/sh

UBUNTU_TGT_DISTRO=$1
ROS_TGT_DISTRO=$2

rosdep init
rosdep update

rm *.deb
rm src/*.deb
./create-deb.sh $UBUNTU_TGT_DISTRO $ROS_TGT_DISTRO src/novatel_oem7_msgs
./create-deb.sh $UBUNTU_TGT_DISTRO $ROS_TGT_DISTRO src/novatel_oem7_driver
mv src/*.deb .



