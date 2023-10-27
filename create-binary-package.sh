#!/bin/sh

UBUNTU_TGT_DISTRO=$1
ROS_TGT_DISTRO=$2

echo build | sudo -S rosdep init # this usually fails with harmless error
rosdep update --include-eol-distros

rm *.deb
rm src/*.deb
./create-deb.sh $UBUNTU_TGT_DISTRO $ROS_TGT_DISTRO src/novatel_oem7_msgs

# reinstall the msgs to satisfy the dependencies for the driver.
cd src
echo build | sudo -S apt remove  -y ros-$ROS_TGT_DISTRO-novatel-oem7-msgs
echo build | sudo -S apt install -y ./*.deb
cd ..

./create-deb.sh $UBUNTU_TGT_DISTRO $ROS_TGT_DISTRO src/novatel_oem7_driver
mv src/*.deb .



