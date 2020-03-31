
source /opt/ros/$ROS_DISTRO/setup.bash

# If no catkin environment is setup, create it
if [  ! -d ./devel ]; then
   echo "catkin environment does not exist; creating it. This runs the local build once as well."
   catkin_make
fi

source ./devel/setup.bash


