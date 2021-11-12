source ~/set_pepper_ip.sh

export ROS_MASTER_URI=http://${PEPPER_IP}:11311
export ROS_HOSTNAME=${PEPPER_IP}

source /tmp/gentoo/opt/ros/kinetic/setup.bash
source ~/ros_ws/toolkit_ws/devel/setup.bash

