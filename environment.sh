#!/bin/bash
echo "Activating ROS..."
#source /opt/ros/$ROSDISTRO/setup.bash
echo "...done."

echo "Setup ROS_HOSTNAME."
export ROS_HOSTNAME=$HOSTNAME.local
export SEGWAYRMP220_ROOT=$HOME/r220-nvision

echo "Building machines file..."
#make -C  $SEGWAYRMP220_ROOT
echo "...done"
echo "Activating development."
source $SEGWAYRMP220_ROOT/catkin_ws/devel/setup.bash

export ROSLAUNCH_SSH_UNKNOWN=1

# TODO: check that the time is >= 2015

# TODO: run a python script that checks all libraries are installed

exec "$@" #Passes arguments. Need this for ROS remote launching to work.


echo "done!!!"
