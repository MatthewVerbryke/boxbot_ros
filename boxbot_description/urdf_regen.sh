#!/bin/bash
# Copyright 2017 University of Cincinnati
# All rights reserved. See LICENSE file at:
# TODO
# Additional copyright may be held by others, as reflected in the commit history.


# Source setup
cd ~/rse_ws/
source devel/setup.bash

#Get path
PACKAGE_PATH="/home/$USER/rse_ws/src/boxbot"

#Delete old URDF file
cd $PACKAGE_PATH/boxbot_description/robots/
rm boxbot.urdf

#Create new URDF file
rosrun xacro xacro.py -o boxbot.urdf boxbot.urdf.xacro
