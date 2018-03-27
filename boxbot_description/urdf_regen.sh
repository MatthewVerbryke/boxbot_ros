#!/bin/bash
# Copyright 2018 University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/MatthewVerbryke/boxbot_ros
# Additional copyright may be held by others, as reflected in the commit history.

# Get static paths
RELATIVE_PATH="`dirname \"$0\"`"
ABSOLUTE_PATH="`( cd \"$RELATIVE_PATH\" && pwd )`"

#Delete old URDF file
cd $ABSOLUTE_PATH/robots/
rm boxbot.urdf

#Create new URDF file
rosrun xacro xacro.py -o boxbot.urdf boxbot.urdf.xacro
