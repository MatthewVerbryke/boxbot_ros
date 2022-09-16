#!/bin/bash
# Copyright 2018-2022 University of Cincinnati
# All rights reserved. See LICENSE file at:
# https://github.com/MatthewVerbryke/boxbot_ros
# Additional copyright may be held by others, as reflected in the commit history.

# Get static paths
RELATIVE_PATH="`dirname \"$0\"`"
ABSOLUTE_PATH="`( cd \"$RELATIVE_PATH\" && pwd )`"

#Delete old URDF files
cd $ABSOLUTE_PATH/robots/
rm boxbot_6dof.urdf
rm boxbot_5dof.urdf

#Create new URDF files
xacro boxbot_6dof.urdf.xacro > boxbot_6dof.urdf
xacro boxbot_5dof.urdf.xacro > boxbot_5dof.urdf
