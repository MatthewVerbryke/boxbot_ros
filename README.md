# Boxbot

## Summary

This repository contains the ROS packages needed for the simulation of "Boxbot", a simple dual-armed robot consisting of two [WidowX Robot Arms (Mark II)](http://www.trossenrobotics.com/widowxrobotarm) attached to a simple "torso". Included is a URDF model of the robot, along with corresponding ros-control and Gazebo interfaces.

Note: We are currently trying to solve the shoulder joint overload problem (see Notes). Additionally, we are working on a modified 6-DOF WidowX arm, which can be used in Gazebo and RViz instead of the original 5-DOF arm.

## Recommended OS/Programs

This software was developed and tested in:
- Ubuntu 18.04 LTS
- ROS Melodic
- Gazebo 9.0.0

## Installation 

To install the current version of the repository to your machine, change directory into the desired catkin workspace source directory and clone the repository.

## Usage

### RViz

To launch a model of the robot in RViz from the top level directory use:

```
roslaunch boxbot_description boxbot_rviz.launch five_dof:=BOOLEAN
```

where the ```BOOLEAN``` argument can be set as ```true``` to spawn the 5-DOF model or ```false``` to use the 6-DOF model. If the argument is left out, the default model is set as the 6-DOF model.

### Gazebo

To launch the robot model in Gazebo with ROS controllers top level directory use:

```
roslaunch boxbot_gazebo boxbot_gazebo.launch five_dof:=BOOLEAN
```

where the ```BOOLEAN``` argument can be set as ```true``` to spawn the 5-DOF model or ```false``` to use the 6-DOF model. If the argument is left out, the default model is set as the 6-DOF model.

### Model Editing

If you want to edit the URDF model, only modify the files that end with ```.urdf.xacro``` or ```.gazebo```, found in the ```boxbot_description/robots``` and ```boxbot_description/urdf``` folders. Generally, you will not want to edit the ```.urdf``` files directly, as they are auto-generated and are also harder to look through than the subcomponent ```.xacro``` files.

After you are done editing, you can regenerate the complete URDF model by switching into the ```boxbot_description``` package directory, and running the autogeneration script:

```
./regen_urdf.sh
```

### Notes

- During initial testing of the physical Boxbot, we found that the first shoulder joint of the WidowX arm (joint_1) has problems holding up the weight of the arm, sporadically shutting down due to what appears to be overload (either torque or power). In response, we are currently redesigning the shoulder joint to utilize a larger and more capable servo.

## License:

All new packages/files use the BSD 3-clause license presented in the main license file, and are Copyright (C) University of Cincinnati.

Exceptions include the whole of `boxbot_description/meshes` and some of the contents of `boxbot_description/urdf`, which are based on files from [Interbotix's widowx_arm repository](https://github.com/Interbotix/widowx_arm). Copyright for these are held by others, as reflected in the commit history for the original files. All modifications are Copyright (C) University of Cincinnati
