# Boxbot v2 (WIP)

## Summary

Note: Currently designing a 6-DOF version of the WidowX arm, which will also solve the shoulder overload problem (see Notes). Gazebo and ros-control functionality is currently broken.

This repository contains the ROS packages needed for the simulation of "Boxbot", a simple dual-armed robot consisting of two [WidowX robotic arms](http://www.trossenrobotics.com/widowxrobotarm) attached to a simple "torso". Included is a URDF model of the robot, along with corresponding ros-control and Gazebo interfaces.

## Recommended OS/Programs

This software was developed and tested in:
- Ubuntu 18.04 LTS
- ROS Melodic
- Gazebo 9.0.0

## Installation 

To install the current version of the repository to your machine, change directory into the desired catkin workspace source directory and clone the repository.

## Usage

### RViz

To launch a model of the original robot in RViz from the top level directory, use:

```
roslaunch boxbot_description rviz.launch
```

For the new preliminary design with 6-DOF arms, use:

```
roslaunch boxbot_description rviz_2.launch
```

Currently, you will manually have to add the robot model to the RViz window.

### Gazebo

To launch an empty world with only the Boxbot robot in it, use:

```
roslaunch boxbot_gazebo empty_world.launch
```

Some other scenarios are included that were used to test dual-arm manipulation in simulation.

### Model Editing

If you want to edit the URDF model, only modify the files that end with ```.urdf.xacro``` or ```.gazebo```, found in the ```boxbot_description/robots``` and ```boxbot_description/urdf``` folders. Generally, you will not want to edit the ```.urdf``` files directly, as they are auto-generated and are also harder to sift through than the subcomponent files.

After you are done editing, you can regenerate the complete urdf model by switching into the ```boxbot_description``` package directory, and running the autogeneration script:

```
./urdf_regen.sh
```

### Notes

- During initial testing of the physical Boxbot system, it was found that the first shoulder joint of the WidowX arm (joint_1) has problems holding up the weight of the arm, sporadically shutting down due to what appears to be overload. In response, the shoulder joint is being redesigned to utilize a larger and more capable servo.

## License:

All new packages/files use the BSD 3-clause license presented in the main license file, and are Copyright (C) University of Cincinnati.

Exceptions include the whole of `boxbot_description/meshes` and some of the contents of `boxbot_description/urdf`, which are based on files from [Interbotix's widowx_arm repository](https://github.com/Interbotix/widowx_arm). Copyright for these are held by others, as reflected in the commit history for the original files.  All modifications are Copyright (C) University of Cincinnati
