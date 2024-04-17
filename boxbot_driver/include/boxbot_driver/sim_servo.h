// Gazebo simulated-servo class header.
//
// Copyright 2022-2023 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the
// commit history.

#ifndef SIMSERVO_H
#define SIMSERVO_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

class SimServo
{
private:
    
    // Parameters
    std::string name;
    std::string full_name;
    std::string side;
    std::string robot;
    std::string command_topic;
    std::string mimic_topic;
    int id;
    bool is_gripper;
    
    // Variables
    double position;
    double desired;
    double velocity;
    double opening;
    
public:

    // Constructor
    SimServo(std::string name, std::string side, ros::NodeHandle n, ros::NodeHandle nh, std::string robot);
    
    // Joint command function
    void setCommandOutput();
    void setPosition(double pose_in);
    
    // ROS publishers and subscribers
    ros::Publisher ControlPub;
    ros::Publisher MimicPub;
    
    // Access functions
    std::string getName(){return name;};
    std::string getFullName(){return full_name;};
    int getID(){return id;};
    double getPosition(){return position;};
    double getVelocity(){return velocity;};
    void setVelocity(double vel_in){velocity = vel_in;};
    void setDesired(double command){desired = command;};
    void setID(int id_in){id = id_in;};
};

#endif
