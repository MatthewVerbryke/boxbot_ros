// Gazebo simulated-servo class header.
//
// Copyright 2018 University of Cincinnati
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
    std::string side;
    std::string robot;
    std::string commandTopic;
    std::string jointStateTopic;
    int index;
    
    // Variables
    double position;
    double desired;
    double velocity;
        
    // ROS publishers and subscribers
    ros::Publisher ControlPub;
    ros::Subscriber JointStateSub;
    
    // Joint state callback
    void jointStateCB(const sensor_msgs::JointState& msg);
    
public:

    // Constructor
    SimServo(std::string name, std::string side, ros::NodeHandle nh, std::string robot);
    
    // Joint command function
    void setCommandOutput();
    
    // Access functions
    std::string getName(){return name;};
    double getPosition(){return position;};
    double getVelocity(){return velocity;}; 
    double setDesired(double command){desired = command;};
};

#endif
