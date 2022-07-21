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
    std::string command_topic;
    int id;
    bool check_inputs;
    
    // Variables
    double position;
    double desired;
    double velocity;
        
    // ROS publishers and subscribers
    ros::Publisher ControlPub;
    
public:

    // Constructor
    SimServo(std::string name, std::string side, ros::NodeHandle nh, std::string robot);
    
    // Joint command function
    void setCommandOutput();
    
    // Access functions
    std::string getName(){return name;};
    int getID(){return id;};
    double getPosition(){return position;};
    double getVelocity(){return velocity;};
    void setPosition(double pose_in){position = pose_in;};
    void setVelocity(double vel_in){velocity = vel_in;};
    void setDesired(double command){desired = command;};
    void setID(int id_in){id = id_in;};
};

#endif
