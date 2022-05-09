// Dynamixel Servo Class header
//
// Copyright 2022 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the
// commit history.

#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <cmath>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

class Dynamixel
{
private:

    // General parameters
    std::string robot;
    std::string side;
    std::string name;
    std::string commandTopic;
    int id;
    int ticks;
    int neutral;
    float range;
    float rad_per_tick;
    
    // Limits
    float max_angle;
    float min_angle;
    float max_speed;
    float tolerance;
    
    // Flags
    bool invert;
    bool readable;
    bool dirty;
    bool enabled;
    bool active;
    
    // Variables
    std::string status;
    double position;
    double desired;
    double last_cmd;
    double speed;
    double velocity;
    int reads;
    int errors;
    double voltage;
    double temperature;
    int load;
    ros::Time last;
    
    // ROS publishers and subscribers
    //ros::Subscriber CommandSub;
    
    // Callback functions
    //void commandCB(std_msgs::Float64 msg)
        
    // Private functions
    int angleToTicks(float angleIn)
    float ticksToAngle(int ticksIn)
    
public:
    
    // Constructor
    Dynamixel::Dynamixel(std::string nameIn, std::string sideIn, ros::NodeHandle nh, std::string robotIn)
    
    // Public functions
    int interpolate(float frame)
    void setCurrentFeedback(int reading)
};

#endif
