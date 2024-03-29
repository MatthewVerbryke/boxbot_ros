// Dynamixel servo class header
//
// Copyright 2022-2023 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the commit history.
//
// TODO: needs to be tested and debugged

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
    std::string full_name;
    int id;
    int ticks;
    int neutral;
    float range;
    float rad_per_tick;
    bool check_inputs;
    
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
    
    // Private functions
    int angleToTicks(float angle_in);
    float ticksToAngle(int ticks_in);
    
public:
    
    // Constructor
    Dynamixel(std::string name_in, std::string side_in, ros::NodeHandle n, ros::NodeHandle nh, std::string robot_in);
    
    // Public functions
    int interpolate(float frame);
    void setCurrentFeedback(int reading);
    int setControlOutput(double goal);
    
    // Access functions
    std::string getName(){return name;};
    std::string getFullName(){return full_name;};
    int getID(){return id;};
    double getPosition(){return position;};
    double getVelocity(){return velocity;};
};

#endif
