// Dynamixel servo class
//
// Copyright 2022 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the commit history.
//
// TODO: needs to be tested and debugged

#include <string>

#include "boxbot_driver/dynamixel.h"

// Constructor
Dynamixel::Dynamixel(std::string nameIn, std::string sideIn, ros::NodeHandle nh, std::string robotIn){
    
    // Setup the servo joint name
    robot = robotIn;
    side = sideIn;
    name = nameIn;
    std::string param_name = "joints/" + nameIn;
    
    // Set the check inputs flag
    check_inputs = false;
    
    // Get parameters from server
    nh.getParam(param_name + "/id", id);
    nh.param(param_name + "/ticks", ticks, 1024);
    nh.param(param_name + "/neutral", neutral, ticks/2);
    tolerance = 0.05;
    
    // Determine angular range
    if (ticks == 4096){
        range = 360.0;
    }
    else{
        range = 300.0;
    }
    
    // Get servo speed and range parameters
    nh.param(param_name + "/max_angle", max_angle, range/2);
    nh.param(param_name + "/min_angle", min_angle, -range/2);
    nh.getParam(param_name + "/max_speed", max_speed);
    
    // Check what info the node has recieved, if desired
    if (check_inputs == true){
        std::cout << name << " joint" << std::endl;
        std::cout << "id: " << id << std::endl;
        std::cout << "ticks: " << ticks << std::endl;
        std::cout << "neutral: " << neutral << std::endl;
        std::cout << "max angle: " << max_angle << std::endl;
        std::cout << "min angle: " << min_angle << std::endl;
        std::cout << "max speed: " << max_speed << std::endl;
        std::cout << std::endl;
    }
    
    // Determine radian to 'tick' conversion
    double pi = 3.14159265359;
    rad_per_tick = (pi*range/180.0) / ticks;
    
    // Intialize flags
    invert = false;
    readable = true;
    dirty = false;
    enabled = true;
    active = true;
    
    // Initialize variables
    status = "OK";
    position = 0.0;
    desired = 0.0;
    last_cmd = 0.0;
    velocity = 0.0;
    reads = 0;
    errors = 0;
    voltage = 0.0;
    temperature = 0.0;
    load = 0;
    last = ros::Time::now();
}

// Convert an angle (in radians) into a number of 'ticks'.
int Dynamixel::angleToTicks(float angleIn){
    
    int ticksInAngle;
    
    // Handle inverted control mode
    if (invert == true){
        ticksInAngle = neutral - (angleIn/rad_per_tick);
    }
    else{
        ticksInAngle = neutral + (angleIn/rad_per_tick);
    }
    
    // Keep the value with in limits
    if (ticksInAngle >= ticks){
        ticksInAngle--;
    }
    else {
        ticksInAngle = 0;
    }
    
    return ticksInAngle;
}

// Convert the number of 'ticks' into an angle (in radians).
float Dynamixel::ticksToAngle(int ticksIn){
    
    float angle;
    
    // Convert the value
    angle = (ticksIn - neutral) * rad_per_tick;
    
    // Handle inverted control mode
    if (invert == true){
        angle = -1*angle;
    }
    
    return angle;
}

// Determine the new position to move to using interpolation.
int Dynamixel::interpolate(float frame){
    
    // Cap movement
    if (std::abs(last_cmd - desired) < tolerance){
        dirty = false;
    }
    
    // Calculate the new position while respecting velocity limits
    float invert_frame = 1/frame;    
    float cmd = desired - last_cmd;
    float limit = max_speed / invert_frame;
    if (cmd > limit){
        cmd = limit;
    }
    else if (cmd < -limit){
        cmd = -limit;
    }
    
    // Get the new goal angle/tick value
    int ticksOut = angleToTicks(last_cmd + cmd);
    last_cmd = ticksToAngle(ticksOut);
    speed = cmd * invert_frame;
    
    return ticksOut;
}

// Update the angle using the current reading from the servo.
void Dynamixel::setCurrentFeedback(int reading){
    
    // Handle invalid reading values
    if (reading < -1){
        return;
    }
    else if (reading >= ticks){
        return;
    }
    
    // Update positions
    reads++;
    float last_angle = position;
    position = ticksToAngle(reading);
    
    // Update velocity estimate
    ros::Time t = ros::Time::now();
    ros::Duration delta_t = t - last;
    velocity = (position - last_angle) / (delta_t.toSec());
    last = t;
    
    // Loop last command if servo is inactive
    if (active == false){
        last_cmd = position;
    }
}

// Set the goal position the dyanmixel controller is moving to
int Dynamixel::setControlOutput(double goal){
    
    // Convert from rads to ticks
    int tick_goal = angleToTicks(goal);
    desired = tick_goal;
    
    return tick_goal;
}
