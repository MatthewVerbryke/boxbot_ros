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
    robot = nameIn;
    side = sideIn;
    name = "~joints/" + nameIn;
    
    // Get parameters from server
    id = nh.param(name + "/id");
    ticks = nh.param(name + "/ticks", 1024);
    neutral = nh.param(name + "/neutral", self.ticks/2);
    max_angle = nh.param(name + "/max_angle");
    min_angle = nh.param(name + "/min_angle");
    max_speed = nh.param(name + "/max_speed");
    tolerance = 0.05;
    
    // Determine angular range
    if (ticks == 4096){
        range = 360.0;
    }
    else{
        range = 300.0;
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
    temperature 0.0;
    load = 0;
    last = ros::Time::now()
}

// Convert an angle (in radians) into a number of 'ticks'.
int angleToTicks(float angleIn){
    
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
    else (ticksInAngle < 0){
        ticksInAngle = 0;
    }
    
    return ticksInAngle;
}

// Convert the number of 'ticks' into an angle (in radians).
float ticksToAngle(int ticksIn){
    
    float angle;
    
    // Convert the value
    angle = (ticksIn - neutral) * rad_per_tick;
    
    // Handle inverted control mode
    if (invert == true){
        angle = -1*angle;
    }
    
    return angle
}

// Determine the new position to move to using interpolation.
int interpolate(float frame){
    
    // Cap movement
    if (std::abs(last_cmd - desired) < tolerance){
        dirty = false;
    }
    
    // Calculate the new position while respecting velocity limits
    float cmd = desired - last_cmd;
    float limit = max_speed / frame;
    if (cmd > limit){
        cmd = limit;
    }
    else if (cmd < -limit){
        cmd = -limit;
    }
    
    // Get the new goal angle/tick value
    int ticksOut = angleToTicks(last_cmd + cmd);
    last_cmd = ticksToAngle(ticksOut);
    speed = cmd * frame;
    
    return ticksOut
}

// Update the angle using the current reading from the servo.
void setCurrentFeedback(int reading){
    
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
    float delta_t = t - last;
    velocity = (position - last_angle) / (delta_t::sec + delta_t::nsec*0.00000001);
    last = t;
    
    // Loop last command if servo is inactive
    if (active == false){
        last_cmd = postion;
    }
}
