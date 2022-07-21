// Gazebo simulated-servo class.
//
// Copyright 2022 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the
// commit history.

#include <string>

#include "boxbot_driver/sim_servo.h"

SimServo::SimServo(std::string name_in, std::string side_in, ros::NodeHandle nh, std::string robot_in){
    
    // Setup names
    side = side_in;
    robot = robot_in;
    name = name_in;
    std::string param_name = "joints/" + name_in;
    
    // Set the check inputs flag
    check_inputs = false;
    
    // Get parameters from server
    nh.getParam(param_name + "/id", id);
    
    //
    command_topic = robot + "/" + side + "_" + name_in + "_controller/command";
    
    // Initialize variables
    id = -1;
    position = 0.0;
    desired = 0.0;
    velocity = 0.0;
    
    // Setup publishers and subscribers
    ControlPub = nh.advertise<std_msgs::Float64>(command_topic, 1);
}

void SimServo::setCommandOutput(){
    
    // Publish commands
    std_msgs::Float64 commandMsg;
    commandMsg.data = desired;
    ControlPub.publish(commandMsg);
}
