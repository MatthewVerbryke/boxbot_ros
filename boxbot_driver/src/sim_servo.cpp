// Gazebo simulated-servo class.
//
// Copyright 2022-2023 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the
// commit history.

#include <string>
#include <math.h>

#include "boxbot_driver/sim_servo.h"

SimServo::SimServo(std::string name_in, std::string side_in, ros::NodeHandle nh, std::string robot_in){
    
    // Setup names
    side = side_in;
    robot = robot_in;
    name = name_in;
    full_name = side_in + "_" + name_in;
    std::string param_name = "joints/" + name_in;
    
    // Set gripper flag
    if (name == "gripper"){
        is_gripper = true;
    }
    else {
        is_gripper = false;
    }
    
    // Get parameters from server
    nh.getParam(param_name + "/id", id);
    
    // Build topic names
    command_topic = robot + "/" + side + "_" + name_in + "_controller/command";
    mimic_topic = robot + "/" + side + "_" + name_in + "_mimic_controller/command";
    
    // Initialize variables
    position = 0.0;
    desired = 0.0;
    velocity = 0.0;
    
    // Setup publishers and subscribers
    ControlPub = nh.advertise<std_msgs::Float64>(command_topic, 1);
    if (is_gripper){
        MimicPub = nh.advertise<std_msgs::Float64>(mimic_topic, 1);
    }
}

void SimServo::setCommandOutput(){
    
    // Calculate desired gripper opening (if needed)
    if (is_gripper){
        opening = 2*sin(desired)*0.02 + 0.01;
    }

    // Build command message
    std_msgs::Float64 commandMsg;
    if (is_gripper){
        commandMsg.data = opening;
    }
    else {
        commandMsg.data = desired;
    }
    
    // Publish commands
    ControlPub.publish(commandMsg);
    if (is_gripper){
        MimicPub.publish(commandMsg);
    }
}
