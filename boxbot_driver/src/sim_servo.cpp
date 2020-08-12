// Gazebo simulated-servo class.
//
// Copyright 2020 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the
// commit history.

#include "boxbot_driver/sim_servo.h"

SimServo::SimServo(std::string nameIn, std::string sideIn, ros::NodeHandle nh, std::string robotIn){
    
    // Setup names
    side = sideIn;
    robot = robotIn;
    name = side + "_" + nameIn + "_joint";
    commandTopic = robot + "/" + side + "_" + nameIn + "_controller/command";
    jointStateTopic = robot + "/joint_states";
    
    // Initialize variables
    index = -1;
    position = 0.0;
    desired = 0.0;
    velocity = 0.0;
    
    // Setup publishers and subscribers
    ControlPub = nh.advertise<std_msgs::Float64>(commandTopic, 1);
}

void SimServo::setCommandOutput(){
    
    // Publish commands
    std_msgs::Float64 commandMsg;
    commandMsg.data = desired;
    ControlPub.publish(commandMsg);
}
