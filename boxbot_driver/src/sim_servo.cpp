// Gazebo simulated-servo class.
//
// Copyright 2020 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the
// commit history.

#include "boxbot_driver/sim_servo.h"

// Joint state callback function
void SimServo::jointStateCB(const sensor_msgs::JointState& msg){
    
    // Get index of joint in JointState topic, if we don't know it yet.
    if (index == -1){
        std::vector<std::string> jointNames = msg.name;
        int i=0;
        while (i<14){
            if (jointNames[i] == name){
                index = i;
                i = 14;
            }
            i++;
        }
    }
    
    // Record positions and velocity
    position = msg.position[index];
    velocity = msg.velocity[index];
}

SimServo::SimServo(std::string name_in, std::string side_in, ros::NodeHandle nh, std::string robot_in){
    
    // Setup names
    name = name_in;
    side = side_in;
    robot = robot_in;
    commandTopic = robot + "/" + side + "_" + name + "_controller/command";
    jointStateTopic = robot + "/joint_states";
    
    // Initialize variables
    index = -1;
    position = 0.0;
    desired = 0.0;
    velocity = 0.0;
    
    // Setup publishers and subscribers
    ControlPub = nh.advertise<std_msgs::Float64>(commandTopic, 1);
    JointStateSub = nh.subscribe(jointStateTopic, 1, &SimServo::jointStateCB, this);
}

void SimServo::setCommandOutput(){
    
    // Publish commands
    std_msgs::Float64 commandMsg;
    commandMsg.data = desired;
    ControlPub.publish(commandMsg);
}
