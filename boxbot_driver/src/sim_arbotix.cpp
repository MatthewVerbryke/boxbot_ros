// Simulated ArbotiX driver class for the Boxbot robot.
//
// Copyright 2020 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the
// commit history.

#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "boxbot_driver/sim_servo.h"

class SimArbotiX
{
private:
    
    // Parameters
    double rate;
    std::string robot;
    std::string side;
    double writeRate;
    double readRate;
    ros::Duration wDelta;
    ros::Duration rDelta;
    XmlRpc::XmlRpcValue jointNames;
    std::vector<std::string> jointNamesVector;
    
    // Topics 
    std::string armStateTopic;
    std::string jointCommandTopic;
    
    // Variables
    long iter;
    std::vector<SimServo> servoVector;
    ros::Time wNext;
    ros::Time rNext;
    
    // ROS publishers and subscribers
    ros::Publisher armStatePub;
    ros::Subscriber armCommandSub;
    
    // Command callback function
    void jointCommandCB(const sensor_msgs::JointState& msg){
        for (int i=0; i<jointNames.size(); ++i){
            servoVector[i].setDesired(msg.position[i]);
        }
    }
    
public:
    SimArbotiX(){
        // Set node handle
        ros::NodeHandle nh("~");
        
        // Setup names
        armStateTopic = "/" + side + "_arm_joint_states";
        jointCommandTopic = "/" + side + "_joint_commands";
        
        // Get ROS parameters
        nh.param("rate", rate, 100.0);
        nh.param("read_rate", readRate, 10.0);
        nh.param("write_rate", writeRate, 10.0);
        nh.getParam("robot", robot);
        nh.getParam("side", side);
        
        // Setup initial read and write times
        ros::Duration rDelta(1/readRate);
        rNext = ros::Time::now() + rDelta;
        ros::Duration wDelta(1/writeRate);
        wNext = ros::Time::now() + wDelta;
        
        // Initialize variables
        iter = 0;
        
        // Setup publishers and subscribers
        armStatePub  = nh.advertise<sensor_msgs::JointState>(armStateTopic, 1);
        armCommandSub = nh.subscribe(jointCommandTopic, 1, &SimArbotiX::jointCommandCB, this);
        
        // Setup joint list
        // See ROS Answers: "Retrieve list of lists from yaml file / parameter server"
        nh.getParam("joint_names", jointNames);
        if (jointNames.getType() != XmlRpc::XmlRpcValue::TypeArray){
            ROS_ERROR("parameter 'joint_names' is not a list");
        }
        else {
            for (int i=0; i<jointNames.size(); ++i){
                std::string name = jointNames[i];
                SimServo new_servo(name, side, nh, robot);
                servoVector.push_back(new_servo);
            }
        }
    }
    
    // Access function
    double getRate(){return rate;};
    
    // Get joint states and publish commands
    void update(){
        
        // Read in the current servo positions
        if (ros::Time::now() >= rNext){
            ros::spin();
            
            // Build joint state message
            sensor_msgs::JointState armStateMsg;
            armStateMsg.header.stamp = ros::Time::now();
            for (int i=0; i<servoVector.size(); ++i){
                std::string newName = servoVector[i].getName();
                float newPosition = servoVector[i].getPosition();
                float newVelocity = servoVector[i].getVelocity();
                armStateMsg.name.push_back(newName);
                armStateMsg.position.push_back(newPosition);
                armStateMsg.velocity.push_back(newVelocity);
            }
            
            // Publish message
            armStatePub.publish(armStateMsg);
        }
        
        // Publish commands
        if (ros::Time::now() >= wNext){
            for (int j=0; j<servoVector.size(); ++j){
                
                // If first iteration publish the current states to keep stationary
                if (iter == 0){
                    double firstCommand = servoVector[j].getPosition();
                    servoVector[j].setDesired(firstCommand);
                    servoVector[j].setCommandOutput();
                }
                else{
                    servoVector[j].setCommandOutput();
                }
            }
        }
        
        // Update iteration number
        iter++;
    }
};

int main(int argc, char **argv){
    
    // Initialize node
    ros::init(argc, argv, "Sim_Arbotix");
    
    // Initialize simulated arbotix
    SimArbotiX arbotix;
    
    // Set loop rate
    double loop_rate = arbotix.getRate();
    ros::Rate r(loop_rate);
    
    // Main Loop
    ros::spinOnce();
    while (ros::ok())
    {
        arbotix.update();
        r.sleep();
    }
}
