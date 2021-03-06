// Simulated ArbotiX driver class.
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
    XmlRpc::XmlRpcValue controllers;
    
    // Topics 
    std::string armStateTopic;
    std::string jointCommandTopic;
    std::string gazeboJointTopic;
    
    // Variables
    long iter;
    std::vector<SimServo> servoVector;
    ros::Time wNext;
    ros::Time rNext;
    
    // ROS publishers and subscribers
    ros::Publisher armStatePub;
    ros::Subscriber armCommandSub;
    ros::Subscriber gazeboJointSub;
    
    // Command callback function
    void jointCommandCB(const sensor_msgs::JointState &msg){
        for (int i=0; i<servoVector.size(); ++i){
            double desiredPosition = msg.position[i];
            servoVector[i].setDesired(desiredPosition);
        }
    }
    
    // Joint state callback function
    void jointStateCB(const sensor_msgs::JointState &msg){
        int msgSize = msg.name.size();
        for (int i=0; i<servoVector.size(); ++i){
            
            // Find the index of the joint in the joint state message if we don't know already
            int jointIndex = servoVector[i].getIndex();
            std::string name = servoVector[i].getName();
            if (jointIndex == -1){
                int j=0;
                while (j<msgSize){
                    std::string jointName = msg.name[j];
                    if (jointName == name){
                        servoVector[i].setIndex(j);
                        ROS_INFO_STREAM(name + " index: " + std::to_string(j));
                        i = msgSize;
                    }
                    j++;
                }
            }
            
            // Set joint values
            else{
                int jointIndex = servoVector[i].getIndex();
                double jointPosition = msg.position[jointIndex];
                double jointVelocity = msg.velocity[jointIndex];
                servoVector[i].setPosition(jointPosition);
                servoVector[i].setVelocity(jointVelocity);
            }
        }
    }
    
public:
    SimArbotiX(){
        // Set node handle
        ros::NodeHandle nh("~");
        
        // Get ROS parameters
        nh.param("rate", rate, 100.0);
        nh.param("read_rate", readRate, 10.0);
        nh.param("write_rate", writeRate, 10.0);
        nh.getParam("robot", robot);
        nh.getParam("side", side);
        
        // Setup names
        std::string robotNs = "/" + robot;
        armStateTopic = robotNs + "/" + side + "_arm/joint_states";
        jointCommandTopic = robotNs + "/" + side + "_arm/joint_commands";
        gazeboJointTopic = robotNs + "/joint_states";
        
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
        gazeboJointSub = nh.subscribe(gazeboJointTopic, 1, &SimArbotiX::jointStateCB, this);
        
        // Setup joint list
        // See ROS Answers: "Retrieve list of lists from yaml file / parameter server"
        nh.getParam("controllers", controllers);
        if (controllers.getType() != XmlRpc::XmlRpcValue::TypeArray){
            ROS_ERROR("Parameter 'controllers' is not a list");
        }
        else {
            for (int i=0; i<controllers.size(); ++i){
                std::string name = controllers[i];
                SimServo new_servo(name, side, nh, robotNs);
                servoVector.push_back(new_servo);
            }
        }
    }
    
    // Access functions
    double getRate(){return rate;};
    std::string getSide(){return side;};
    
    // Get joint states and publish commands
    void update(){
        
        // Read in the current servo positions
        if (ros::Time::now() >= rNext){
            
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
        
        //// Publish commands
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
        
        // Update new read/write times
        rNext = ros::Time::now() + rDelta;
        wNext = ros::Time::now() + wDelta;
    }
};

int main(int argc, char **argv){
    
    // Initialize node
    ros::init(argc, argv, "Sim_Arbotix");
    
    // Initialize simulated arbotix
    SimArbotiX arbotix;
    std::string side = arbotix.getSide();
    ROS_INFO_STREAM("Simulated ArbotiX board initialized for " + side + " arm");
    
    // Set loop rate
    double loop_rate = arbotix.getRate();
    ros::Rate r(loop_rate);
    
    // Sleep so that Gazebo and ROS-controller can fully launch
    ros::Duration(2.0).sleep();
    
    // Main Loop
    while (ros::ok())
    {
        arbotix.update();
        ros::spinOnce();
        r.sleep();
    }
}
