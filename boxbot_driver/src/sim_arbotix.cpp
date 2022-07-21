// Simulated ArbotiX driver class.
//
// Copyright 2022 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the
// commit history.
//
// TODO: Test

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
    double write_rate;
    double read_rate;
    ros::Duration w_delta;
    ros::Duration r_delta;
    std::vector<std::string> joints;
    bool check_inputs = false;
    bool first_cmd = false;
    
    // Topics 
    std::string arm_state_topic;
    std::string joint_command_topic;
    std::string gazebo_joint_topic;
    
    // Variables
    long iter;
    std::vector<SimServo> servo_vector;
    int num_servos;
    ros::Time w_next;
    ros::Time r_next;
    
    // ROS publishers and subscribers
    ros::Publisher armStatePub;
    ros::Subscriber armCommandSub;
    ros::Subscriber gazeboJointSub;
    
    // Command callback function
    void jointCommandCB(const sensor_msgs::JointState &msg){
        for (int i=0; i<num_servos; ++i){
            double desired_pos = msg.position[i];
            servo_vector[i].setDesired(desired_pos);
        }
    }
    
    // Joint state callback function
    void jointStateCB(const sensor_msgs::JointState &msg){
        int msg_size = msg.name.size();
        for (int i=0; i<num_servos; ++i){
            
            // Find the index of the joint in the joint state message if we don't know already
            int joint_index = servo_vector[i].getID();
            std::string name = servo_vector[i].getName();
            if (joint_index == -1){
                int j=0;
                while (j<msg_size){
                    std::string joint_name = msg.name[j];
                    if (joint_name == name){
                        servo_vector[i].setID(j);
                        ROS_INFO_STREAM(name + " index: " + std::to_string(j));
                        i = msg_size;
                    }
                    j++;
                }
            }
            
            // Set joint values
            else{
                int id = servo_vector[i].getID();
                double joint_position = msg.position[id];
                double joint_velocity = msg.velocity[id];
                servo_vector[i].setPosition(joint_position);
                servo_vector[i].setVelocity(joint_velocity);
            }
        }
    }
    
public:
    SimArbotiX(){
        // Set node handle
        ros::NodeHandle nh("~");
        
        // Get ROS parameters
        nh.param("rate", rate, 100.0);
        nh.param("read_rate", read_rate, 10.0);
        nh.param("write_rate", write_rate, 10.0);
        nh.getParam("robot", robot);
        nh.getParam("side", side);
        
        // Setup names
        std::string robot_ns = "/" + robot;
        arm_state_topic = robot_ns + "/" + side + "_arm/joint_states";
        joint_command_topic = robot_ns + "/" + side + "_arm/joint_commands";
        gazebo_joint_topic = robot_ns + "/joint_states";
        
        // Setup initial read and write times
        ros::Duration r_delta(1/read_rate);
        r_next = ros::Time::now() + r_delta;
        ros::Duration w_delta(1/write_rate);
        w_next = ros::Time::now() + w_delta;
        
        // Initialize variables
        iter = 0;
        
        // Setup publishers and subscribers
        armStatePub  = nh.advertise<sensor_msgs::JointState>(arm_state_topic, 1);
        armCommandSub = nh.subscribe(joint_command_topic, 1, &SimArbotiX::jointCommandCB, this);
        gazeboJointSub = nh.subscribe(gazebo_joint_topic, 1, &SimArbotiX::jointStateCB, this);
        
        // Setup joint list
        nh.getParam("joint_names", joints);        
        if (joints.size() == 0){
            ROS_ERROR("No joint names read in from parameter server");
        }
        else {
            for (int i=0; i<joints.size(); ++i){
                std::string name = joints[i];
                SimServo new_servo(name, side, nh, robot);
                servo_vector.push_back(new_servo);
            }
        }
        
        // Store number of servos
        num_servos = servo_vector.size();
    }
    
    // Access functions
    double getRate(){return rate;};
    std::string getSide(){return side;};
    
    // Get joint states and publish commands
    void update(){
        
        // Read in the current servo positions
        if (ros::Time::now() >= r_next){
            
            // Build joint state message
            sensor_msgs::JointState arm_state_msg;
            arm_state_msg.header.stamp = ros::Time::now();
            for (int i=0; i<num_servos; ++i){
                std::string new_name = servo_vector[i].getName();
                float new_position = servo_vector[i].getPosition();
                float new_velocity = servo_vector[i].getVelocity();
                arm_state_msg.name.push_back(new_name);
                arm_state_msg.position.push_back(new_position);
                arm_state_msg.velocity.push_back(new_velocity);
            }
    
            // Publish message
            armStatePub.publish(arm_state_msg);
            
            // Update new read time
            r_next = ros::Time::now() + r_delta;
        }
        
        // Publish commands
        if (ros::Time::now() >= w_next){
            
            for (int j=0; j<num_servos; ++j){
                
                // If first iteration publish the current states to keep stationary
                if (iter == 0){
                    double first_cmd = servo_vector[j].getPosition();
                    servo_vector[j].setDesired(first_cmd);
                    servo_vector[j].setCommandOutput();
                }
                else{
                    servo_vector[j].setCommandOutput();
                }
                
                // Update new write time
                w_next = ros::Time::now() + w_delta;
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
    std::string side = arbotix.getSide();
    ROS_INFO_STREAM("Simulated ArbotiX board initialized for " + side + " arm");
    
    // Set loop rate
    double loop_rate = arbotix.getRate();
    ros::Rate r(loop_rate);
    
    // Sleep so that Gazebo and ROS-controller can fully launch
    ros::Duration(2.0).sleep();
    
    // Start simulated servo reading
    arbotix.update();
    ROS_INFO_STREAM("Starting simulated servo updates");
    
    // Main Loop
    while (ros::ok())
    {
        arbotix.update();
        ros::spinOnce();
        r.sleep();
    }
}
