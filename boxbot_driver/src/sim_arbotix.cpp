// Simulated ArbotiX driver class.
//
// Copyright 2022-2026 University of Cincinnati
// All rights reserved. See LICENSE file at:
// https://github.com/MatthewVerbryke/rse_dam
// Additional copyright may be held by others, as reflected in the
// commit history.

#include <string>
#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include "boxbot_driver/sim_servo.h"

class SimArbotiX : public rclcpp::Node
{
private:
    
    // Parameters
    double rate;
    std::string robot;
    std::string side;
    double write_rate;
    double read_rate;
    rclcpp::Duration w_delta;
    rclcpp::Duration r_delta;
    std::vector<std::string> joints;
    bool check_inputs = false;
    bool first_cmd = false;
    std::chrono::duration interval;
    
    // Topic name 
    std::string arm_state_topic;
    std::string joint_command_topic;
    std::string gazebo_joint_topic;
    
    // Variables
    long iter;
    std::vector<SimServo> servo_vector;
    int num_servos;
    rclcpp::Time w_next;
    rclcpp::Time r_next;
    
    // ROS publishers and subscribers
    rclcpp::Publisher armStatePub;
    rclcpp::Subscription armCommandSub;
    rclcpp::Subscription gazeboJointSub;
    
    // ROS wall timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Command callback function
    void jointCommandCB(const sensor_msgs::JointState& msg){
        
        // Catch empty message command message; TODO: expand checks?
        if (msg.position.empty()){
            RCLCPP_WARN(this->get_logger(), "No values detected in joint command");
        }
        
        // Extract commands from message
        else {
            for (int i=0; i<num_servos; ++i){
                double desired_pos = msg.position[i];
                servo_vector[i].setDesired(desired_pos);
            }
        }
    }
    
    // Joint state callback function
    void jointStateCB(const sensor_msgs::JointState& msg){
        int msg_size = msg.name.size();
        for (int i=0; i<num_servos; ++i){
            
            // Find the index of the joint in the joint state message if we don't know already
            int joint_index = servo_vector[i].getID();
            std::string name = servo_vector[i].getFullName();
            if (joint_index == -1){
                int j=0;
                while (j<msg_size){
                    std::string joint_name = msg.name[j];
                    if (joint_name == name){
                        servo_vector[i].setID(j);
                        std::string id_note = name + " index: " + std::to_string(j);
                        RCLCPP_INFO(this->get_logger(), id_note);
                        j = msg_size;
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
    
    // Get joint states and publish commands
    void update(){
        
        // Get current time
        rclcpp::Time now = this->get_clock->now()
        
        // Read in current servo positions
        if (now >= r_next){
            sensor_msgs::JointState arm_state_msg;
            arm_state_msg.header.stamp = now;
            for (int i=0; i<num_servos; ++i){
                std::string new_name = servo_vector[i].getFullName();
                float new_position = servo_vector[i].getPosition();
                float new_velocity = servo_vector[i].getVelocity();
                arm_state_msg.name.push_back(new_name);
                arm_state_msg.position.push_back(new_position);
                arm_state_msg.velocity.push_back(new_velocity);
            }
            
            // Publish message
            armStatePub.publish(arm_state_msg);
            
            // Update new read time
            r_next = now + r_delta;
        }
        
        // Publish commands
        if (now >= w_next){
            
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
                w_next = now + w_delta;
            }
        }
        
        // Update iteration number
        iter++;
    }
    
public:
    SimArbotiX()
    : Node("SimArbotix")
    {
        // Declare parameters
        this->declare_parameter("rate", 100.0);
        this->declare_parameter("read_rate", 10.0);
        this->declare_parameter("write_rate", 10.0);
        this->declare_parameter("robot", "");
        this->declare_parameter("side", "");
        this->declare_parameter("joint_names", "");
        
        // Get parameters
        rate = this->get_parameter("rate").as_double();
        read_rate = this->get_parameter("read_rate").as_double();
        write_rate = this->get_parameter("write_rate").as_double();
        robot = this->get_parameter("robot").as_string()
        side = this->get_parameter("side").as_string()
        joints = this.get_parameter("joint_names").as_string_array();
        interval = std::chrono::milliseconds(1000/rate);
        
        // Setup names
        arm_state_topic = "/" + robot + "/" + side + "_arm/joint_states";
        joint_command_topic = "/" + robot + "/" + side + "_arm/joint_commands";
        gazebo_joint_topic = "/" + robot + "/joint_states";
        
        // Setup initial read and write times
        rclcpp::Duration r_delta(1/read_rate);
        r_next = this->get_clock->now() + r_delta;
        rclcpp::Duration w_delta(1/write_rate);
        w_next = this->get_clock->now() + w_delta;
        
        // Initialize variables
        iter = 0;
        
        // Setup publishers and subscribers
        armStatePub = this->create_publisher<sensor_msgs::JointState>(arm_state_topic, 1);
        armCommandSub = this->create_subscription<sensor_msgs::JointState(joint_command_topic, 1, jointCommandCB);
        gazeboJointSub = this->create_subscription<sensor_msgs::JointState(gazebo_joint_topic, 1, jointStateCB);
        
        // Setup joint list
        if (joints.size() == 0){
            RCLCPP_ERROR(this->get_logger(), "No joint names read in from parameter server");
        }
        else {
            for (int i=0; i<joints.size(); ++i){
                std::string name = joints[i];
                SimServo new_servo(name, side, this->shared_from_this(), robot);
                servo_vector.push_back(new_servo);
            }
        }
        
        // Store number of servos
        num_servos = servo_vector.size();
        
        // Setup wall timer
        RCLCPP_INFO(this->get_logger(), "Starting simulated servo updates")
        timer_ = this->create_wall_timer(interval, std::bind(&SimArbotiX::update, this));
    }
};

int main(int argc, char **argv){
	
	rclcpp::init(argc, argv);
	rclcpp::sleep_for(std::chrono::seconds(2.0));
	rclcpp::spin(std::make_shared<SimArbotiX>());
	rclcpp::shutdown();
	
	return 0;
}
