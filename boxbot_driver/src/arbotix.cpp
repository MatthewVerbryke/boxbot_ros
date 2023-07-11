// Simulated ArbotiX driver class.
//
// Copyright 2022-2023 University of Cincinnati
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

#include "boxbot_driver/control_table.h"
#include "boxbot_driver/dynamixel.h"
#include "boxbot_driver/interface.h"

class ArbotiX
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
    
    // Serial port info
    std::string port_name;
    int baud_rate;
    int time_out;
    SerialInterface Interface;
    
    // Topics
    std::string arm_state_topic;
    std::string joint_command_topic;
    
    // Variables
    bool first_read;
    long iter;
    std::vector<Dynamixel> servo_vector;
    int num_servos;
    std::vector<int> read_list;
    ros::Time w_next;
    ros::Time r_next;
    
    // ROS Publishers/Subscribers
    ros::Publisher armStatePub;
    ros::Subscriber jointCommandSub;
    
    // Command callback function
    void jointCommandCB(const sensor_msgs::JointState &msg){
        
        // Catch empty message command message; TODO: expand checks?
        if (msg.position.empty()){
            ROS_WARN("No values detected in joint command");
        }
        
        // Extract commands from message
        else {
            for (int i=0; i<num_servos; ++i){
                double desired_pos = msg.position[i];
                servo_vector[i].setControlOutput(desired_pos);
            }
        }
        
        // Set flag if this is the first command message
        if (first_cmd == false){
            first_cmd = true;
        }
    }
    
public:

    // Constructor
    ArbotiX(){
        
        // Set node handle
        ros::NodeHandle nh("~"); // Private node handle
        ros::NodeHandle n(""); // Public node handle; outside this nodes namespace
        
        // ROS parameters
        nh.param("rate", rate, 100.0);
        nh.param("read_rate", read_rate, 10.0);
        nh.param("write_rate", write_rate, 10.0);
        nh.getParam("robot", robot);
        nh.getParam("side", side);
        
        // Setup serial port
        nh.getParam("port", port_name);
        nh.param("baud", baud_rate, 115200);
        nh.param("timeout", time_out, 250);   
        Interface.setupPort(port_name, baud_rate, time_out);
        
        // Setup names
        arm_state_topic = "/" + robot + "/" + side + "_arm/joint_state";
        joint_command_topic = "/" + robot + "/" + side + "_arm/joint_command";
        
        // Set the check inputs flag
        check_inputs = false;
        
        // Check what info the node has recieved, for debugging
        if (check_inputs == true){
            std::cout << "" << std::endl;
            std::cout << "READ PARAMETERS" << std::endl;
            std::cout << "" << std::endl;
            std::cout << "robot: " << robot << std::endl;
            std::cout << "side: " << side << std::endl;
            std::cout << "" << std::endl;
            std::cout << "port name: " << port_name << std::endl;
            std::cout << "baud rate: " << baud_rate << std::endl;
            std::cout << "timeout: " << time_out << std::endl;
            std::cout << "loop rate: " << rate << std::endl;
            std::cout << "read rate: " << read_rate << std::endl;
            std::cout << "write rate: " << write_rate << std::endl;
            std::cout << "" << std::endl;
        }
        
        // Setup initial read and write times
        w_delta = ros::Duration(1/write_rate);
        w_next = ros::Time::now() + w_delta;
        r_delta = ros::Duration(1/read_rate);
        r_next = ros::Time::now() + r_delta;
        
        // Initialize variables
        iter = 0;
        
        // Setup publishers and subscribers
        armStatePub = n.advertise<sensor_msgs::JointState>(arm_state_topic, 1);
        jointCommandSub = n.subscribe(joint_command_topic, 1, &ArbotiX::jointCommandCB, this);
        
        // Setup joint list
        nh.getParam("joint_names", joints);        
        if (joints.size() == 0){
            ROS_ERROR("No joint names read in from parameter server");
        }
        else {
            for (int i=0; i<joints.size(); ++i){
                std::string name = joints[i];
                Dynamixel new_servo(name, side, n, nh, robot);
                read_list.push_back(new_servo.getID());
                servo_vector.push_back(new_servo);
            }
        }
        
        // Store number of servos
        num_servos = servo_vector.size();
    }
    
    // Access functions
    double getRate(){return rate;};
    std::string getSide(){return side;};
    
    // Publish commands and get current servo states
    void update(){
        
        // Read current servo positions from Arbotix
        if (ros::Time::now() >= r_next){
            std::vector<int> values;
            values = Interface.read(read_list, P_PRESENT_POSITION_L, 2);
            
            // TODO: Check if we got something
            
            // Process and store output values
            for (int i=0; i<num_servos; ++i){
                int processed = values[2*i] + (values[2*i+1]<<8);
                servo_vector[i].setCurrentFeedback(processed);
            }
            
            // Build arm joint state message
            sensor_msgs::JointState arm_state_msg;
            arm_state_msg.header.stamp = ros::Time::now();
            for (int j=0; j<servo_vector.size(); ++j){
                std::string new_name = servo_vector[j].getFullName();
                double new_position = servo_vector[j].getPosition();
                double new_velocity = servo_vector[j].getVelocity();
                arm_state_msg.name.push_back(new_name);
                arm_state_msg.position.push_back(new_position);
            }
            
            // Publish state message
            armStatePub.publish(arm_state_msg);
            
            // Update next read time
            r_next = ros::Time::now() + r_delta;
        }
        
        // Write commands to servos
        if (first_cmd == true){
            if (ros::Time::now() >= w_next){
                std::vector<int> write_values;
                
                // Construct messages for each servo
                for (int j=0; j<num_servos; ++j){
                    int v = servo_vector[j].interpolate(w_delta.toSec());
                    write_values.push_back(servo_vector[j].getID());
                    write_values.push_back(v%256);
                    write_values.push_back(v>>8);
                }
                
                // Write the commands to the Arbotix
                int error = Interface.write(write_values, P_GOAL_POSITION_L, 2);
                
                // Update next write time
                w_next = ros::Time::now() + w_delta;
            }
            
        // Update iteration number
        iter++;
        }
    }
};
    
// Main loop
int main(int argc, char **argv){
    
    // Initialize node
    ros::init(argc, argv, "Arbotix");
    
    // Initialize arbotix driver
    ArbotiX ArbotixDriver;
    std::string side = ArbotixDriver.getSide();
    ROS_INFO_STREAM("ArbotiX board driver initialized for " + side + " WidowX arm");
    
    // Set loop rate
    double loop_rate = ArbotixDriver.getRate();
    ros::Rate r(loop_rate);
    
    // Pause briefly so Gazebo/ROS can fully launch
    ros::Duration(2.0).sleep();
    
    // Start reading servo information
    ArbotixDriver.update();
    ROS_INFO_STREAM("Starting servo updates");
    
    // Run main loop
    while (ros::ok()){
        ArbotixDriver.update();
        ros::spinOnce();
        r.sleep();
    }
}
