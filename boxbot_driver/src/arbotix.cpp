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

#include "boxbot_driver/control_table.h"
#include "boxbot_driver/dynamixel.h"
#include "boxbot_driver/interface.h"

class ArbotiX{

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
    
    // Serial port info
    std::string port_name;
    int baud_rate;
    int time_out;
    SerialInterface Interface;
    
    // Topics
    std::string arm_state_topic;
    std::string joint_command_topic;
    
    // Variables
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
        for (int i=0; i<num_servos; ++i){
            double desired_pos = msg.position[i];
            servo_vector[i].setControlOutput(desired_pos);
        }
    }
    
public:

    // Constructor
    ArbotiX(){
        
        // Set node handle
        ros::NodeHandle nh("~");
        
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
        
        // Setup initial read and write times
        ros::Duration w_delta(1/write_rate);
        w_next = ros::Time::now() + w_delta;
        ros::Duration r_delta(1/read_rate);
        r_next = ros::Time::now() + r_delta;
        
        // Initialize variables
        iter = 0;
        
        // Setup publishers and subscribers
        armStatePub = nh.advertise<sensor_msgs::JointState>(arm_state_topic, 1);
        jointCommandSub = nh.subscribe(joint_command_topic, 1, &ArbotiX::jointCommandCB, this);
        
        // Setup joint list
        nh.getParam("joint_names", joints);        
        if (joints.size() == 0){
            ROS_ERROR("No joint names read in from parameter server");
        }
        else {
            for (int i=0; i<joints.size(); ++i){
                std::string name = joints[i];
                Dynamixel new_servo(name, side, nh, robot);
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
                
            // Process and store output values
            for (int i=0; i<num_servos; ++i){
                int processed = values[2*i] + values[2*i+1]<<8;
                servo_vector[i].setCurrentFeedback(processed);
            }
            
            // Publish system joint state message
            //TODO
            
            // Update next read time
            r_next = ros::Time::now() + r_delta;
        }
        
        // Write commands to servos
        if (ros::Time::now() >= w_next){
            std::vector<int> write_values;
            
            // Construct messages for each servo
            for (int j=0; j<num_servos; ++j){
                int delta_ws = w_delta.toSec();
                int v = servo_vector[j].interpolate(1/(delta_ws));
                write_values.push_back(servo_vector[j].getID());
                write_values.push_back(v%256);
                write_values.push_back(v>>8);
            }
            
            // write the commands to the Arbotix
            int error = Interface.write(write_values, P_GOAL_POSITION_L, 2);
            
            // Update next write time
            w_next = ros::Time::now() + w_delta;
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
    ROS_INFO_STREAM("ArbotiX board driver initialized for " + side + " arm");
    
    // Set loop rate
    double loop_rate = ArbotixDriver.getRate();
    ros::Rate r(loop_rate);
    
    // Pause briefly so Gazebo/ROS can fully launch
    ros::Duration(2.0).sleep();
    
    // Run main loop
    while (ros::ok()){
        ArbotixDriver.update();
        ros::spinOnce();
        r.sleep();
    }
}
