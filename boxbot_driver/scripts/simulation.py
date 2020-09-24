#!/usr/bin/env python

"""
  A simulated ArbotiX driver module for use with simulated Boxbot in 
  Gazebo.

  Copyright (c) 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import sys
import thread
import traceback

import rospy
from sensor_msgs.msg import JointState

from servo import SimServo

# Retrieve Rosbridge/Websocket utilities
file_dir = sys.path[0]
sys.path.append(file_dir + '/../../..')
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
from rss_git_lite.common import rosConnectWrapper as rC


class ArbotixGazeboDriver(object):
    """
    A simulated ArbotiX driver object.
    """
    
    def __init__(self):
        
        # Initialize Node
        rospy.init_node("sim_arbotix_driver")
        
        # Setup cleanup
        rospy.on_cleanup(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Get parameters
        self.side = rospy.get_param("~/side", "")
        self.robot = rospy.get_param("~/robot", "")
        self.rate = rospy.get_param("~/rate", 100.0)
        
        read_rate = rospy.get_param("~/readRate", 10.0)
        write_rate = rospy.get_param("~/writeRate", 10.0)
        controllers = rospy.get_param("~/controllers")
        command_ip = rospy.get_param("~/command_ip")
        state_ip = rospy.get_param("~/state_ip")
        
        # Create servo objects
        self.servos = []
        for name in controllers:
            new_servo = SimServo(name, self.side, self.robot, command_ip)
            self.servo.append(new_servo)
            
        # Setup topic names
        whole_joint_state = "{}/joint_states".format(self.robot)
        arm_joint_state = "{}/{}_arm/joint_states".format(self.robot,
                                                         self.side)
        arm_commands = "{}/{}_arm/joint_commands".format(self.robot,
                                                        self.side)
        
        # Setup read/write rates
        self.w_delta = rospy.Duration(1.0/write_rate)
        self.r_delta = rospy.Duration(1.0/read_rate)
        self.w_next = rospy.Time.now() + self.w_delta
        self.r_next = rospy.Time.now() + self.r_delta
        
        # Create ROS subcribers
        self.state_sub = rospy.Subscriber(whole_joint_state, JointState,
                                          self.joint_state_cb)
        self.command_sub = rospy.Subscriber(arm_commands, JointState,
                                            self.joint_commands_cb)
        
        # Setup arm state publisher
        if state_ip == "local":
            self.local = True
            self.state_pub = rospy.Publisher(arm_joint_state, JointState,
                                             queue_size=1)
        else:
            self.local = False
            self.state_pub = rC.RosMsg("ws4py", state_ip, "pub", 
                                       arm_joint_state,
                                       "sensor_msgs/JointState",
                                       pack_joint_state)
        
        # Run main program
        rospy.loginfo("{} arm driver initialized".format(self.side))
        self.main()
        
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown
        rospy.loginfo("Shutting down {} arm driver".format(self.side))
        rospy.sleep(1)
        
    def joint_state_cb(self, msg):
        """
        Callback for joint state messages from Gazebo.
        """
        
        self.joint_state_msg = msg
        
    def joint_commands_cb(self, msg):
        """
        Callback for joint command messages.
        """
        
        for servo in self.servos:
            i = msg.name.index(servo.name)
            servo.desired = msg.position[i]
            
    def update_state(self):
        """
        Publish the current joint state of the arm.
        """
        
        # Acquire lock
        self.lock.acquire()
        
        state_msg = JointState()
        state_msg.header.stamp = rospy.Time.now()
        
        try:
            # Update servo information
            for servo in self.servos:
                servo.update_joint_info(self.joint_state_msg)
                
                # Fill out state message
                state_msg.name.append(servo.name)
                state_msg.positions.append(servo.position)
                state_msg.velocity.append(servo.velocity)
                    
            # Publish message
            if self.local:
                self.state_pub.publish(state_msg)
            else:
                self.state_pub.send(state_msg)
                
        finally:
            # Release lock
            self.lock.release()
            
    def publish_commands(self):
        """
        Publish the current joint commands to Gazebo.
        """
        
        # Acquire lock
        self.lock.acquire()
        
        try:
            # Publish command message
            for servo in self.servo:
                servo.publish_command()
                
        finally:
            # Release lock
            self.lock.release()                
    
    def main(self):
        """
        Main execution function for the class
        """
        
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            
            # Read and publish state 
            if rospy.Time.now() >= self.r_next:
                self.update_state()
                self.r_next = rospy.Time.now() + self.r_delta
            
            # Write commands
            if rospy.Time.now() >= self.w_next:
                self.publish_commands()
                self.w_next = rospy.Time.now() + self.w_delta
                
            # Hold cycle rate
            r.sleep()


if __name__ == "__main__":
    try:
        ArbotixGazeboDriver()
    except rospy.ROSInterruptException:
        pass
        
    sys.exit(0)
