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
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Get parameters
        self.side = rospy.get_param("~side", "")
        self.robot = rospy.get_param("~robot", "")
        self.rate = 50.0#rospy.get_param("~rate", 50.0)
        read_rate = rospy.get_param("~read_rate", 10.0)
        write_rate = rospy.get_param("~write_rate", 10.0)
        
        # Get full joint names
        joint_dict = rospy.get_param("~joints")
        arm_joints = joint_dict["arm"]
        eef_joints = joint_dict["eef"]
        joints = arm_joints + eef_joints
        
        # Message holding variables
        self.joint_state_msg = None
        self.joint_commands = None
        
        # Get ROSbridge publishers destinations
        local_ip = rospy.get_param("~connections/{}_arm".format(self.side))
        command_ip = rospy.get_param("~connections/sim")
        if command_ip == local_ip:
            rospy.loginfo("Publishing commands to local ROS environment")
            command_ip = "local" # TODO: rework this
        else:
            rospy.loginfo("Publishing commands to ws://{}:9090/".format(command_ip))
        
        # Create servo objects
        self.servos = []
        for name in joints:
            new_servo = SimServo(name, self.side, self.robot, command_ip)
            self.servos.append(new_servo)

        # Setup topic names
        sim_joint_state = "{}/{}_joint_state".format(self.robot,
                                                         self.side)
        arm_joint_state = "{}/{}_arm/joint_states".format(self.robot,
                                                          self.side)
        arm_commands = "{}/{}_arm/commands".format(self.robot,
                                                   self.side)
        
        # Setup read/write rates
        self.w_delta =rospy.Duration(0.01)#rospy.Duration(1.0/write_rate)
        self.r_delta = rospy.Duration(0.01)#rospy.Duration(1.0/read_rate)
        self.w_next = rospy.Time.now() + self.w_delta
        self.r_next = rospy.Time.now() + self.r_delta
        
        # Create ROS subcribers
        self.state_sub = rospy.Subscriber(sim_joint_state, JointState,
                                          self.joint_state_cb)
        self.command_sub = rospy.Subscriber(arm_commands, JointState,
                                            self.joint_commands_cb)
                                            
        # Setup arm state publisher
        self.local = True
        self.state_pub = rospy.Publisher(arm_joint_state, JointState,
                                         queue_size=1)
        rospy.loginfo("Publishing state data to the local ROS environment")
        
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
        
        self.joint_commands = msg
            
    def update_state(self):
        """
        Publish the current joint state of the arm.
        """
        
        # Acquire lock
        self.lock.acquire()
        
        state_msg = JointState()
        state_msg.header.stamp = rospy.Time.now()
        name = []
        position = []
        velocity = []
        
        try:
        
            # Update servo information
            for servo in self.servos:
                self.joint_state_msg
                servo.update_joint_info(self.joint_state_msg)
                
                # Fill out state message
                name.append(servo.name)
                position.append(servo.position)
                velocity.append(servo.velocity)
            
            state_msg.name = name
            state_msg.position = position
            state_msg.velocity = velocity
            
            # Clear out old state message
            self.joint_state_msg = None
                
            return state_msg
            
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
            
            msg = self.joint_commands
            
            for servo in self.servos:
                
                # Assign desired joint positions to correct servo
                if msg.name[servo.index] != servo.name:
                    servo.index = msg.name.index(servo.name)
                servo.desired = msg.position[servo.index]
                
                # Publish command message
                servo.publish_command()
                
            # Clear out old commands
            self.joint_commands = None
                
        finally:
            # Release lock
            self.lock.release()
        
    def main(self):
        """
        Main execution function for the class
        """
        
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            
            # Read state
            #if rospy.Time.now() >= self.r_next:
            if self.joint_state_msg != None:
                state_msg = self.update_state()
                
                # Publish message
                if self.local:
                    self.state_pub.publish(state_msg)
                else:
                    self.state_pub.send(state_msg)
                
            # Update next read time
            #self.r_next = rospy.Time.now() + self.r_delta
                
            # Write commands
            #if rospy.Time.now() >= self.w_next:
            if self.joint_commands != None:
                self.publish_commands()
            
            # Update next write time
            #self.w_next = rospy.Time.now() + self.w_delta
        
            # Hold cycle rate
            r.sleep()
            
def pack_header(header):
    """
    Package 'std_msgs/Header' message.
    """
    
    # Get header info
    seq = header.seq
    secs = header.stamp.secs
    nsecs = header.stamp.nsecs
    frame = header.frame_id
    
    # Place into dictionary
    header_msg = {"seq": seq,
                  "stamp": {"secs": secs, "nsecs": nsecs},
                  "frame_id": frame}
                  
    return header_msg

def pack_joint_state(joint_state):
    """
    Package 'sensor_msgs/JointState' message
    """
    
    # Get header message
    header_msg = pack_header(joint_state.header)
        
    # Get joint state info
    names = joint_state.name
    positions = joint_state.position
    velocities = joint_state.velocity
    efforts = joint_state.effort
    
    # Package into dict
    joint_state_msg = {"header": header_msg,
                       "name": names,
                       "position": positions,
                       "velocity": velocities,
                       "effort": efforts}
    
    return joint_state_msg
            

if __name__ == "__main__":
    try:
        ArbotixGazeboDriver()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception")
        
    sys.exit(0)
