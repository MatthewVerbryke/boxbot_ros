#!/usr/bin/env python

"""
  Publishes the joint state message for one arm from the simulation 
  computer to another computer (or series of computers).

  Copyright 2018-2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test
"""


import sys
import thread

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import rospy

from packing import pack_jointstate

# Retrieve Rosbridge/Websocket utilities
file_dir = sys.path[0]
sys.path.append(file_dir + '/../..')
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
from rss_git_lite.common import rosConnectWrapper as rC


class ArmStatePublisher():
    """ 
    A ROS node to publish the joint states of the Boxbot arms to the secondary
    computer.
    """
    
    def __init__(self):
        
        # Inititialize node
        rospy.init_node("arm_state_publisher", anonymous=True)
        
        # Initialize cleanup for this node
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Get parameters
        robot = rospy.get_param("~/robot", "")
        side = rospy.get_param("~/side", "")
        ip = rospy.get_param("~/connections/{}_arm".format(side))
        
        # Parse out joint information from param server
        self.joints = []
        for name in rospy.get_param("~joints", dict().keys()):
            self.joints.append(side + "_" + name)
        
        # Message storage variable
        self.robot_joint_state = None
        
        # Set rate (100 Hz)
        self.rate = rospy.Rate(100.0)
        
        # Get topics
        whole_joint_state = "{}/joint_states".format(robot)
        arm_joint_state = "{}/{}_joint_state".format(robot, side)
        
        # Setup ROS Subscriber
        rospy.Subscriber(whole_joint_state, JointState, self.joint_state_cb)
        
        # Setup ROSbridge publisher
        if ip == "local":
            self.local = True
            self.pub = rospy.Publisher(whole_joint_state, Float64,
                                       queue_size=5)
        else:
            self.local = False
            self.pub = rC.RosMsg("ws4py", ip, "pub", arm_state_topic,
                                 "sensor_msgs/JointState", 
                                 pack_jointstate)
        
        # Run publisher
        rospy.loginfo("{} side arm state publisher initialized")
        self.main()
        
    def get_joint_states(self, msg):
        """
        Callback for joint states.
        """
        
        self.robot_joint_state = msg
    
    def copy_and_clear_received(self):
        """
        Clear out received data once retrieved/pulled.
        """
        
        # Set lock
        self.lock.acquire()
        
        # Send the clock time and clear the state variable
        cur_joint_state = self.robot_joint_state
        self.robot_joint_state = None
        
        # Release lock
        self.lock.release()
        
        return cur_joint_state
        
    def parse_joint_state(self, msg):
        """
        Get out information related to this arm from full joint state
        message.
        """
        
        position = []
        velocity = []
        effort = []
        
        # Find names in message
        for joint in self.joints:
            i = msg.name.index(joint)
            position.append(msg.position[i])
            velocity.append(msg.velocity[i])
            effort.append(msg.effort[i])
            
        # Fill out truncated message for just this arm
        arm_joint_state = JointState()
        arm_joint_state.header = msg.header
        arm_joint_state.name = self.joints
        arm_joitn_state.position = position
        arm_joint_state.velocity = velocity
        arm_joint_state.effort = effort
        
        return arm_joint_state
        
    def main(self):
        """
        Retrieve and publish the current state of the arm.
        """

        while not rospy.is_shutdown():
            
            # Check for a new joint state message
            joint_state = self.copy_and_clear_received()
            
            # If there is a new a message, publish it.
            if joint_state == None:
                pass
            else:
                arm_joint_state = self.parse_joint_state(joint_state)
                if local:
                    self.pub.publish(arm_joint_state)
                else:
                    self.pub.send(arm_joint_state)
                
            self.rate.sleep()

    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown
        rospy.loginfo("Shutting down '{}_arm_state_publisher'".format(self.arm))
        rospy.sleep(1)


if __name__ == "__main__":
    try:
        ArmStatePublisher()
    except rospy.ROSInterruptException:
        pass
        
    sys.exit(0)
