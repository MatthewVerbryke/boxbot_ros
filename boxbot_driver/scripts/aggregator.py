#!/usr/bin/env python

"""
  A module to combine JointStates messages from various hardware/sim 
  sources (i.e. arms, neck, legs, etc.) into one unified JointState 
  message. This makes it possible to use tf on a distributed setup
  (hopefully).
  
  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""

import sys
import thread

import rospy
from sensor_msgs.msg import JointState


class JointStateAggregator(object):
    """
    ROS node for aggregating joint state messages across the system from
    multiple sources.
    
    WARN: Do not run on same computer as Gazebo, as it will overwrite 
          the "$ROBOT/joint_state" message.
    """
    
    def __init__(self):
        
        # Initialize Node
        rospy.init_node("joint_state_aggragator")
        
        # Initialize cleanup
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Get parameters
        self.robot = "boxbot"
        self.rate =  50.0

	    # Message storage variables
        self.left_joint_state_msg = JointState()
        self.right_joint_state_msg = JointState()
        
        # Setup topic names
        left_joint_state = "{}/left_arm/joint_states".format(self.robot)
        right_joint_state = "{}/right_arm/joint_states".format(self.robot)
        combined_state = "{}/joint_states".format(self.robot)
        
        # Setup publishers and subscribers
        self.combined_pub = rospy.Publisher(combined_state, JointState,
                                            queue_size = 1)
        self.left_sub = rospy.Subscriber(left_joint_state, JointState, 
                                         self.left_joint_state_cb)
        self.right_sub = rospy.Subscriber(right_joint_state, JointState,
                                          self.right_joint_state_cb)
        
        # Wait for subscribed topics to publish before continuing
        rospy.sleep(5)
        rospy.loginfo("'joint_state_aggragator' node initialized")
        
        # Run main script
        self.main()
        
    def cleanup(self):
        """
        Things to do on shutdown
        """
        rospy.loginfo("Shutting down 'joint_state_aggragator' node")
        rospy.sleep(1)
    
    def left_joint_state_cb(self, msg):
        """
        Callback function for the left joint state message
        """
        self.left_joint_state_msg = msg
        
    def right_joint_state_cb(self, msg):
        """
        Callback function for the right joint state message
        """
        self.right_joint_state_msg = msg
    
    def main(self):
        """
        Main execution function
        """
        
        r = rospy.Rate(self.rate)
        
        while not rospy.is_shutdown():
            
            # Acquire a lock
            self.lock.acquire()
            
            try:
                # Retrieve current messages
                left_cur = self.left_joint_state_msg
                right_cur = self.right_joint_state_msg
                
                # Combine like items into unified lists
                combined_name = left_cur.name + right_cur.name
                combined_position = left_cur.position + right_cur.position
                combined_velocity = left_cur.velocity + right_cur.velocity
                combined_effort = left_cur.effort + right_cur.effort
                
                # Create a new combined joint state message
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.name = combined_name
                msg.position = combined_position
                msg.velocity = combined_velocity
                msg.effort = combined_effort
                
                # Publish message
                self.combined_pub.publish(msg)
            
            finally:
                
                # Release lock
                self.lock.release()
                
                r.sleep()


if __name__ == "__main__":
    try:
        JointStateAggregator()
    except rospy.ROSInterruptException:
        pass
        
    sys.exit(0)

