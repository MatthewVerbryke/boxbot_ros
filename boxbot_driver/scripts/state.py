#!/usr/bin/env python

"""
  Publishes the joint state message for one arm from the simulation 
  computer to another computer (or series of computers).

  Copyright 2018-2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import sys
import thread

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import rospy

# Retrieve Rosbridge/Websocket utilities
file_dir = sys.path[0]
sys.path.append(file_dir + '/../../..')
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
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Get parameters
        robot = rospy.get_param("~robot")
        side = rospy.get_param("~side")
	self.side = side
        ip = rospy.get_param("~connections/{}_arm".format(side))
        
        # Parse out joint information from param server
        self.joints = []
        for name in rospy.get_param("~joints", dict().keys()):
            self.joints.append(side + "_" + name)
        
        # Setup arm state storage variable
        self.arm_state = JointState()
        self.arm_state.name = self.joints
        self.arm_state.position = [0.0]*len(self.joints)
        self.arm_state.velocity = [0.0]*len(self.joints)
        self.arm_state.effort = [0.0]*len(self.joints)
        self.j = [0]*len(self.joints)

        # Initialize cleanup for this node
        rospy.on_shutdown(self.cleanup)
        
        # Message storage variable
        self.robot_joint_state = None
        
        # Set rate (100 Hz)
        self.rate = rospy.Rate(100.0)
        
        # Get topics
        whole_joint_state = "{}/joint_states".format(robot)
        arm_joint_state = "{}/{}_joint_state".format(robot, side)
        
        # Setup ROS Subscriber
        rospy.Subscriber(whole_joint_state, JointState, self.joint_state_cb)
        
        # Setup ROSbridge publisher or local publisher
        if ip == "local":
            self.local = True
            self.pub = rospy.Publisher(arm_joint_state, JointState,
                                       queue_size=5)
        else:
            self.local = False
            self.pub = rC.RosMsg("ws4py", "ws://"+ip+":9090/", "pub", arm_joint_state,
                                 "sensor_msgs/JointState", 
                                 pack_joint_state)
        
        # Run publisher
        rospy.loginfo("State publisher for {} arm initialized".format(side))
        self.main()
        
    def joint_state_cb(self, msg):
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
        Given the full state of boxbot, get out only the joint state of
        the current arm.
        """
        
        # Steal the header
        self.arm_state.header = msg.header
                
        # Retrieve information on arm joints
        for i in range(0, len(self.joints)):
            
            # Save time on indexes since they shouldn't change change
            j = self.j[i]
            if msg.name[j] != self.joints[i]:
                self.j[i] = msg.name.index(self.joints[i])
                j = self.j[i]
            
            # Fill out remaining data
            self.arm_state.position[i] = msg.position[j]
            self.arm_state.velocity[i] = msg.velocity[j]
            self.arm_state.effort[i] = msg.effort[j]
        
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
                self.parse_joint_state(joint_state)
                if self.local:
                    self.pub.publish(self.arm_state)
                else:
                    self.pub.send(self.arm_state)
                
            self.rate.sleep()

    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown
        rospy.loginfo("Shutting down '{}_state_publisher'...".format(self.side))
        rospy.sleep(1)

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
        

if __name__ == "__main__":
    try:
        ArmStatePublisher()
    except rospy.ROSInterruptException:
        pass
        
    sys.exit(0)
