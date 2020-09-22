#!/usr/bin/env python

"""
  Servo objects for use with an ArbotiX driver (simulated or otherwise)
  on the Boxbot robot.
   
  Copyright (c) 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy
from std_msgs.msg import Float64

# Retrieve Rosbridge/Websocket utilities
file_dir = sys.path[0]
sys.path.append(file_dir + '/../../..')
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
from rss_git_lite.common import rosConnectWrapper as rC


class DynamixelServo(object):
    """
    TODO.
    """
    
    def __init__(self):
        pass
    
    
class SimServo(object):
    """
    An object for a simulated Dynamixel servo in Gazebo.
    """
    
    def __init__(self, name, side, robot, command_ip):
        
        # Setup parameters for this servo
        namespace = "~joints/{}/".format(name)
        self.name = "{}_{}".format(side, name)
        command_topic = "{}/{}_{}_controller/command".format(robot, side, name)
        if command_ip == "local":
            self.local = True
            ip = None
        else:
            self.local = False
            ip = command_ip
        
        # Setup joint parameters
        self.id = int(rospy.get_param(namespace+"id"))
        
        # Setup joint variables
        self.position = 0.0
        self.desired = 0.0
        self.velocity = 0.0
        
        # Setup publishers and subscribers
        if local:
            self.pub = rospy.Publisher(command_topic, Float64, queue_size=5)
        else:
            self.pub = rC.RosMsg("ws4py", ip, "pub", command_topic,
                                 "std_msgs/Float64", self.pack_float)
        
    def update_joint_info(self, msg):
        """
        Update the locally stored state of the joint (simulates the 
        """
        
        names = msg.name
        index = names.index(self.name)
        self.position = msg.position[index]
        self.velocity = msg.velocity[index]
        
    def publish_command(self):
        """
        Publish the current desired position to Gazebo.
        """
        
        # Get command
        command = self.desired
        
        # Publish
        if self.local:
            self.pub.publish(command)
        else:
            self.pub.send(command)
