#!/usr/bin/env python

"""
  Publishes the Gazebo clock message from the simulation computer to 
  another computer (or series of computers).

  Copyright 2018-2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit history.
"""


import sys
import thread

from rosgraph_msgs.msg import Clock
import rospy

# Retrieve Rosbridge/Websocket utilities
file_dir = sys.path[0]
sys.path.append(file_dir + '/../../..')
from rss_git_lite.common import ws4pyRosMsgSrvFunctions_gen as ws4pyROS
from rss_git_lite.common import rosConnectWrapper as rC


class SimClockPublisher():
    """
    A ROS node to publish the values of the clock to a new computer
    """
    
    def __init__(self):
        
        # Initialize node
        rospy.init_node("clock_publisher", anonymous=True)
        
        # Initialize cleanup for this node
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Set the publishing rate (1 kHz)
        self.rate = rospy.Rate(1000.0)
        
        # Initialize variables
        self.clock_time = None
        
        # '/clock' Subscriber
        rospy.Subscriber("/clock", Clock, self.time_cb)
        
        # Setup ROSbridge publisher(s)
        self.pubs = []
        for name in rospy.get_param("~/connections", dict()).keys():
            ip = rospy.get_param("~/connections/" + name)
            if ip == "local":
                pass
            else:
                self.pubs.append(rC.RosMsg("ws4py", ip, "pub", "/clock", 
                                           "rosgraph_msgs/Clock",
                                           pack_time))
                                           
        # Make sure there are actual publishing needs, or else shut down
        if len(self.pubs) == 0:
            rospy.logwarn("All clock publish destinations are local. Clock publisher not needed")
            exit()
        
        # Run publisher
        rospy.loginfo("Simulation clock publisher initialized")
        self.main()
        
    def time_cb(self, msg):
        """
        Callback for Gazebo/clock.
        """
        
        self.clock_time = msg
        
    def copy_and_clear_received(self):
        """
        Clear out received data once retrieved/pulled.
        """
        
        # Send the clock time and clear the state variable
        self.lock.acquire()
        clock_current = self.clock_time
        self.clock_time = None
        self.lock.release()
        
        return clock_current
        
    def pack_time(time):
        """
        Unpackage 'rosgraph_msgs/Clock' message.
        """
        
        # Get time data
        data = time.clock
        seconds = data.secs
        nanoseconds = data.nsecs
        
        # Place data into dictionary
        time_msg = {"clock": {"secs": seconds,
                              "nsecs": nanoseconds}}
                
        return time_msg
    
    def main(self):
        """
        Retrieve and publish the clock time.
        """
        
        while not rospy.is_shutdown():
            
            # Check for new time message
            clock_now = self.copy_and_clear_received()
            
            # If there is a new message publish it to all destinations
            if (clock_now==None):
                pass
            else:
                for pub in self.pubs:
                    pub.send(clock_now)
                    
            self.rate.sleep()
    
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown
        rospy.loginfo("Shutting down 'clock_publisher'...")
        rospy.sleep(1)


if __name__ == "__main__":
    try:
        SimClockPublisher()
    except rospy.ROSInterruptException:
        pass
        
    sys.exit(0)
