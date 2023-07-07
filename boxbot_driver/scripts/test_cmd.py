#!/usr/bin/env python3

"""
  A very simple node for testing the boxbot driver.
  
  Copyright (c) 2020-2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


from sensor_msgs.msg import JointState
import rospy


def test_command():
    
    # Initialize node
    rospy.init_node("command_test_publisher", anonymous=True)
    
    # Get needed information
    topic = "/boxbot/left_arm/joint_commands"
    names = ["shoulder_1", "shoulder_2", "elbow", "wrist_1", "wrist_2", "wrist_3", "gripper"]
    goal_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #ENTER COMMANDS HERE
    
    # Setup publisher
    pub = rospy.Publisher(topic, JointState, queue_size = 1)
    
    # Build test message
    test_msg = JointState()
    test_msg.header.stamp = rospy.Time.now()
    test_msg.name = names
    test_msg.position = goal_pos
    
    # Publish test message
    i = 0
    while i < 10:
        pub.publish(test_msg)
        i += 1
        rospy.sleep(0.1)
        
        
if __name__ == '__main__':
    try:
        test_command()
    except rospy.ROSInterruptException:
        pass
