#! /usr/bin/python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from owl_client import OwlClient
import time
if __name__ == "__main__":
    rospy.init_node('owl_ping')
    robot_ip = "10.42.0.53"
    client = OwlClient(robot_ip)
    while not rospy.is_shutdown():
        # degree_val = [0, 0, 0, 0, 0, 0]
        # joint_values = np.deg2rad(degree_val)
        joint_values = client.get_joint().get_joints() # getting joint values of real robot
        joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        joint_state = JointState()
        joint_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        joint_state.position = joint_values
        joint_state.header.stamp = rospy.get_rostime()
        joint_state_pub.publish(joint_state)
        print(joint_state)