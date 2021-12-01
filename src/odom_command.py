#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import numpy as np
import time
import argparse
import math

if __name__ == '__main__':
    pub = rospy.Publisher("/joint_command", JointState, queue_size=10)

    rospy.init_node("odom_commands")
    
    joint_state = JointState()

    joint_state.name = [
        "odom_x_joint",
        "odom_y_joint",
        "odom_z_joint"
    ]

    num_joints = len(joint_state.name)
    joint_state.velocity = np.array([1.0] * num_joints)

    # position control the robot to wiggle around each joint
    time_start = time.time()
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        # joint_state.position = np.sin(time.time() - time_start) * (max_joints - min_joints) * 0.5 + default_joints
        joint_state.velocity = -joint_state.velocity
        pub.publish(joint_state)
        rate.sleep()
        