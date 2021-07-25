#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import numpy as np
import time
import argparse
import math

if __name__ == '__main__':
    pub = rospy.Publisher("/joint_command", JointState, queue_size=10)

    rospy.init_node("joint_commands")
    
    joint_state = JointState()

    joint_state.name = [
        "ur5_shoulder_pan_joint",
        "ur5_shoulder_lift_joint",
        "ur5_elbow_joint",
        "ur5_wrist_1_joint",
        "ur5_wrist_2_joint",
        "ur5_wrist_3_joint"
    ]

    num_joints = len(joint_state.name)

    joint_state.position = np.array([0.0] * num_joints)
    default_joints = [0.0, -math.pi/2, 0.0, 0.0, 0.0, 0.0]

    # limiting the movements to a smaller range (this is not the range of the robot, just the range of the movement
    max_joints = np.array(default_joints) + 0.5
    min_joints = np.array(default_joints) - 0.5

    # position control the robot to wiggle around each joint
    time_start = time.time()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        joint_state.position = np.sin(time.time() - time_start) * (max_joints - min_joints) * 0.5 + default_joints
        pub.publish(joint_state)
        rate.sleep()