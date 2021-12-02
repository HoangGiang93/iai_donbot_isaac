#! /usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import tf
from tf2_ros import TransformException
import math

class Odometry(object):
    def __init__(self):
        self.vel_pub = rospy.Publisher("/joint_command", JointState, queue_size=10)
        self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.vel_callback, queue_size=10)
        self.joint_state = JointState()
        self.joint_state.name = [
            "odom_x_joint",
            "odom_y_joint",
            "odom_z_joint"
        ]
        self.tf_listener = tf.TransformListener()

    def vel_callback(self, vel_msg):
        try:
            self.tf_listener.waitForTransform("odom", "base_footprint", rospy.Time(), rospy.Duration(1))
        except TransformException:
            rospy.logwarn("Odometry couldn't find map frame or base_footprint frame")
        else:
            t = self.tf_listener.getLatestCommonTime("odom", "base_footprint")
            _, quat = self.tf_listener.lookupTransform("odom", "base_footprint", t)
            (_, _, yaw) = euler_from_quaternion(quat)
            
            self.joint_state.velocity = [vel_msg.linear.x * math.cos(yaw) - vel_msg.linear.y * math.sin(yaw), vel_msg.linear.x * math.sin(yaw) + vel_msg.linear.y * math.cos(yaw), vel_msg.angular.z]
            self.vel_pub.publish(self.joint_state)

if __name__ == '__main__':
    rospy.init_node("odometry")

    # publish info to the console for the user
    rospy.loginfo("odometry starts")

    # start the base control
    Odometry()

    # keep it running
    rospy.spin()
        