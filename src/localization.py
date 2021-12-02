#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped

def initialization():
  pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, latch=True, queue_size=1)
  msg = rospy.wait_for_message('/iai_donbot/joint_states', JointState)

  rospy.loginfo('Initialization is starting...')

  pose = PoseWithCovarianceStamped()

  pose.header.frame_id = 'map'

  pose.pose.pose.position.x = msg.position[0] - 3.3465 + 5
  pose.pose.pose.position.y = msg.position[1] - 4.5211 + 5
  q = quaternion_from_euler(0.0, 0.0, msg.position[2])

  pose.pose.pose.orientation.x = q[0]
  pose.pose.pose.orientation.y = q[1]
  pose.pose.pose.orientation.z = q[2]
  pose.pose.pose.orientation.w = q[3]

  pose.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  pub.publish(pose)

  rospy.loginfo('Initialization finished ')

def localization():
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  rate = rospy.Rate(10) # 1hz
  twist = Twist()
  rate.sleep()

  # rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
  rospy.loginfo('Localization is starting...')
  rospy.loginfo('Moving in x...')
  vel = 0.5
  twist.linear.x = vel
  for i in range(10):
    pub.publish(twist)
    rate.sleep()

  twist.linear.x = -vel
  for i in range(10):
    pub.publish(twist)
    rate.sleep()

  rospy.loginfo('Moving in y...')
  twist.linear.x = 0.0
  twist.linear.y = vel
  for i in range(10):
    pub.publish(twist)
    rate.sleep()

  twist.linear.y = -vel
  for i in range(10):
    pub.publish(twist)
    rate.sleep()

  rospy.loginfo('Rotating around...')
  twist.linear.y = 0.0
  twist.angular.z = vel
  for i in range(10):
    pub.publish(twist)
    rate.sleep()

  twist.angular.z = -vel
  for i in range(10):
    pub.publish(twist)
    rate.sleep()

  rospy.loginfo('Localization finished')
  twist.angular.z = 0.0
  pub.publish(twist)

if __name__ == "__main__":
  rospy.init_node('localization')
  initialization()
  localization()