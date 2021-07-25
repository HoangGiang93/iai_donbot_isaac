#! /usr/bin/env python

from isaac_ros_messages.srv import IsaacPose, IsaacPoseRequest
from geometry_msgs.msg import Pose, Twist, Vector3
import rospy
import numpy as np


def send_pose_cube_client(new_pose):
    rospy.wait_for_service("/teleport_pos")
    try:
        send_pose = rospy.ServiceProxy("/teleport_pos", IsaacPose)
        send_pose(new_pose)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def compose_pose(pos_vec, quat_vec):
    obj_pose = Pose()
    obj_pose.position.x = pos_vec[0]
    obj_pose.position.y = pos_vec[1]
    obj_pose.position.z = pos_vec[2]
    obj_pose.orientation.w = quat_vec[0]
    obj_pose.orientation.x = quat_vec[1]
    obj_pose.orientation.y = quat_vec[2]
    obj_pose.orientation.z = quat_vec[3]
    return obj_pose


def compose_twist(lx, ly, lz, ax, ay, az):
    obj_twist = Twist()
    obj_twist.linear.x = lx
    obj_twist.linear.y = ly
    obj_twist.linear.z = lz
    obj_twist.angular.x = ax
    obj_twist.angular.y = ay
    obj_twist.angular.z = az
    return obj_twist

if __name__ == "__main__":
    rospy.init_node("teleport_commands", anonymous=True)
    new_isaac_pose = IsaacPoseRequest()
    new_isaac_pose.names = ["/iai_donbot"]

    pos_vec = np.array([0.0, 0.0, 0.0])
    quat_vec = np.array([1, 0.0, 0.0, 0.0])

    rate = rospy.Rate(1)  # hz

    while not rospy.is_shutdown():
        # new random pose
        pos_vec = np.random.rand(3) * 0.1
        pose = compose_pose(pos_vec, quat_vec)
        new_isaac_pose.poses = [pose]
        # publish
        send_pose_cube_client(new_isaac_pose)
        rate.sleep()
