#!/usr/bin/env python
import rospy
import sys

import numpy as np

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class CentralPlanner(object):
    def __init__(self):
        # get parameters

        # initialize attributes
        self.done = False

        # initialize node
        rospy.init_node('central_planner', anonymous=True)

        # initialize ROS publishers and subscribers
        rospy.Subscriber('base/done', Bool, self.base_done_callback)
        self.target_pose_pub = rospy.Publisher('base/target_pose', Pose, queue_size=10)

        self.go_to_pose(1, 0, 0)
        self.go_to_pose(1, 1, np.pi/2)
        self.go_to_pose(0, 1, np.pi)
        self.go_to_pose(0, 0, -np.pi/2)

        rospy.spin() 

    # ROS CALLBACK FUNCTIONS

    def base_done_callback(self, msg):
        self.done = msg.data

    # HELPER FUNCTIONS

    def go_to_pose(self, x, y, th):
        while not self.done:
            continue

        self.done = False

        th = np.arctan2(np.sin(th), np.cos(th))
        ang_quat = quaternion_from_euler(0, 0, th)

        # publish pose
        pose_msg = Pose()
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = 0
        pose_msg.orientation.x = ang_quat[0]
        pose_msg.orientation.y = ang_quat[1]
        pose_msg.orientation.z = ang_quat[2]

        self.target_pose_pub.publish(pose_msg)

if __name__ == '__main__':
    try:
        CentralPlanner()
    except rospy.ROSInterruptException:
        pass