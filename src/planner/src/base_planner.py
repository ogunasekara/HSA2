#!/usr/bin/env python
import rospy
import sys

import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

class BasePlanner(object):
    def __init__(self):
        # get parameters
        

        # initialize attributes
        self.state = []        

        # initialize ROS publishers and subscribers
        rospy.Subscriber('base/pose', Pose, self.pose_callback)
        self.motor1_vel_pub = rospy.Publisher('motor1/cmd/vel', Float32, queue_size=10)
        self.motor2_vel_pub = rospy.Publisher('motor2/cmd/vel', Float32, queue_size=10)

        # start main loop
        rospy.init_node('base_planner', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            try:
                pass
            except:
                continue

            rate.sleep()

    # ROS CALLBACK FUNCTIONS

    def pose_callback(self, msg):
        self.state[0] = msg.position.x
        self.state[1] = msg.position.y

        quat = [0, 0, 0, 0]
        quat[0] = msg.orientation.x
        quat[1] = msg.orientation.y
        quat[2] = msg.orientation.z
        quat[3] = msg.orientation.w

        eul = self.quaternion_to_euler(quat)
        self.state[2] = eul[0]
        pass

    # HELPER FUNCTIONS

    def quaternion_to_euler(self, quat):
        x = quat[0]
        y = quat[1]
        z = quat[2]
        w = quat[3]

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.atan2(t3, t4)
        return [yaw, pitch, roll]


if __name__ == '__main__':
    BasePlanner()