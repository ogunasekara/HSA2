#!/usr/bin/env python
import rospy
import tf2_ros
import sys
import time

import numpy as np

from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Pose, TransformStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class BaseLocalizer(object):
    def __init__(self):
        # get parameters
        self.WHEEL_RADIUS = rospy.get_param('wheel_radius', 0.075)
        self.WHEEL_BASE = rospy.get_param('wheel_base', 0.4064)
        self.ENC_TPR = rospy.get_param('tpr', 1120.0)

        # initialize attributes
        self.state = [0, 0, 0]
        self.ang_quat = [0, 0, 0, 0]
        self.cur_vel = np.array([None, None])

        # start main loop
        rospy.init_node('base_localizer', anonymous=True)

        # initialize ROS publishers and subscribers
        self.pose_pub = rospy.Publisher('base/pose', Pose, queue_size=10)

        rospy.Subscriber("left_motor/fb/vel", Float32, self.left_motor_vel_callback)
        rospy.Subscriber("right_motor/fb/vel", Float32, self.right_motor_vel_callback)

        while None in self.cur_vel:
            continue

        timer = time.time()

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # perform runge-kutta update
            dt = time.time() - timer
            self.runge_kutta_update(dt)
            timer = time.time()

            # publish pose
            pose_msg = Pose()
            pose_msg.position.x = self.state[0]
            pose_msg.position.y = self.state[1]
            pose_msg.position.z = 0
            pose_msg.orientation.x = self.ang_quat[0]
            pose_msg.orientation.y = self.ang_quat[1]
            pose_msg.orientation.z = self.ang_quat[2]
            pose_msg.orientation.w = self.ang_quat[3]

            self.pose_pub.publish(pose_msg)

            # send transform message to ROS
            broadcaster = tf2_ros.StaticTransformBroadcaster()
            tf_msg = TransformStamped()

            tf_msg.header.stamp = rospy.Time.now()
            tf_msg.header.frame_id = "odom"
            tf_msg.child_frame_id = "base_link"

            tf_msg.transform.translation.x = self.state[0]
            tf_msg.transform.translation.y = self.state[1]
            tf_msg.transform.translation.z = 0
            tf_msg.transform.rotation.x = self.ang_quat[0]
            tf_msg.transform.rotation.y = self.ang_quat[1]
            tf_msg.transform.rotation.z = self.ang_quat[2]
            tf_msg.transform.rotation.w = self.ang_quat[3]

            broadcaster.sendTransform(tf_msg)

            rate.sleep()

    # ROS CALLBACK FUNCTIONS

    def left_motor_vel_callback(self, msg):
        self.cur_vel[0] = msg.data

    def right_motor_vel_callback(self, msg):
        self.cur_vel[1] = msg.data

    # HELPER FUNCTIONS

    def runge_kutta_update(self, dt):
        v = (self.cur_vel[1] + self.cur_vel[0])/2.0
        w = (self.cur_vel[1] - self.cur_vel[0])/self.WHEEL_BASE

        th = self.state[2]

        k00 = v*np.cos(th)
        k01 = v*np.sin(th)
        k02 = w

        k10 = v*np.cos(th + 0.5 * dt * k02)
        k11 = v*np.sin(th + 0.5 * dt * k02)
        k12 = w
        
        k20 = v*np.cos(th + 0.5 * dt * k12)
        k21 = v*np.sin(th + 0.5 * dt * k12)
        k22 = w

        k30 = v*np.cos(th + dt * k22)
        k31 = v*np.sin(th + dt * k22)
        k32 = w

        self.state[0] = self.state[0] + dt * (1.0/6.0) * (k00 + 2*(k10 + k20) * k30)
        self.state[1] = self.state[1] + dt * (1.0/6.0) * (k01 + 2*(k11 + k21) * k31)
        self.state[2] = self.state[2] + dt * w
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))

        self.ang_quat = quaternion_from_euler(0, 0, self.state[2])

    # https://computergraphics.stackexchange.com/questions/8195/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
    def euler_to_quaternion(self, roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

if __name__ == '__main__':
    try:
        BaseLocalizer()
    except rospy.ROSInterruptException:
        pass