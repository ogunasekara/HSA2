#!/usr/bin/env python
import rospy
import tf2_ros
import sys

import numpy as np

from std_msgs.msg import Int32
from geometry_msgs.msg import Pose, TransformStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class BaseLocalizer(object):
    def __init__(self):
        # get parameters
        self.WHEEL_RADIUS = rospy.get_param('wheel_radius', 0.075)
        self.WHEEL_BASE = rospy.get_param('wheel_base', 0.3937)
        self.ENC_TPR = rospy.get_param('tpr', 1180.0)

        # initialize attributes
        self.state = [0, 0, 0]
        self.ang_quat = [0, 0, 0, 0]
        self.prev_enc = np.array([0, 0])
        self.cur_enc = np.array([None, None])

        # start main loop
        rospy.init_node('base_localizer', anonymous=True)

        # initialize ROS publishers and subscribers
        self.pose_pub = rospy.Publisher('base/pose', Pose, queue_size=10)
        rospy.Subscriber("left_motor/fb/enc", Int32, self.left_motor_enc_callback)
        rospy.Subscriber("right_motor/fb/enc", Int32, self.right_motor_enc_callback)

        rospy.loginfo(self.WHEEL_BASE)

        while None in self.cur_enc:
            continue

        self.prev_enc[0] = self.cur_enc[0]
        self.prev_enc[1] = self.cur_enc[1]

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            enc_diff = np.array(self.cur_enc) - np.array(self.prev_enc)

            # rospy.loginfo(enc_diff)
            
            self.prev_enc[0] = self.cur_enc[0]
            self.prev_enc[1] = self.cur_enc[1]

            self.runge_kutta_update(enc_diff)

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

    def left_motor_enc_callback(self, msg):
        self.cur_enc[0] = msg.data

    def right_motor_enc_callback(self, msg):
        self.cur_enc[1] = msg.data

    # HELPER FUNCTIONS

    def runge_kutta_update(self, enc_diff):
        dist_diff = enc_diff * (2.0 * np.pi * self.WHEEL_RADIUS / self.ENC_TPR)

        v = (dist_diff[1] + dist_diff[0])/2.0
        w = (dist_diff[1] - dist_diff[0])/self.WHEEL_BASE

        th = self.state[2]

        k00 = v*np.cos(th)
        k01 = v*np.sin(th)
        k02 = w

        k10 = v*np.cos(th + 0.5 * k02)
        k11 = v*np.sin(th + 0.5 * k02)
        k12 = w
        
        k20 = v*np.cos(th + 0.5 * k12)
        k21 = v*np.sin(th + 0.5 * k12)
        k22 = w

        k30 = v*np.cos(th + k22)
        k31 = v*np.sin(th + k22)
        k32 = w

        self.state[0] = self.state[0] + (1.0/6.0) * (k00 + 2*(k10 + k20) * k30)
        self.state[1] = self.state[1] + (1.0/6.0) * (k01 + 2*(k11 + k21) * k31)
        self.state[2] = self.state[2] + w
        self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))

        rospy.loginfo(self.state)

        self.ang_quat = quaternion_from_euler(0, 0, self.state[2])

    # https://computergraphics.stackexchange.com/questions/8195/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
    def euler_to_quaternion(self, roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

if __name__ == '__main__':
    BaseLocalizer()