#!/usr/bin/env python
import rospy
import sys
import time

import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class BasePlanner(object):
    def __init__(self):
        # get parameters
        self.ang_vel_max = rospy.get_param('~ang_vel_max', 3)
        self.ang_acc = rospy.get_param('~ang_acc', 0.4)
        self.lin_vel_max = rospy.get_param('~lin_vel_max', 0.4)
        self.lin_acc = rospy.get_param('~lin_acc', 0.4)

        # initialize attributes
        self.state = [0, 0, 0]
        self.target = [0, 0, 0]
        self.L = 0.4064

        # initialize node
        rospy.init_node('base_planner', anonymous=True)

        # initialize ROS publishers and subscribers
        rospy.Subscriber('base/pose', Pose, self.pose_callback)
        rospy.Subscriber('base/target_pose', Pose, self.target_pose_callback)
        self.left_motor_vel_pub = rospy.Publisher('left_motor/cmd/vel', Float32, queue_size=10)
        self.right_motor_vel_pub = rospy.Publisher('right_motor/cmd/vel', Float32, queue_size=10)

        # start main loop
        rospy.spin()

    # ROS CALLBACK FUNCTIONS

    def pose_callback(self, msg):
        # extract state information
        self.state[0] = msg.position.x
        self.state[1] = msg.position.y

        quat = [0, 0, 0, 0]
        quat[0] = msg.orientation.x
        quat[1] = msg.orientation.y
        quat[2] = msg.orientation.z
        quat[3] = msg.orientation.w

        (_, _, yaw) = euler_from_quaternion(quat)
        self.state[2] = yaw

    def target_pose_callback(self, msg):
        # extract target information        
        self.target[0] = msg.position.x
        self.target[1] = msg.position.y

        quat = [0, 0, 0, 0]
        quat[0] = msg.orientation.x
        quat[1] = msg.orientation.y
        quat[2] = msg.orientation.z
        quat[3] = msg.orientation.w

        (_, _, yaw) = euler_from_quaternion(quat)
        self.target[2] = yaw

        # do pre-rotation
        point_ang = np.arctan2(self.target[1] - self.state[1], self.target[0] - self.state[0])
        ang = point_ang - self.state[2]
        rospy.loginfo("rotate %f radians\n" %(ang))
        self.rotate(ang, 0.05)

        # move in straight line
        dist = np.sqrt((self.state[1] - self.target[1])**2 + (self.state[0] - self.target[0])**2)
        rospy.loginfo("move straight %f m\n" %(dist))
        self.move_straight(dist, 0.05)
        
        # do post-rotation
        ang = self.target[2] - self.state[2]
        rospy.loginfo("rotate %f radians\n" %(ang))
        self.rotate(ang, 0.05)

    # HELPER FUNCTIONS

    def rotate(self, ang, dt):
        w_traj = self.trapezoidal_trajectory(ang, self.ang_vel_max, self.ang_acc, dt)
        
        counter = 0
        timer = time.time()

        while counter < len(w_traj):
            if (time.time() - timer) > dt:
                timer = time.time()
                v = 0
                w = w_traj[counter][1]
                self.send_vels(v, w)
                counter = counter + 1

    def move_straight(self, dist, dt):
        v_traj = self.trapezoidal_trajectory(dist, self.lin_vel_max, self.lin_acc, dt)
        
        counter = 0
        timer = time.time()

        while counter < len(v_traj):
            if (time.time() - timer) > dt:
                timer = time.time()
                v = v_traj[counter][1]
                w = 0
                self.send_vels(v, w)
                counter = counter + 1
        pass

    def trapezoidal_trajectory(self, d, max_vel, accel, dt):
        t_ramp = max_vel / accel
        t_end = 0

        if (d >= t_ramp * max_vel):
            t_end = (abs(d) / max_vel) + t_ramp
        else:
            t_end = 2 * np.sqrt(abs(d) / accel)

        traj = []

        for t in np.arange(0, t_end+1, dt):
            vel = self.trapezoidal_trajectory_step(t, d, max_vel, accel)
            traj.append((t, vel))

        return traj
            

    def trapezoidal_trajectory_step(self, t, d, max_vel, accel):
        t_ramp = max_vel / accel
        sign = 1
        if d < 0: 
            sign = -1

        if (d >= t_ramp * max_vel):
            t_end = (abs(d) / max_vel) + t_ramp

            if (t < t_ramp):
                return sign * t * accel
            elif (t < t_end - t_ramp):
                return sign * max_vel
            elif (t < t_end):
                return sign * (t_end - t) * accel
            else:
                return 0
        else:
            t_mid = np.sqrt(abs(d) / accel)
            t_end = 2 * t_mid

            if (t < t_mid):
                return sign * t * accel
            elif (t < t_end):
                return sign * (t_end - t) * accel
            else:
                return 0

    def send_vels(self, v, w):
        vl = (2.0 * v - self.L * w) / 2.0
        vr = (2.0 * v + self.L * w) / 2.0
        
        self.left_motor_vel_pub.publish(vl)
        self.right_motor_vel_pub.publish(vr)

if __name__ == '__main__':
    BasePlanner()