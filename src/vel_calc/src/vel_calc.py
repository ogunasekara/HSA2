#!/usr/bin/env python
# license removed for brevity
import time
import math
import rospy
from std_msgs.msg import Float32, Int32

WHEEL_DIAM = 0.152 # m
TICKS_PER_REV = 280
VEL_GAIN = 0.08

left_dist = 0.0
left_vel = 0.0
left_prev_time = None

right_dist = 0.0
right_vel = 0.0
right_prev_time = None

def left_enc_callback(msg):
    global left_dist, left_vel, left_prev_time

    new_dist = msg.data * (math.pi * WHEEL_DIAM) / TICKS_PER_REV

    if left_prev_time == None:
        left_prev_time = time.time()
        left_dist = new_dist
        return

    cur_time = time.time()
    dt = cur_time - left_prev_time
    left_prev_time = cur_time

    new_vel = (new_dist - left_dist) / dt
    left_vel = left_vel + (new_vel - left_vel) * VEL_GAIN
    left_dist = new_dist

def right_enc_callback(msg):
    global right_dist, right_vel, right_prev_time

    new_dist = msg.data * (math.pi * WHEEL_DIAM) / TICKS_PER_REV

    if right_prev_time == None:
        right_prev_time = time.time()
        right_dist = new_dist
        return

    cur_time = time.time()
    dt = cur_time - right_prev_time
    right_prev_time = cur_time

    new_vel = (new_dist - right_dist) / dt
    right_vel = right_vel + (new_vel - right_vel) * VEL_GAIN
    right_dist = new_dist

def main():
    global left_vel, right_vel

    left_vel_pub = rospy.Publisher('left_vel', Float32, queue_size=10)
    right_vel_pub = rospy.Publisher('right_vel', Float32, queue_size=10)

    rospy.init_node('vel_calc', anonymous=True)

    rospy.Subscriber("/left_encoder", Int32, left_enc_callback)
    rospy.Subscriber("/right_encoder", Int32, right_enc_callback)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        left_vel_pub.publish(left_vel)
        right_vel_pub.publish(right_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
