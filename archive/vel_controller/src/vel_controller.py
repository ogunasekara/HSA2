#!/usr/bin/env python
# license removed for brevity
import time
import math
import rospy
from std_msgs.msg import Float32, Int32

K_P = 140
K_I = 0.1
K_D = 1

left_vel = 0
left_cmd = 0
left_cmd_run = False
left_err_arr = [None, 0, 0] # prev_time, prev_err, err_sum

right_vel = 0
right_cmd = 0
right_cmd_run = False
right_err_arr = [None, 0, 0] # prev_time, prev_err, err_sum

def left_vel_callback(msg):
    global left_vel
    left_vel = msg.data

def right_vel_callback(msg):
    global right_vel
    right_vel = msg.data

def left_vel_cmd_callback(msg):
    global left_cmd, left_cmd_run, left_err_arr
    left_cmd = msg.data
    left_cmd_run = False
    left_err_arr[2] = 0

def right_vel_cmd_callback(msg):
    global right_cmd, right_cmd_run, right_err_arr
    right_cmd = msg.data
    right_cmd_run = False
    right_err_arr[2] = 0

def calculate_pwm(motor):
    if motor == 'left':
        global left_vel, left_cmd, left_err_arr, left_cmd_run

        err = left_cmd - left_vel

        if left_err_arr[0] == None:
            left_err_arr[0] = time.time()
            left_err_arr[1] = err

        base_val = left_cmd * 200

        if left_cmd_run == False:
            left_cmd_run = True
            return base_val

        cur_time = time.time()
        dt = cur_time - left_err_arr[0]

        d_err = (err - left_err_arr[1]) / dt
        left_err_arr[2] = left_err_arr[2] + err * dt

        pwm_val = base_val + K_P * err + K_I * left_err_arr[2] + K_D * d_err

        left_err_arr[0] = cur_time
        left_err_arr[1] = err

        return pwm_val

    elif motor == 'right':
        global right_vel, right_cmd, right_err_arr, right_cmd_run

        err = right_cmd - right_vel

        if right_err_arr[0] == None:
            right_err_arr[0] = time.time()
            right_err_arr[1] = err

        base_val = right_cmd * 200

        if right_cmd_run == False:
            right_cmd_run = True
            return base_val

        cur_time = time.time()
        dt = cur_time - right_err_arr[0]

        d_err = (err - right_err_arr[1]) / dt
        right_err_arr[2] = right_err_arr[2] + err * dt

        pwm_val = base_val + K_P * err + K_I * right_err_arr[2] + K_D * d_err

        right_err_arr[0] = cur_time
        right_err_arr[1] = err

        rospy.loginfo(right_vel)

        return pwm_val

def main():
    global left_vel, right_vel

    left_vel_pub = rospy.Publisher('left_motor', Int32, queue_size=10)
    right_vel_pub = rospy.Publisher('right_motor', Int32, queue_size=10)

    rospy.init_node('vel_controller', anonymous=True)

    rospy.Subscriber("/left_vel", Float32, left_vel_callback)
    rospy.Subscriber("/right_vel", Float32, right_vel_callback)
    rospy.Subscriber("/left_vel_cmd", Float32, left_vel_cmd_callback)
    rospy.Subscriber("/right_vel_cmd", Float32, right_vel_cmd_callback)

    rate = rospy.Rate(50) # 10hz

    while not rospy.is_shutdown():
        left_pwm = calculate_pwm('left')
        right_pwm = calculate_pwm('right')

        left_vel_pub.publish(left_pwm)
        right_vel_pub.publish(right_pwm)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
