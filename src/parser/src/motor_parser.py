#!/usr/bin/env python
import rospy
import serial
import sys

from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Imu

class MotorParser(object):
    def __init__(self):
        # get parameters
        SERIAL_PORT = rospy.get_param('port', '/dev/ttyACM0')
        SERIAL_BAUD = rospy.get_param('baud', 115200)

        # initialize attributes
        self.left_motor_vel = 0
        self.right_motor_vel = 0

        # configure and initialize serial port
        self.ser = serial.Serial()
        self.ser.baudrate = SERIAL_BAUD
        self.ser.port = SERIAL_PORT
        self.ser.open()

        # initialize ROS publishers and subscribers
        self.left_motor_vel_pub = rospy.Publisher('left_motor/fb/vel', Float32, queue_size=10)
        self.left_motor_enc_pub = rospy.Publisher('left_motor/fb/enc', Int32, queue_size=10)
        rospy.Subscriber("left_motor/cmd/vel", Float32, self.left_motor_vel_callback)

        self.right_motor_vel_pub = rospy.Publisher('right_motor/fb/vel', Float32, queue_size=10)
        self.right_motor_enc_pub = rospy.Publisher('right_motor/fb/enc', Int32, queue_size=10)
        rospy.Subscriber("right_motor/cmd/vel", Float32, self.right_motor_vel_callback)

        # self.imu_pub = rospy.Publisher('imu', Imu, queue_size=10)

        # start main loop
        rospy.init_node('motor_parser', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            try:
                msg = self.ser.readline()
                parsed = msg.decode("utf-8")
                vals = self.parse_message(parsed)

                self.left_motor_enc_pub.publish(int(vals[0]))
                self.right_motor_enc_pub.publish(int(vals[1]))
                self.left_motor_vel_pub.publish(float(vals[2]))
                self.right_motor_vel_pub.publish(float(vals[3]))

                # imu_msg = Imu()
                # imu_msg.orientation.w = float(vals[4])
                # imu_msg.orientation.x = float(vals[5])
                # imu_msg.orientation.y = float(vals[6])
                # imu_msg.orientation.z = float(vals[7])
                # self.imu_pub.publish(imu_msg)

                self.ser.write(b'%f,%f\n' % (self.left_motor_vel, self.right_motor_vel))
            except:
                continue
            rate.sleep()        

    # ROS CALLBACK FUNCTIONS

    def left_motor_vel_callback(self, msg):
        self.left_motor_vel = msg.data

    def right_motor_vel_callback(self, msg):
        self.right_motor_vel = msg.data

    # HELPER FUNCTIONS

    def parse_message(self, msg):
        vals = msg.strip().split(',')
        return vals

if __name__ == '__main__':
    try:
        MotorParser()
    except rospy.ROSInterruptException:
        pass