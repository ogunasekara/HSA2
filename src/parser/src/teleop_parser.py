#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

class TeleopParser(object):
    def __init__(self):
        self.v = 0
        self.w = 0
        self.L = 0.338

        self.left_motor_vel_pub = rospy.Publisher('left_motor/cmd/vel', Float32, queue_size=10)
        self.right_motor_vel_pub = rospy.Publisher('right_motor/cmd/vel', Float32, queue_size=10)
        rospy.Subscriber("joy", Joy, self.joy_callback)
        
        rospy.init_node('teleop_parser', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            vels = self.convert_vels()
            self.left_motor_vel_pub.publish(vels[0])
            self.right_motor_vel_pub.publish(vels[1])
            rate.sleep()

    def joy_callback(self, msg):
        if msg.buttons[0] == 1:
            self.v = 0.2
        elif msg.buttons[1] == 1:
            self.v = -0.2
        else:
            self.v = 0

        if abs(msg.axes[0]) > 0.05:
            self.w = msg.axes[0] * 0.5
        else:
            self.w = 0
        
    def convert_vels(self):
        vl = (2.0 * self.v - self.L * self.w) / 2.0
        vr = (2.0 * self.v + self.L * self.w) / 2.0
        return [vl, vr]


if __name__ == '__main__':
    try:
        TeleopParser()
    except rospy.ROSInterruptException:
        pass