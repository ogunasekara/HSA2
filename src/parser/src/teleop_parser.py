#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Joy

class TeleopParser(object):
    def __init__(self):
        # get parameters
        self.lin_vel_max = rospy.get_param('lin_vel_max', 0.4)
        self.ang_vel_scale = rospy.get_param('ang_vel_scale', 2.5)
        
        self.v = 0
        self.w = 0
        self.L = 0.3937

        rospy.init_node('teleop_parser', anonymous=True)
        
        self.left_motor_vel_pub = rospy.Publisher('left_motor/cmd/vel', Float32, queue_size=10)
        self.right_motor_vel_pub = rospy.Publisher('right_motor/cmd/vel', Float32, queue_size=10)
        rospy.Subscriber("joy", Joy, self.joy_callback)
        
        rospy.spin()

    def joy_callback(self, msg):
        if (msg.axes[5] < 1.0):
            self.v = self.lin_vel_max * (msg.axes[5] - 1.0)/(-2.0)
        else:
            self.v = self.lin_vel_max * (msg.axes[2] - 1.0)/(2.0)

        # if msg.buttons[0] == 1:
        #     self.v = self.lin_vel_max
        # elif msg.buttons[1] == 1:
        #     self.v = -self.lin_vel_max
        # else:
        #     self.v = 0

        if abs(msg.axes[0]) > 0.2:
            self.w = msg.axes[0] * self.ang_vel_scale
        else:
            self.w = 0

        vels = self.convert_vels()
        
        self.right_motor_vel_pub.publish(vels[1])
        self.left_motor_vel_pub.publish(vels[0])

        # motor_str = "L%f,R%f" %(vels[1], vels[0])
        # self.motor_vel_pub.publish(motor_str)
        
    def convert_vels(self):
        vl = (2.0 * self.v - self.L * self.w) / 2.0
        vr = (2.0 * self.v + self.L * self.w) / 2.0
        return [vl, vr]


if __name__ == '__main__':
    try:
        TeleopParser()
    except rospy.ROSInterruptException:
        pass
