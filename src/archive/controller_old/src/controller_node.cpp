#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <math.h>

// Designed exclusively for Logitech Gamepad F310

#define FORWARD 30
#define BACKWARD -30
#define STOP 0

int left_steer = STOP;
int right_steer = STOP;

void joyCallback(const sensor_msgs::Joy& joy_msg) {
  if (joy_msg.buttons[0] == 1) {
    left_steer = FORWARD;
    right_steer = FORWARD;
  } else if (joy_msg.buttons[1] == 1) {
    left_steer = BACKWARD;
    right_steer = BACKWARD;
  } else if (joy_msg.buttons[4] == 1) {
    right_steer = FORWARD;
    left_steer = BACKWARD;
  } else if (joy_msg.buttons[5] == 1){
    right_steer = BACKWARD;
    left_steer = FORWARD;
  } else {
    right_steer = STOP;
    left_steer = STOP;
  }
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "steering_controller");

    ros::NodeHandle node;

    ros::Subscriber joySub = node.subscribe("/joy", 10, &joyCallback);
    ros::Publisher rightSteerPub = node.advertise<std_msgs::Int32>("/right_motor", 10);
    ros::Publisher leftSteerPub = node.advertise<std_msgs::Int32>("/left_motor", 10);

    ros::Rate loop_rate(10);

    std_msgs::Int32 left_msg;
    std_msgs::Int32 right_msg;

    while (node.ok())
    {
        left_msg.data = left_steer;
        right_msg.data = right_steer;

        rightSteerPub.publish(right_msg);
        leftSteerPub.publish(left_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
