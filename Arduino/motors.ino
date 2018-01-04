#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <odometry/Encoder.h>

// MACRO DEFINITIONS
#define PWM_L 10
#define DIR_L 8
#define ENCA_L 2
#define ENCB_L 4
#define PWM_R 11
#define DIR_R 7
#define ENCA_R 3
#define ENCB_R 5

// GLOBAL VARIABLES
ros::NodeHandle nh;
volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;

void leftMotorCallback(const std_msgs::Int32& msg){
  // extract left velocity from msg
  int velL = msg.data;

  // bound velocity between -255 and 255
  if(abs(velL) > 255){
    velL = 255 * (velL/abs(velL));
  }

  // set direction of left wheel
  if(velL < 0){
    digitalWrite(DIR_L, HIGH);
  }
  else{
    digitalWrite(DIR_L, LOW);
  }

  // send PWM signals to left motor
  analogWrite(PWM_L, abs(velL));
}

void rightMotorCallback(const std_msgs::Int32& msg){
  // extract right velocity from msg
  int velR = msg.data;

  // bound velocity between -255 and 255
  if(abs(velR) > 255){
    velR = 255 * (velR/abs(velR));
  }

  // set direction of right wheel
  if(velR < 0){
    digitalWrite(DIR_R, HIGH);
  }
  else{
    digitalWrite(DIR_R, LOW);
  }

  // send PWM signals to right motor
  analogWrite(PWM_R, abs(velR));
}

void leftEncEvent(){
  if (digitalRead(ENCA_L) == HIGH) {
    if (digitalRead(ENCB_L) == LOW) {
      leftCount++;
    } else {
      leftCount--;
    }
  } else {
    if (digitalRead(ENCB_L) == LOW) {
      leftCount--;
    } else {
      leftCount++;
    }
  }
}

void rightEncEvent(){
  if (digitalRead(ENCA_R) == HIGH) {
    if (digitalRead(ENCB_R) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(ENCB_R) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  }
}

ros::Subscriber<std_msgs::Int32> lmotor_sub("left_motor", &leftMotorCallback);
ros::Subscriber<std_msgs::Int32> rmotor_sub("right_motor", &rightMotorCallback);

odometry::Encoder enc_msg;
ros::Publisher enc_pub("encoders", &enc_msg);

// MAIN FUNCTIONS
void setup(){
  // Initialize motor controller pins
  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);

  // Initialize encoder pins
  pinMode(ENCA_L, INPUT);
  pinMode(ENCB_L, INPUT);
  pinMode(ENCA_R, INPUT);
  pinMode(ENCB_R, INPUT);
  digitalWrite(ENCA_L, LOW);
  digitalWrite(ENCA_R, LOW);

  // initialize hardware interrupts
  attachInterrupt(0, leftEncEvent, CHANGE);
  attachInterrupt(1, rightEncEvent, CHANGE);

  // Initialize ROS
  nh.initNode();
  nh.subscribe(lmotor_sub);
  nh.subscribe(rmotor_sub);
  nh.advertise(enc_pub);
}

void loop(){
  enc_msg.header.stamp = nh.now();
  enc_msg.left_enc = leftCount;
  enc_msg.right_enc = rightCount;
  enc_pub.publish(&enc_msg);

  delay(1);
  nh.spinOnce();
}