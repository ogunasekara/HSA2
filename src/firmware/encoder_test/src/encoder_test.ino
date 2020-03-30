#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>

// MACRO DEFINITIONS
#define PWM_L 10
#define DIR_L 9
#define ENCA_L 2
#define ENCB_L 5

#define PWM_R 11
#define DIR_R 8
#define ENCA_R 3
#define ENCB_R 4

#define WHEEL_DIAM 152 // mm
#define TICKS_PER_REV 280 // from datasheet

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
    digitalWrite(DIR_R, LOW);
  }
  else{
    digitalWrite(DIR_R, HIGH);
  }

  // send PWM signals to right motor
  analogWrite(PWM_R, abs(velR));
}

void leftEncEvent(){
  if (digitalRead(ENCB_L) == LOW) {
    leftCount++;
  } else {
    leftCount--;
  }
}

void rightEncEvent(){
  if (digitalRead(ENCB_R) == LOW) {
    rightCount--;
  } else {
    rightCount++;
  }
}

std_msgs::Int32 left_enc_msg;
std_msgs::Int32 right_enc_msg;
ros::Publisher left_enc_pub("left_encoder", &left_enc_msg);
ros::Publisher right_enc_pub("right_encoder", &right_enc_msg);
ros::Subscriber<std_msgs::Int32> lmotor_sub("left_motor", &leftMotorCallback);
ros::Subscriber<std_msgs::Int32> rmotor_sub("right_motor", &rightMotorCallback);

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

  // initialize hardware interrupts
  attachInterrupt(0, leftEncEvent, RISING);
  attachInterrupt(1, rightEncEvent, RISING);

  // Initialize ROS
  nh.initNode();
  nh.advertise(left_enc_pub);
  nh.advertise(right_enc_pub);
  nh.subscribe(lmotor_sub);
  nh.subscribe(rmotor_sub);

  while (!nh.connected())
    nh.spinOnce();
}

void loop(){
  left_enc_msg.data = leftCount;
  right_enc_msg.data = rightCount;

  left_enc_pub.publish(&left_enc_msg);
  right_enc_pub.publish(&right_enc_msg);

  delay(1);
  nh.spinOnce();
}
