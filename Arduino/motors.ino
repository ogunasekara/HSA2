#include <ros.h>
#include <std_msgs/String.h>
#include <string.h>

// MACRO DEFINITIONS
#define PWM_L 10
#define DIR_L 8
// #define ENCA_L 2
// #define ENCB_L
#define PWM_R 11
#define DIR_R 7
// #define ENCA_R 3
// #define ENCB_R

// GLOBAL VARIABLES
ros::NodeHandle nh;

char c;
int velL, velR;
bool errFlag;

void motorCallback(const std_msgs::String& msg){
  std::string buf = msg.data;
  if(buf[0] == 'L'){

  }
  else if(buf[0] == 'R'){

  }
}

ros::Subscriber<std_msgs::String> lmotor_sub("left_motor", &motorCallback);
ros::Subscriber<std_msgs::String> rmotor_sub("right_motor", &motorCallback);

// HELPER FUNCTIONS
void sendVelocity(int velL, int velR){
  // bound velocities between -255 and 255
  if(abs(velL) > 255){
    velL = 255 * (velL/abs(velL));
  }
  if(abs(velR) > 255){
    velR = 255 * (velR/abs(velR));
  }

  // set direction of left wheel
  if(velL < 0){
    digitalWrite(DIR_L, HIGH);
  }
  else{
    digitalWrite(DIR_L, LOW);
  }

  // set direction of right wheel
  if(velR < 0){
    digitalWrite(DIR_R, HIGH);
  }
  else{
    digitalWrite(DIR_R, LOW);
  }

  // send PWM signals to left and right motors
  analogWrite(PWM_L, abs(velL));
  analogWrite(PWM_R, abs(velR));
}

// MAIN FUNCTIONS
void setup(){
  // Initialize motor controller pins
  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);

  // Initialize encoder pins
  // pinMode(ENCA_L, INPUT);
  // pinMode(ENCB_L, INPUT);
  // pinMode(ENCA_R, INPUT);
  // pinMode(ENCB_R, INPUT);
  // digitalWrite(ENCA_L, LOW);
  // digitalWrite(ENCA_R, LOW);

  // Initialize ROS
  nh.initNode();
  nh.subscribe(lmotor_sub);
  nh.subscribe(rmotor_sub);
}

void loop(){
  // if(Serial.available() > 0){
  //   // reset errFlag
  //   errFlag = false;
  //
  //   // only read if first character is not \n
  //   while((c = Serial.read()) != '\n'){
  //     // check which motor is being actuated
  //     if(c == 'L'){
  //       velL = Serial.parseInt();
  //     }
  //     else if(c == 'R'){
  //       velR = Serial.parseInt();
  //     }
  //     else{
  //       errFlag = true;
  //     }
  //   }
  //
  //   // send recieved velocities if no error
  //   if(!errFlag){
  //     sendVelocity(velL, velR);
  //   }
  // }
  delay(1);
  nh.spinOnce();
}