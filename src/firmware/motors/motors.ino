#include <Encoder.h>

// MACRO DEFINITIONS
#define RIGHT_WHEEL_PWM 6
#define RIGHT_WHEEL_DIR 12
#define RIGHT_WHEEL_ENCA 2
#define RIGHT_WHEEL_ENCB 7

#define LEFT_WHEEL_PWM 5
#define LEFT_WHEEL_DIR 11
#define LEFT_WHEEL_ENCA 3
#define LEFT_WHEEL_ENCB 4

#define TICKS_PER_REV 1180.0 // ppr of motor encoder
#define WHEEL_RADIUS 0.075 // radius of attached wheel (m)

// PID Constants
float left_motor_p = 80;
float left_motor_i = 0;
float left_motor_d = 0;
float left_motor_punch = 0;
float left_motor_deadzone = 0;
float left_motor_i_clamp = 0;
float left_motor_out_clamp = 255;
float left_motor_last, left_motor_accumulate;

float right_motor_p = 80;
float right_motor_i = 0;
float right_motor_d = 0;
float right_motor_punch = 0;
float right_motor_deadzone = 0;
float right_motor_i_clamp = 0;
float right_motor_out_clamp = 255;
float right_motor_last, right_motor_accumulate;

// Motor Variables
Encoder left_enc(LEFT_WHEEL_ENCA, LEFT_WHEEL_ENCB);
Encoder right_enc(RIGHT_WHEEL_ENCA, RIGHT_WHEEL_ENCB);

int left_enc_cur_ticks = 0;
int left_enc_prev_ticks = 0;
int left_motor_command = 0;
float left_motor_desired_speed = 0;
float left_motor_cur_speed = 0;

int right_enc_cur_ticks = 0;
int right_enc_prev_ticks = 0;
int right_motor_command = 0;
float right_motor_desired_speed = 0;
float right_motor_cur_speed = 0;

// Timer variables
int timer1_freq = 10; 
int timer1_counter;
unsigned long cur_time;
 
// speed should be a number from -255 to 255
void setMotorSpeed(int vel, int motor)
{
    int DIR_PIN = LEFT_WHEEL_DIR;
    int PWM_PIN = LEFT_WHEEL_PWM;
    
    if (motor == 2){
        DIR_PIN = RIGHT_WHEEL_DIR;
        PWM_PIN = RIGHT_WHEEL_PWM;
        vel = -vel;
    }

    // set direction of left wheel
    if(vel < 0){
        digitalWrite(DIR_PIN, HIGH);
    }
    else{
        digitalWrite(DIR_PIN, LOW);
    }

    // send PWM signals to motor
    analogWrite(PWM_PIN, abs(vel));
}

void checkEncoderLeft(){
    if (digitalRead(LEFT_WHEEL_ENCB) == LOW) {
      left_enc_cur_ticks += 1;
    } else {
      left_enc_cur_ticks -= 1;
    }
}

void checkEncoderRight(){
    if (digitalRead(RIGHT_WHEEL_ENCB) == LOW) {
      right_enc_cur_ticks -= 1;
    } else {
      right_enc_cur_ticks += 1;
    }
}
 
// ISR for timer1
// calculate velocity and perform PID update @ 10 Hz
ISR(TIMER1_OVF_vect)
{
  TCNT1 = timer1_counter;   // reset timer

  left_enc_cur_ticks = -left_enc.read();
  right_enc_cur_ticks = right_enc.read();

  int left_enc_diff = left_enc_cur_ticks - left_enc_prev_ticks;
  int right_enc_diff = right_enc_cur_ticks - right_enc_prev_ticks;

  float k = 0.9;

  left_motor_cur_speed += k * (2 * PI * WHEEL_RADIUS * (left_enc_diff / TICKS_PER_REV) / 0.1 - left_motor_cur_speed);
  right_motor_cur_speed += k * (2 * PI * WHEEL_RADIUS * (right_enc_diff / TICKS_PER_REV) / 0.1 - right_motor_cur_speed);

  left_enc_prev_ticks = left_enc_cur_ticks;
  right_enc_prev_ticks = right_enc_cur_ticks;

  if (left_motor_desired_speed == 0){
    left_motor_command = 0;
  }
  else {
    int left_motor_pid =  pid(left_motor_desired_speed, left_motor_cur_speed, 
                    left_motor_p, left_motor_i, left_motor_d, 
                    left_motor_i_clamp, left_motor_out_clamp, left_motor_punch, left_motor_deadzone, 
                    &left_motor_last, &left_motor_accumulate);  
                    
    left_motor_command += left_motor_pid;
  }

  if (right_motor_desired_speed == 0){
    right_motor_command = 0;
  }
  else{
    int right_motor_pid =  pid(right_motor_desired_speed, right_motor_cur_speed, 
                    right_motor_p, right_motor_i, right_motor_d, 
                    right_motor_i_clamp, right_motor_out_clamp, right_motor_punch, right_motor_deadzone, 
                    &right_motor_last, &right_motor_accumulate);
                    
    right_motor_command += right_motor_pid;
  }

  setMotorSpeed(left_motor_command, 1);
  setMotorSpeed(right_motor_command, 2);

  Serial.print(left_enc_cur_ticks);
  Serial.print(",");
  Serial.print(right_enc_cur_ticks);
  Serial.print(",");
  Serial.print(left_motor_cur_speed);
  Serial.print(",");
  Serial.print(right_motor_cur_speed);
//  Serial.print(",");
//  Serial.print(left_motor_command);
//  Serial.print(",");
//  Serial.print(right_motor_command);
  Serial.print("\n");
}

void setup()
{
  pinMode(LEFT_WHEEL_PWM, OUTPUT);
  pinMode(LEFT_WHEEL_DIR, OUTPUT);
  pinMode(RIGHT_WHEEL_PWM, OUTPUT);
  pinMode(RIGHT_WHEEL_DIR, OUTPUT);
  
  pinMode(LEFT_WHEEL_ENCA, INPUT);
  pinMode(LEFT_WHEEL_ENCB, INPUT);
  pinMode(RIGHT_WHEEL_ENCA, INPUT);
  pinMode(RIGHT_WHEEL_ENCB, INPUT);

  digitalWrite(LEFT_WHEEL_ENCA, LOW);
  digitalWrite(RIGHT_WHEEL_ENCA, LOW);

  // initialize Serial w/ baud 115200
  Serial.begin(115200);
  Serial.println("Initializing...");

//  attachInterrupt(digitalPinToInterrupt(LEFT_WHEEL_ENCA), checkEncoderLeft, RISING); 
//  attachInterrupt(digitalPinToInterrupt(RIGHT_WHEEL_ENCA), checkEncoderRight, RISING);

  // setup timers for velocity calculation
  cli();//stop interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  timer1_counter = 65536 - (16000000 / 256 / timer1_freq);   // preload timer 65536 - 16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt

  sei();//allow interrupts

  cur_time = millis();  
}
 
void loop()
{    
  while(Serial.available() > 0){
    
    char r = Serial.read();
    if(r == 'L'){
      left_motor_desired_speed = Serial.parseFloat();
    }
    if(r == 'R'){
      right_motor_desired_speed = Serial.parseFloat();
    }
    if(r == '\n'){}

  }  
}
