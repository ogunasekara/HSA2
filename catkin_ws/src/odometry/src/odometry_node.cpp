#include <geometry_msgs/Pose.h>
#include <odometry/Encoder.h>
#include <std_msgs/String.h>
#include <string.h>
#include <math.h>

// GLOBAL CONSTANTS
#define TICKS_PER_REV 1000
#define DIST_PER_REV 0.4785
#define WIDTH 10

// GLOBAL VARIABLES
int x, y, th;
int prev_enc_l, prev_enc_r;
int prev_time;
bool init = false;

void encCallback(const odometry::Encoder::ConstPtr& msg)
{
  // get encoder values
  int left_enc = msg.left_enc;
  int right_enc = msg.right_enc;
  float cur_time = msg.header.stamp.secs + (msg.header.stamp.nsecs/1E9);

  if(!init){
    // initialize globals
    x = 0;
    y = 0;
    th = 0;
    prev_enc_l = left_enc;
    prev_enc_r = right_enc;
    prev_time = cur_time;
    init = true;
  }
  else{
    float dt = cur_time - prev_time;
    float v_l = (((left_enc - prev_enc_l)/TICKS_PER_REV)*DIST_PER_REV)/dt;
    float v_r = (((right_enc - prev_enc_r)/TICKS_PER_REV)*DIST_PER_REV)/dt;
    float V = (v_r + v_l)/2.0;
    float w = (v_r - v_l)/WIDTH;
    th += w * dt;
    x += V * cos(th);
    y += V * sin(th);
    th += w * dt;
  }
}

int main(int argc, char **argv)
{
  // set up ROS
  ros::init(argc, argv, "odom");
  ros::NodeHandle n;

  // set up subscribers and publishers
  ros::Publisher odom_pub = n.advertise<geometry_msgs::Pose>("pose", 1000);
  ros::Subscriber enc_sub = n.subscribe<odometry::Encoder>("odom/pose", 1000, encCallback);
  ros::Rate loop_rate(10);

  while(ros::ok()){
    // create the pose message
    geometry_msgs::Pose msg;
    msg.position.x = x;
    msg.position.y = y;
    msg.orientation.z = th;

    // publish the message
    odom_pub.publish(msg);

    // spin and sleep
    ros::spinOnce();
    loop_rate.sleep();
  }
}