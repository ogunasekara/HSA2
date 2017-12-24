#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <string.h>

// GLOBAL VARIABLES
int x, y, th;
int prev_enc_l, prev_enc_r;
bool init;

void encCallback(const std_msgs::String::ConstPtr& msg)
{
  char *charBuf = msg.data.c_str();
  const char s[2] = "/";

  // get encoder values
  int left_enc = atoi(strtok(charBuf, s));
  int right_enc = atoi(strtok(NULL, s));

  if(!init){
    prev_enc_l = left_enc;
    prev_enc_r = right_enc;
    init = true;
  }
  else{
    
  }
}

int main(int argc, char **argv)
{
  // initialize globals
  init = false;
  x = 0;
  y = 0;
  th = 0;

  // set up ROS
  ros::init(argc, argv, "odom");
  ros::NodeHandle n;

  // set up subscribers and publishers
  ros::Publisher odom_pub = n.advertise<geometry_msgs::Pose>("pose", 1000);
  ros::Subscriber enc_sub = n.subscribe<std_msgs::String>("odom/pose", 1000, encCallback);
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