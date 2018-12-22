#ifndef _ROS_robot_msgs_RobotCommand_h
#define _ROS_robot_msgs_RobotCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robot_msgs
{

  class RobotCommand : public ros::Msg
  {
    public:
      typedef uint32_t _forward_type;
      _forward_type forward;
      typedef uint32_t _backward_type;
      _backward_type backward;
      typedef uint32_t _strafe_left_type;
      _strafe_left_type strafe_left;
      typedef uint32_t _strafe_right_type;
      _strafe_right_type strafe_right;
      typedef uint32_t _turn_left_type;
      _turn_left_type turn_left;
      typedef uint32_t _turn_right_type;
      _turn_right_type turn_right;

    RobotCommand():
      forward(0),
      backward(0),
      strafe_left(0),
      strafe_right(0),
      turn_left(0),
      turn_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->forward >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->forward >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->forward >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->forward >> (8 * 3)) & 0xFF;
      offset += sizeof(this->forward);
      *(outbuffer + offset + 0) = (this->backward >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->backward >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->backward >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->backward >> (8 * 3)) & 0xFF;
      offset += sizeof(this->backward);
      *(outbuffer + offset + 0) = (this->strafe_left >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->strafe_left >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->strafe_left >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->strafe_left >> (8 * 3)) & 0xFF;
      offset += sizeof(this->strafe_left);
      *(outbuffer + offset + 0) = (this->strafe_right >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->strafe_right >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->strafe_right >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->strafe_right >> (8 * 3)) & 0xFF;
      offset += sizeof(this->strafe_right);
      *(outbuffer + offset + 0) = (this->turn_left >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->turn_left >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->turn_left >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->turn_left >> (8 * 3)) & 0xFF;
      offset += sizeof(this->turn_left);
      *(outbuffer + offset + 0) = (this->turn_right >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->turn_right >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->turn_right >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->turn_right >> (8 * 3)) & 0xFF;
      offset += sizeof(this->turn_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->forward =  ((uint32_t) (*(inbuffer + offset)));
      this->forward |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->forward |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->forward |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->forward);
      this->backward =  ((uint32_t) (*(inbuffer + offset)));
      this->backward |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->backward |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->backward |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->backward);
      this->strafe_left =  ((uint32_t) (*(inbuffer + offset)));
      this->strafe_left |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->strafe_left |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->strafe_left |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->strafe_left);
      this->strafe_right =  ((uint32_t) (*(inbuffer + offset)));
      this->strafe_right |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->strafe_right |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->strafe_right |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->strafe_right);
      this->turn_left =  ((uint32_t) (*(inbuffer + offset)));
      this->turn_left |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->turn_left |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->turn_left |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->turn_left);
      this->turn_right =  ((uint32_t) (*(inbuffer + offset)));
      this->turn_right |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->turn_right |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->turn_right |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->turn_right);
     return offset;
    }

    const char * getType(){ return "robot_msgs/RobotCommand"; };
    const char * getMD5(){ return "31a673ead1408eeca5efa0d25da4472a"; };

  };

}
#endif