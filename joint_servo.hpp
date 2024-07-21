#ifndef JOINT_SERVO_HPP
#define JOINT_SERVO_HPP
//include
#include "Arduino.h"
//private macro
#define SERVO_PIN             4
//private types
//private variables
//private function declaration

namespace Motor 
{

struct ServoParam {
  int32_t con_period = 20000;
  int32_t hight_ttl_microsec = 0;
  int32_t getHighTTLMicroSec() { return hight_ttl_microsec; };
  int32_t getLowTTLMicroSec() { return con_period - hight_ttl_microsec; };
};

class Servo
{
  public:
    Servo(){};
    ~Servo(){};
    void servoEnable(bool is_enable = false)
    {
      if (is_enable) {
        pinMode(SERVO_PIN, OUTPUT);
      }
    };
    void servoInput(int32_t angle)
    {
      int32_t micro_second = (int32_t)(angle/270.0*2000) + 500;
      servo_param_.hight_ttl_microsec = micro_second;
      digitalWrite(SERVO_PIN, HIGH);
      delayMicroseconds(servo_param_.getHighTTLMicroSec());
      digitalWrite(SERVO_PIN, LOW);
      delayMicroseconds(servo_param_.getLowTTLMicroSec());
    };
  private:
    ServoParam servo_param_;
};

}

#endif