
#ifndef STEP_MOTOR_HPP
#define STEP_MOTOR_HPP
//include
#include "Arduino.h"
//private macro
//private types
//private variables
//private function declaration

namespace Motor 
{
enum MotorDir : uint8_t {
  kDirNormal,
  kDirOpposite,
};

struct AccOptiData {
  int32_t initial_pulse_interval[2] = {1000, 1000};
  int32_t final_pulse_interval[2] = {1000, 1000};
};

struct MotorParam {
  int con_period = 10;           // us
  int32_t full_cycle_pulse = 3200;   // 1/16
  int getHighTTLPeriod(void) { return con_period/2; };
  int getLowTTLPeriod(void) { return con_period/2; };
  MotorDir dir = MotorDir::kDirNormal;
  int32_t Ratio = 30;
  int enPin = 38;
  int stepPin = 54;
  int dirPin = 55;
};

struct MotorData {
  int32_t angle = 0;
  int32_t input_pulse_num = 0;
  int32_t current_pulse_index = 0;
};

class StepMotor 
{
  public:
    StepMotor(MotorParam motor_param)
    : motor_param_(motor_param){};
    StepMotor(){};
    ~StepMotor(){};
    void resetMotor();
    void setStepMotorEnPin(int pin)
    {
      motor_param_.enPin = pin;
      pinMode(motor_param_.enPin, OUTPUT);
      digitalWrite(motor_param_.enPin, LOW);
    };
    void setStepMotorStepPin(int pin)
    {
      motor_param_.stepPin = pin;
      pinMode(motor_param_.stepPin, OUTPUT);
    };
    void setStepMotorDirPin(int pin)
    {
      motor_param_.dirPin = pin;
      pinMode(motor_param_.dirPin, OUTPUT);
    };
    void setStepMotorDir(uint8_t dir);
    void setStepMotorRatio(int32_t ratio);
    void setStepMotorFullCyclePulseNum(int32_t pulse_num);
    void setStepMotorCurrentAngle(int32_t angle);
    void setStepMotorRotSpeed(uint32_t vel);
    void setStepMotorAngle(int32_t angle, int mode);
    void enableStepMotor(bool is_enable);
    void enableStepMotorAcc(bool is_enable = true);
    Motor::MotorParam& motorParam() { return motor_param_; };
    Motor::MotorData& motorData() { return motor_data_; };
  private:
    void stepMotorRawInput(void);
    int32_t stepMotorAccOpti(int mode);
    void accSet(int32_t pulse_interval);
    MotorParam motor_param_;
    AccOptiData acc_opti_data_;
    MotorData motor_data_;
    bool acc_is_enable_ = true;
    int acc_weight_ = 1;
};

}



#endif
