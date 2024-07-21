/* 3 freedom degree joint control */
//include
#include "step_motor.hpp"
//private macro
//private types
//private variables
//private function declaration
namespace Motor 
{

void StepMotor::resetMotor()
{
  motor_data_.input_pulse_num = 0;
  motor_data_.current_pulse_index = 0;
}

void StepMotor::enableStepMotor(bool is_enable)
{
  pinMode(motor_param_.stepPin, OUTPUT);
  pinMode(motor_param_.dirPin, OUTPUT);
  pinMode(motor_param_.enPin, OUTPUT);
  if (is_enable) {
    digitalWrite(motor_param_.enPin, LOW);
  } else {
    digitalWrite(motor_param_.enPin, HIGH);
  }
}

void StepMotor::enableStepMotorAcc(bool is_enable = true) 
{
  acc_is_enable_ = is_enable;
}

void StepMotor::setStepMotorAngle(int32_t angle, int mode)
{
  angle -= motor_data_.angle;
  angle *= motor_param_.Ratio;
  if (angle > 0) {
    motor_param_.dir = MotorDir::kDirNormal;
  } else {
    motor_param_.dir = MotorDir::kDirOpposite;
    angle = - angle;
  }
  float ang_per_pulse = 360.0/motor_param_.full_cycle_pulse;
  motor_data_.input_pulse_num = (int32_t)(angle / ang_per_pulse);
  if (mode == 1) {
    stepMotorRawInput();
  }
}

void StepMotor::setStepMotorCurrentAngle(int32_t angle)
{
  motor_data_.angle = angle;
}

void StepMotor::setStepMotorDir(uint8_t dir)
{
  if (dir == 0) {
    motor_param_.dir = MotorDir::kDirNormal;
  } else {
    motor_param_.dir = MotorDir::kDirOpposite;
  }
}

void StepMotor::setStepMotorFullCyclePulseNum(int32_t pulse_num)
{
  motor_param_.full_cycle_pulse = pulse_num;
}

void StepMotor::setStepMotorRatio(int32_t ratio)
{
  motor_param_.Ratio = ratio;
}

void StepMotor::setStepMotorRotSpeed(uint32_t vel)
{
  vel *= motor_param_.Ratio;
  if (vel <= 150) {
    vel = 150;
  } else if (vel >= 2100) {
    vel = 2100;
  }
  float cycle = vel / 360.0;
  float one_pulse_ms = 1000 / cycle / motor_param_.full_cycle_pulse;
  int32_t one_pulse_us = (int32_t)(one_pulse_ms * 1000);
  if (one_pulse_us % 2 == 1) {
    one_pulse_us += 1;
  } 
  motor_param_.con_period = one_pulse_us;
}

void StepMotor::stepMotorRawInput()
{
  int32_t high_delay_us = motor_param_.getHighTTLPeriod();
  int32_t low_delay_us = motor_param_.getLowTTLPeriod();
  accSet(high_delay_us);
  if (motor_param_.dir == MotorDir::kDirNormal) {
    digitalWrite(motor_param_.dirPin, LOW);
  } else {
    digitalWrite(motor_param_.dirPin, HIGH);
  }
  for(int32_t i = 0; i < motor_data_.input_pulse_num; i ++) {
    if (high_delay_us > motor_param_.getHighTTLPeriod()) {
      high_delay_us = stepMotorAccOpti(1);
      low_delay_us = stepMotorAccOpti(1);      
    };
    digitalWrite(motor_param_.stepPin, HIGH);
    delayMicroseconds(high_delay_us);
    digitalWrite(motor_param_.stepPin, LOW);
    delayMicroseconds(low_delay_us);
    motor_data_.current_pulse_index ++;
  }
}

int32_t StepMotor::stepMotorAccOpti(int mode)
{
  int32_t output_pulse_interval = 0;
  if (mode == 1) {
    output_pulse_interval = acc_opti_data_.initial_pulse_interval[0];
    output_pulse_interval -= 1;
    acc_opti_data_.initial_pulse_interval[0] = output_pulse_interval;
  } else {
    output_pulse_interval = acc_opti_data_.initial_pulse_interval[1];
    output_pulse_interval += 1;
    acc_opti_data_.initial_pulse_interval[1] = output_pulse_interval;
  }
  return output_pulse_interval;
}

void StepMotor::accSet(int32_t pulse_interval)
{
  acc_opti_data_.initial_pulse_interval[0] = 1000;
  acc_opti_data_.initial_pulse_interval[1] = pulse_interval;
  acc_opti_data_.final_pulse_interval[0] = pulse_interval;
  acc_opti_data_.final_pulse_interval[1] = 1000;
}

}
