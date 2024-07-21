/* 3 freedom degree joint control */
//include
#include "step_motor.hpp"
#include "joint_servo.hpp"
#include "joint_serial.hpp"
namespace joint_motor = Motor;
namespace joint_serial = Joint;
//private macro
//private types
enum MotorIdx : uint8_t {
  KMotorIdxLeft,
  kMotorIdxRight,
  kMotorIdxYaw,
  kMotorIdxNum,
};

enum CmdIdx : uint8_t {
  kCmdIdxYaw,
  kCmdIdxPitch,
  kCmdIdxRoll,
  kCmdIdxCatchOrNot,
};

enum Thread : uint8_t {
  kThreadStepMotor,
  kThreadServo,
  kThreadNum,
};

enum ThreadState : uint8_t {
  kThreadSuspend,
  kThreadReady,
  kThreadWorking,
};

struct thread_t {
  Thread threadId;
  void (*thread_entry)();
  ThreadState state = ThreadState::kThreadSuspend;
};

struct JointData {
  int32_t yaw_ang = 0;
  int32_t pitch_ang = 0;
  int32_t roll_ang = 0;
};

struct joint_motor::AccOptiData acc_data;

//private variables
joint_motor::StepMotor *motor_ptr[MotorIdx::kMotorIdxNum] = {nullptr};
joint_motor::Servo *servo_ptr = nullptr;
joint_serial::JointSerial *serial_ptr = nullptr;

thread_t thread[kThreadNum];
JointData joint_data;
//private function declaration
void JointInit();
void JointRun();
void StepMotorThread();
void ServoMotorThread();
void SerialThread();
void JointYawRot();
void JointPitchRot();
void JointRollRot();
void StepMotorRotMeantime();
int32_t StepMotorAcc(int mode);
void AccSet(int32_t pulse_interval);

void JointInit()
{
  for(uint8_t i = 0; i < MotorIdx::kMotorIdxNum; i ++) {
    motor_ptr[i] = new joint_motor::StepMotor();
    motor_ptr[i]->enableStepMotor(true);
  }
  motor_ptr[MotorIdx::KMotorIdxLeft]->setStepMotorRatio(1);
  motor_ptr[MotorIdx::KMotorIdxLeft]->setStepMotorRotSpeed(500);
  motor_ptr[MotorIdx::KMotorIdxLeft]->setStepMotorEnPin(38);
  motor_ptr[MotorIdx::KMotorIdxLeft]->setStepMotorStepPin(54);
  motor_ptr[MotorIdx::KMotorIdxLeft]->setStepMotorDirPin(55);

  motor_ptr[MotorIdx::kMotorIdxRight]->setStepMotorRatio(1);
  motor_ptr[MotorIdx::kMotorIdxRight]->setStepMotorRotSpeed(500);
  motor_ptr[MotorIdx::kMotorIdxRight]->setStepMotorEnPin(56);
  motor_ptr[MotorIdx::kMotorIdxRight]->setStepMotorStepPin(60);
  motor_ptr[MotorIdx::kMotorIdxRight]->setStepMotorDirPin(61);

  motor_ptr[MotorIdx::kMotorIdxYaw]->setStepMotorRatio(30);
  motor_ptr[MotorIdx::kMotorIdxYaw]->setStepMotorRotSpeed(70);
  motor_ptr[MotorIdx::kMotorIdxYaw]->setStepMotorEnPin(62);
  motor_ptr[MotorIdx::kMotorIdxYaw]->setStepMotorStepPin(46);
  motor_ptr[MotorIdx::kMotorIdxYaw]->setStepMotorDirPin(48);
  servo_ptr = new joint_motor::Servo();
  servo_ptr->servoEnable(true);
  serial_ptr = new joint_serial::JointSerial();
}

void JointRun()
{
  SerialThread();
  StepMotorThread();
  ServoMotorThread();

}

void SerialThread()
{
  // serial_ptr->serialCmdReset();
  serial_ptr->serialCmd();
}

void StepMotorThread()
{
  Serial.println("rotate start ...");
  JointYawRot();
  JointPitchRot();
  JointRollRot();
  Serial.println("rotate finished ...");
}

void JointYawRot()
{
  int32_t yaw_angle_delta = serial_ptr->cmd[CmdIdx::kCmdIdxYaw] - joint_data.yaw_ang;
  motor_ptr[MotorIdx::kMotorIdxYaw]->setStepMotorAngle(yaw_angle_delta, 1); 
  joint_data.yaw_ang += yaw_angle_delta;
}

void JointPitchRot()
{
  motor_ptr[MotorIdx::KMotorIdxLeft]->setStepMotorRotSpeed(700);
  motor_ptr[MotorIdx::kMotorIdxRight]->setStepMotorRotSpeed(700);
  int32_t pitch_angle_delta = serial_ptr->cmd[CmdIdx::kCmdIdxPitch] - joint_data.pitch_ang;
  motor_ptr[MotorIdx::KMotorIdxLeft]->setStepMotorAngle(-pitch_angle_delta, 0);
  motor_ptr[MotorIdx::kMotorIdxRight]->setStepMotorAngle(pitch_angle_delta, 0);
  StepMotorRotMeantime(motor_ptr[MotorIdx::KMotorIdxLeft], motor_ptr[MotorIdx::kMotorIdxRight]);
  joint_data.pitch_ang += pitch_angle_delta;
}

void JointRollRot()
{
  motor_ptr[MotorIdx::KMotorIdxLeft]->setStepMotorRotSpeed(1200);
  motor_ptr[MotorIdx::kMotorIdxRight]->setStepMotorRotSpeed(1200);
  int32_t roll_angle_delta = serial_ptr->cmd[CmdIdx::kCmdIdxRoll] - joint_data.roll_ang;
  int32_t angle = 3*roll_angle_delta;
  motor_ptr[MotorIdx::KMotorIdxLeft]->setStepMotorAngle(angle, 0);
  motor_ptr[MotorIdx::kMotorIdxRight]->setStepMotorAngle(angle, 0);
  StepMotorRotMeantime(motor_ptr[MotorIdx::KMotorIdxLeft], motor_ptr[MotorIdx::kMotorIdxRight]);
  joint_data.roll_ang += roll_angle_delta;
}

void JointRollPitchRot()
{
  motor_ptr[MotorIdx::KMotorIdxLeft]->setStepMotorRotSpeed(500);
  motor_ptr[MotorIdx::kMotorIdxRight]->setStepMotorRotSpeed(500);
  int32_t pitch_angle_delta = serial_ptr->cmd[CmdIdx::kCmdIdxPitch] - joint_data.pitch_ang;
  int32_t roll_angle_delta = serial_ptr->cmd[CmdIdx::kCmdIdxRoll] - joint_data.roll_ang;
  int32_t angle = 3*roll_angle_delta;
  int32_t angle_left = -pitch_angle_delta + angle;
  int32_t angle_right = pitch_angle_delta + angle;
  motor_ptr[MotorIdx::KMotorIdxLeft]->setStepMotorAngle(angle_left, 0);
  motor_ptr[MotorIdx::kMotorIdxRight]->setStepMotorAngle(angle_right, 0);
  StepMotorRotMeantime(motor_ptr[MotorIdx::KMotorIdxLeft], motor_ptr[MotorIdx::kMotorIdxRight]);
  joint_data.roll_ang += roll_angle_delta;
  joint_data.pitch_ang += pitch_angle_delta;
}

void ServoMotorThread()
{
  if (serial_ptr->cmd[CmdIdx::kCmdIdxCatchOrNot] == 1) {
    Serial.println("catch triggered ...");
    servo_ptr->servoInput(120);
  } else if (serial_ptr->cmd[CmdIdx::kCmdIdxCatchOrNot] == 2) {
    Serial.println("catch triggered ...");
    servo_ptr->servoInput(100);
  } else {
    servo_ptr->servoInput(0);
  }
}

void StepMotorRotMeantime(joint_motor::StepMotor *motor_ptr1, joint_motor::StepMotor* motor_ptr2)
{
  int32_t high_delay_us = motor_ptr2->motorParam().getHighTTLPeriod();
  int32_t low_delay_us = motor_ptr2->motorParam().getLowTTLPeriod();
  AccSet(high_delay_us);
  high_delay_us = StepMotorAcc(1);
  if (motor_ptr1->motorParam().dir == 0) {
    digitalWrite(motor_ptr1->motorParam().dirPin, LOW);
  } else {
    digitalWrite(motor_ptr1->motorParam().dirPin, HIGH);  
  }
  if (motor_ptr2->motorParam().dir == 0) {
    digitalWrite(motor_ptr2->motorParam().dirPin, LOW);
  } else {
    digitalWrite(motor_ptr2->motorParam().dirPin, HIGH);  
  }
  for(int32_t i = 0; i < motor_ptr1->motorData().input_pulse_num; i ++) {
    if (high_delay_us > motor_ptr2->motorParam().getHighTTLPeriod() && i < motor_ptr1->motorData().input_pulse_num *4 / 5) {
      high_delay_us = StepMotorAcc(1);
      low_delay_us = StepMotorAcc(1);
    } 
    if (i > motor_ptr1->motorData().input_pulse_num - 400) {
      high_delay_us = StepMotorAcc(2);
      low_delay_us = StepMotorAcc(2);

    }
    digitalWrite(motor_ptr1->motorParam().stepPin, HIGH);
    digitalWrite(motor_ptr2->motorParam().stepPin, HIGH);
    delayMicroseconds(high_delay_us);
    digitalWrite(motor_ptr1->motorParam().stepPin, LOW);
    digitalWrite(motor_ptr2->motorParam().stepPin, LOW);
    delayMicroseconds(low_delay_us);
  }
}

int32_t StepMotorAcc(int mode)
{
  int32_t output_pulse_interval = 0;
  if (mode == 1) {
    output_pulse_interval = acc_data.initial_pulse_interval[0];
    output_pulse_interval -= 2;
    acc_data.initial_pulse_interval[0] = output_pulse_interval;
  } else {
    output_pulse_interval = acc_data.initial_pulse_interval[1];
    output_pulse_interval += 2;
    acc_data.initial_pulse_interval[1] = output_pulse_interval;
  }
  return output_pulse_interval;
}

void AccSet(int32_t pulse_interval)
{
  acc_data.initial_pulse_interval[0] = 1000;
  acc_data.initial_pulse_interval[1] = pulse_interval;
  acc_data.final_pulse_interval[0] = pulse_interval;
  acc_data.final_pulse_interval[1] = 1000;
}

void setup() {
  // put your setup code here, to run once:
  JointInit();
}

void loop() {
  // put your main code here, to run repeatedly:
  JointRun();
}
