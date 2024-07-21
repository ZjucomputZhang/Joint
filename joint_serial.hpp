#ifndef JOINT_SERIAL_HPP
#define JOINT_SERIAL_HPP
//include
#include "Arduino.h"
//private macro
//private types
//private variables
//private function declaration

namespace Joint
{
enum ReadState : uint8_t {
  kReadYawAng,
  kReadPitchAng,
  kReadRollAng,
  kReadCatchOrNot,
  kReadStateNum,
};
enum ReadType : uint8_t {
  kReadLetter,
  kReadNumber,
  kReadTypeNum,
};

class JointSerial 
{
  public:
    int32_t cmd[4] = {0};
    JointSerial()
    {
      Serial.begin(9600);
    };
    void serialCmd()
    {
      read_type_ = ReadType::kReadLetter;
      Serial.println("<------------------------------------------------------------------>");
      Serial.println("please input cmd ...");
      while(Serial.available() == 0);
      if (Serial.available() > 0) {
        rx_byte_ = Serial.read();
      }
      do {
        if(read_type_ == ReadType::kReadLetter) {
          switch(rx_byte_) {
            case 'Y':case 'y': read_state_ = ReadState::kReadYawAng;cmd[read_state_] = 0;break;
            case 'P':case 'p': read_state_ = ReadState::kReadPitchAng;cmd[read_state_] = 0;break;
            case 'R':case 'r': read_state_ = ReadState::kReadRollAng;cmd[read_state_] = 0;break;
            case 'C':case 'c': read_state_ = ReadState::kReadCatchOrNot;cmd[read_state_] = 0;break;
            default: Serial.println("invalid cmd. please input again");break;
          }
          read_type_ = ReadType::kReadNumber;
        } else if (read_type_ == ReadType::kReadNumber) {
          if (rx_byte_ >= '0' && rx_byte_ <= '9') {
            current_number_ = rx_byte_ - '0';
            cmd[read_state_] = cmd[read_state_] * 10 + current_number_;
          } else {
            read_type_ = ReadType::kReadLetter;
          }
        }
        while(Serial.available() == 0); 
      } while((rx_byte_ = Serial.read() )!= '\n');
        Serial.println("cmd read finish ...");
      };
    void serialCmdReset()
    {
      memset(cmd, 0, 4*sizeof(int32_t));
    };

  private:
    uint8_t rx_byte_;
    int32_t current_number_;
    ReadState read_state_;
    ReadType read_type_;
};

}


#endif