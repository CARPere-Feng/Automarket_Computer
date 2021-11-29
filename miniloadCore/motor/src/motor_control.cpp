#include "../include/miniloadCore/motor/motor_control.h"
#include "../../communication/include/miniloadCore/communication/can_drive.h"
#include "../../communication/include/miniloadCore/communication/ICANCmd.h"

#include <iostream>

#include "rclcpp/rclcpp.hpp"


Motor_Control::Motor_Control(std::shared_ptr<can_communication>& CAN_obj):
    CAN_obj_(CAN_obj){
  CAN_obj_->Can_Open();
  CAN_obj_->Can_Get_Device_Info();
  CAN_obj_->Can_Channel_Start();
  CAN_obj_->Can_Get_Error_Info();
  PDO_Open[0] = 0x01;
  PDO_Open[1] = 0x00;
}

Motor_Control::~Motor_Control() {
    BYTE Control_Word_Stop[2] = {0x06, 0x00};
  // this->Motor_Speed_Control(0x201, 0, 0x03,Control_Word_Stop);
  this->Motor_Speed_Control(0x301, 0);
  this->Motor_Mode_Control(0x201, 0xfd, Control_Word_Stop);
  // this->Motor_Speed_Control(0x202, 0, 0x03,Control_Word_Stop);
  this->Motor_Speed_Control(0x302, 0);
  this->Motor_Mode_Control(0x202, 0xfd, Control_Word_Stop);
  CAN_obj_->Can_Channel_Stop();
}

void Motor_Control::Can_Close(){
  CAN_obj_->Can_Channel_Stop();
}

void Motor_Control::Can_Start(){
  CAN_obj_->Can_Channel_Start();
}

bool Motor_Control::Motor_PDO_Open() {
  if (CAN_obj_->Can_Channel_Send(0x00, 2, PDO_Open)) {
    // std::cout << "PDO start  success" << std::endl;
    return 1;
  } else {
    // std::cout << "PDO start failure" << std::endl;
    return 0;
  }
}

/// <Motor_Speed> unit: rpm </Motor_Speed>
void Motor_Control::Motor_Speed_Control(INT16 Motor_RPDO_ID, int Motor_Speed) {
  if (is_Speed_Safe(Motor_Speed)) {
      BYTE Speed_Message[4];
      double speed;
      speed = (512*Motor_Speed * encoder_num) / 1875.0;
      Dec2HexVector(Speed_Message, speed, 4);

      if (CAN_obj_->Can_Channel_Send(Motor_RPDO_ID, 4, Speed_Message) == TRUE) {
          // std::cout << "speed control send sucess" << std::endl;
      } else {
          // std::cout << "speed control send failure" << std::endl;
      }
  }
}

void Motor_Control::Motor_Mode_Control(INT16 Motor_RPDO_ID, BYTE WorkMode,
                                       BYTE CONTROL_Word[2]) {
  BYTE Send_Message[3];
  Send_Message[0] = WorkMode;
  Send_Message[1] = CONTROL_Word[0];
  Send_Message[2] = CONTROL_Word[1];
  if (CAN_obj_->Can_Channel_Send(Motor_RPDO_ID, 3, Send_Message) == TRUE) {
    // std::cout << "mode control send sucess" << std::endl;
  } else {
    // std::cout << "mode control send failure" << std::endl;
  }
}

void Motor_Control::Motor_Acc_Control(INT16 Motor_PRDO_ID, const hex2int32 &positive, const hex2int32 &negative) {
    BYTE Send_Message[8];
    for (int i = 0; i < 4; ++i) {
        Send_Message[i] = positive.hexVal[i];
        Send_Message[i+4] = negative.hexVal[i];
    }
    if (CAN_obj_->Can_Channel_Send(Motor_PRDO_ID, 8, Send_Message) == TRUE) {
        // std::cout << "accelration control send sucess" << std::endl;
    } else {
        // std::cout << "acceleration control send failure" << std::endl;
    }
}

void Motor_Control::Motor_Pos_Control(INT16 Motor_RPDO_ID, int Target_Position_inc,
                                      int Trapezoid_Vel_rpm) {
  BYTE Send_Message[8];
  hex2int32 position, speed;
  position.integer32 = Target_Position_inc;
  speed.integer32 = (512.0 * Trapezoid_Vel_rpm * encoder_num) / 1875.0;
  Send_Message[0] = position.hexVal[0];
  Send_Message[1] = position.hexVal[1];
  Send_Message[2] = position.hexVal[2];
  Send_Message[3] = position.hexVal[3];
  Send_Message[4] = speed.hexVal[0];
  Send_Message[5] = speed.hexVal[1];
  Send_Message[6] = speed.hexVal[2];
  Send_Message[7] = speed.hexVal[3];
  if (CAN_obj_->Can_Channel_Send(Motor_RPDO_ID, 8, Send_Message) == TRUE) {
    // std::cout << "lift control send sucess" << std::endl;
  } else {
    // std::cout << "lift control send failure" << std::endl;
  }
}



void Motor_Control::Dec2HexVector(BYTE *data_vec, const int &dec_value,
                                  const int &len) {
  for (int i = 0; i < len; i++) {
    data_vec[i] = (((int)dec_value >> (i * 8)) & 0xff);
  }
}

// velecity should be within 50 rpm
bool Motor_Control::is_Speed_Safe(int &vel) {
    if (vel > up_vel_) {
        vel = up_vel_;
    }
    else if(vel < low_vel_) {
        vel = low_vel_;
    }
    return true;
}

void Motor_Control::Set_Speed_Boundary(const int &upper, const int &lower) {
    up_vel_ = upper;
    low_vel_ = lower;
}