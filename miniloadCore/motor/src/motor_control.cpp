#include "../include/miniloadCore/motor/motor_control.h"
#include "../../communication/include/miniloadCore/communication/can_drive.h"
#include "../../communication/include/miniloadCore/communication/ICANCmd.h"

#include <iostream>

#include "rclcpp/rclcpp.hpp"

union Control_value {
  BYTE modify_value[4];
  int exchange_value;
};

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

void Motor_Control::Motor_Lift_Control(INT16 Motor_RPDO_ID, int Target_Position,
                                       int Lift_Trapezoid_Speed) {
  BYTE Send_Message[8];
  union Control_value position, speed;
  position.exchange_value = Target_Position;
  speed.exchange_value = (512.0 * Lift_Trapezoid_Speed * encoder_num) / 1875.0;
  Send_Message[0] = position.modify_value[0];
  Send_Message[1] = position.modify_value[1];
  Send_Message[2] = position.modify_value[2];
  Send_Message[3] = position.modify_value[3];
  Send_Message[4] = speed.modify_value[0];
  Send_Message[5] = speed.modify_value[1];
  Send_Message[6] = speed.modify_value[2];
  Send_Message[7] = speed.modify_value[3];
  if (CAN_obj_->Can_Channel_Send(Motor_RPDO_ID, 8, Send_Message) == TRUE) {
    // std::cout << "lift control send sucess" << std::endl;
  } else {
    // std::cout << "lift control send failure" << std::endl;
  }
}

bool Motor_Control::Motor_Feedback() {
  BYTE data[2];
  CAN_obj_->Can_Channel_Send(0x80, 0, data);

  int count = 0;
  bool flag_tmp[2];
  flag_tmp[0] = false;
  flag_tmp[1] = false;
  if (CAN_obj_->Can_Channel_Receive()) {
    left_dis = 0;
    left_dis_value = 0;

    right_dis = 0;
    right_dis_value = 0;

    for (int i = 0; i < 4; i++) {
      if (CAN_obj_->Cdf_i[i].uID == 0x282)  // 0x282
      {
        if (flag_tmp[0]) {
          count++;
          continue;
        }
        speed_change.real_time_speed[0] = CAN_obj_->Cdf_i[i].arryData[0];
        speed_change.real_time_speed[1] = CAN_obj_->Cdf_i[i].arryData[1];
        speed_change.real_time_speed[2] = CAN_obj_->Cdf_i[i].arryData[2];
        speed_change.real_time_speed[3] = CAN_obj_->Cdf_i[i].arryData[3];
        dis_wheel.dis[0] = CAN_obj_->Cdf_i[i].arryData[4];
        dis_wheel.dis[1] = CAN_obj_->Cdf_i[i].arryData[5];
        dis_wheel.dis[2] = CAN_obj_->Cdf_i[i].arryData[6];
        dis_wheel.dis[3] = CAN_obj_->Cdf_i[i].arryData[7];

        // std::cout << std::hex << "0x" << CAN_obj_->Cdf_i[i].uID << "  : ";
        // for (size_t j = 0; j < 8; j++) {
        //   std::cout << std::hex << "0x" << (int)CAN_obj_->Cdf_i[i].arryData[j]
        //             << "  ";
        // }
        // std::cout << std::endl;

        left_dis_value = (double)dis_wheel.dis_moved / (double)encoder_num;
        left_dis = -((double)dis_wheel.dis_moved * 2);
        // ROS_WARN("left dis : %.6f", left_dis);
        /*for(i=0;i<4;i++)
        {
                std::cout<<std::to_string(speed_change.real_time_speed[i])<<std::endl;
        }*/
        left_realtime_Speed =
            -((speed_change.real_speed * 1875.0) / (encoder_num * 512.0 ));
        if (left_realtime_Speed<5 & left_realtime_Speed> - 5) {
          left_realtime_Speed = 0;
        }
        // std::cout<<"left_real_speed:"<<speed_change.real_speed<<std::endl;
        // std::cout<<CAN_obj_->Cdf_i[i].uID<<std::endl;
        // std::cout<<i<<std::endl;
        flag_tmp[0] = true;
        count++;
      } else if (CAN_obj_->Cdf_i[i].uID == 0x281)  // 0x281
      {
        if (flag_tmp[1]) {
          count++;
          continue;
        }
        speed_change.real_time_speed[0] = CAN_obj_->Cdf_i[i].arryData[0];
        speed_change.real_time_speed[1] = CAN_obj_->Cdf_i[i].arryData[1];
        speed_change.real_time_speed[2] = CAN_obj_->Cdf_i[i].arryData[2];
        speed_change.real_time_speed[3] = CAN_obj_->Cdf_i[i].arryData[3];
        dis_wheel.dis[0] = CAN_obj_->Cdf_i[i].arryData[4];
        dis_wheel.dis[1] = CAN_obj_->Cdf_i[i].arryData[5];
        dis_wheel.dis[2] = CAN_obj_->Cdf_i[i].arryData[6];
        dis_wheel.dis[3] = CAN_obj_->Cdf_i[i].arryData[7];

        // std::cout << std::hex << "0x" << CAN_obj_->Cdf_i[i].uID << "  : ";
        // for (size_t j = 0; j < 8; j++) {
        //   std::cout << std::hex << "0x" << (int)CAN_obj_->Cdf_i[i].arryData[j]
        //             << "  ";
        // }
        // std::cout << std::endl;

        right_dis_value = (double)dis_wheel.dis_moved / (double)encoder_num;
        right_dis = ((double)dis_wheel.dis_moved * 2 * PI);
        // ROS_WARN("right dis : %.6f", right_dis);
        /*for(i=0;i<4;i++)
        {
                std::cout<<std::to_string(speed_change.real_time_speed[i])<<std::endl;
        }*/
        right_realtime_Speed =
            (speed_change.real_speed * 1875.0) / (encoder_num * 512.0 );
        if (right_realtime_Speed<5 & right_realtime_Speed> - 5) {
          right_realtime_Speed = 0;
        }
        // std::cout<<CAN_obj_->Cdf_i[i].uID<<std::endl;
        //std::cout<<"right_real_speed:"<<std::to_string(right_realtime_Speed)<<std::endl;
        // std::cout<<i<<std::endl;

        flag_tmp[1] = true;
        count++;
      }
    }
  } else {
    std::cout << CAN_obj_->Cdf_i[0].uID << std::endl;
    std::cout << "I don't receive data" << std::endl;
    return 0;
  }

  // ROS_INFO("data quantity for computation : %d", count);
  if (flag_tmp[0] && flag_tmp[1]) {
    return true;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"One of the motor does not receive data ");
    return false;
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