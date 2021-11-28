#include <iostream>
#include <memory>
#include "../motor/include/miniloadCore/motor/motor_control.h"
#include "../motor/include/miniloadCore/motor/doubleMotor.h"
#include "../communication/include/miniloadCore/communication/can_drive.h"

#include "rclcpp/rclcpp.hpp"
//auto CAN_obj = std::make_shared<can_communication>();
//Motor_Control motor_Ctr(CAN_obj);
//
//
//struct MotorASpeedMode {
//  INT16 Motor_ID1 = 0x202;  // mode
//  INT16 Motor_ID2 = 0x302;  // speed
//  BYTE Control_Word[2] = {0x0f, 0x00};
//  BYTE Control_Word_Stop[2] = {0x06, 0x00};
//  BYTE Work_Mode = 0xfd;  // -3
//  int Speed;
//
//} motorA_v;
//
//struct MotorBSpeedMode {
//  INT16 Motor_ID1 = 0x201;  // mode
//  INT16 Motor_ID2 = 0x301;  // speed
//  BYTE Control_Word[2] = {0x0f, 0x00};
//  BYTE Control_Word_Stop[2] = {0x06, 0x00};
//
//  BYTE Work_Mode = 0xfd;  // -3
//  int Speed;
//
//} motorB_v;

struct MotorAPosMode {
  INT16 Motor_ID1 = 0x202;
  INT16 Motor_ID2 = 0x402;
  // Control Word:
  // low byte is at left, high byte is at right
  // low byte is small number of the array
  BYTE CW_Relative1[2] = {0x4f, 0x00}; BYTE CW_Relative2[2] = {0x5f, 0x00};
  BYTE CW_Abs_Fast[2] = {0x3F, 0x10};
  BYTE CW_Abs_Wait1[2] = {0x2f, 0x00}; BYTE CW_Abs_Wait2[2] = {0x3f, 0x00};
  BYTE CW_Stop[2] = {0x06, 0x00};
  BYTE Work_Mode = 0x01;
  int target_position;
  int Speed;

} motorA_q;

struct MotorBPosMode {
  INT16 Motor_ID1 = 0x201;
  INT16 Motor_ID2 = 0x401;
  // Control Word:
  // low byte is at left, high byte is at right
  // low byte is small number of the array
  BYTE CW_Relative1[2] = {0x4f, 0x00}; BYTE CW_Relative2[2] = {0x5f, 0x00};
  BYTE CW_Abs_Fast[2] = {0x3F, 0x10};
  BYTE CW_Abs_Wait1[2] = {0x2f, 0x00}; BYTE CW_Abs_Wait2[2] = {0x3f, 0x00};
  BYTE CW_Stop[2] = {0x06, 0x00};
  BYTE Work_Mode = 0x01;
  int target_position;
  int Speed;

} motorB_q;

//void writePositionDEMO() {
//  motorA_q.target_position = 4000000;
//  motorA_q.Speed = 170;
//  motorB_q.target_position = 8000000;
//  motorB_q.Speed = 200;
//  motor_Ctr.Motor_Lift_Control(motorA_q.Motor_ID2,
//                               motorA_q.target_position,
//                               motorA_q.Speed);
//  motor_Ctr.Motor_Mode_Control(motorA_q.Motor_ID1,
//                               motorA_q.Work_Mode,
//                               motorA_q.CW_Abs_Fast);
//
//  motor_Ctr.Motor_Lift_Control(motorB_q.Motor_ID2,
//                               motorB_q.target_position,
//                               motorB_q.Speed);
//  motor_Ctr.Motor_Mode_Control(motorB_q.Motor_ID1,
//                               motorB_q.Work_Mode,
//                               motorB_q.CW_Abs_Fast);
//
//  motorA_q.target_position = -3000000;
//  motorA_q.Speed = 200;
//  motorB_q.target_position = 8000000;
//  motorB_q.Speed = 0;
//  motor_Ctr.Motor_Lift_Control(motorA_q.Motor_ID2,
//                               motorA_q.target_position,
//                               motorA_q.Speed);
//  motor_Ctr.Motor_Mode_Control(motorA_q.Motor_ID1,
//                               motorA_q.Work_Mode,
//                               motorA_q.CW_Abs_Fast);
//
//  motor_Ctr.Motor_Lift_Control(motorB_q.Motor_ID2,
//                               motorB_q.target_position,
//                               motorB_q.Speed);
//  motor_Ctr.Motor_Mode_Control(motorB_q.Motor_ID1,
//                               motorB_q.Work_Mode,
//                               motorB_q.CW_Abs_Fast);
//
//  motorA_q.target_position = -3000000;
//  motorA_q.Speed = 0;
//  motorB_q.target_position = 0;
//  motorB_q.Speed = 200;
//  motor_Ctr.Motor_Lift_Control(motorA_q.Motor_ID2,
//                               motorA_q.target_position,
//                               motorA_q.Speed);
//  motor_Ctr.Motor_Mode_Control(motorA_q.Motor_ID1,
//                               motorA_q.Work_Mode,
//                               motorA_q.CW_Abs_Fast);
//
//  motor_Ctr.Motor_Lift_Control(motorB_q.Motor_ID2,
//                               motorB_q.target_position,
//                               motorB_q.Speed);
//  motor_Ctr.Motor_Mode_Control(motorB_q.Motor_ID1,
//                               motorB_q.Work_Mode,
//                               motorB_q.CW_Abs_Fast);
//  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"motor_Mode is Position!!!!!!");
//}

//void writeV() {
//    // A move
//  motor_Ctr.Motor_Mode_Control(motorA_v.Motor_ID1, motorA_v.Work_Mode,
//                               motorA_v.Control_Word);
//  motor_Ctr.Motor_Speed_Control(motorA_v.Motor_ID2, 10);
//
//    // A stop
//    motor_Ctr.Motor_Mode_Control(motorA_v.Motor_ID1, motorA_v.Work_Mode,
//                                 motorA_v.Control_Word_Stop);
//    motor_Ctr.Motor_Speed_Control(motorA_v.Motor_ID2, 0);
//
//  // B stop
//    motor_Ctr.Motor_Mode_Control(motorB_v.Motor_ID1, motorB_v.Work_Mode,
//                               motorB_v.Control_Word_Stop);
//    motor_Ctr.Motor_Speed_Control(motorB_v.Motor_ID2, 0);
//  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"ROS Node initialized successful.");
//
//}
//
//void stop_cmd() {
//  motor_Ctr.Motor_Speed_Control(motorA_v.Motor_ID2, 0);
//  motor_Ctr.Motor_Mode_Control(motorA_v.Motor_ID1, motorA_v.Work_Mode,
//                               motorA_v.Control_Word_Stop);
//  motor_Ctr.Motor_Speed_Control(motorB_v.Motor_ID2, 0);
//  motor_Ctr.Motor_Mode_Control(motorB_v.Motor_ID1, motorB_v.Work_Mode,
//                               motorB_v.Control_Word_Stop);
////  motor_Ctr.Can_Close();
//}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);  //初始化节点
  rclcpp::Rate loop_rate(100);

  DoubleMotor miniload;
  miniload.disableVelocityMode(':');

  // writePositionDEMO();
  miniload.enableVelocityMode(':');
  miniload.fastWriteRPM('a',10);    // a is up-down; b is forward-backward
    miniload.fastWriteRPM('a',0);
//  miniload.fastWriteRPM('b',-50);
//    miniload.fastWriteRPM('b',0);

    miniload.disableVelocityMode(':');

  while (rclcpp::ok()) {
    miniload.a_motor_->Motor_Feedback();

    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
