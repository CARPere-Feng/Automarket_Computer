#include <iostream>
#include <memory>
#include "../motor/include/miniloadCore/motor/motor_control.h"
#include "../motor/include/miniloadCore/motor/doubleMotor.h"
#include "../communication/include/miniloadCore/communication/can_drive.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);  //初始化节点
  rclcpp::Rate loop_rate(100);

  DoubleMotor miniload;
  miniload.disableVelocityMode(':');

  // writePositionDEMO();
  miniload.enableVelocityMode(':');
  miniload.fastWriteRPM('a',-100);    // a is up-down; b is forward-backward
    miniload.fastWriteRPM('a',10);
//  miniload.fastWriteRPM('b',-50);
//    miniload.fastWriteRPM('b',0);
    miniload.Motor_Feedback();
    RCLCPP_INFO(rclcpp::get_logger("vel_control_node"), " current a is %d, b is %d",miniload.a_dis_inc_,miniload.b_dis_inc_);
    miniload.disableVelocityMode(':');

  while (rclcpp::ok()) {
    miniload.Motor_Feedback();

    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
