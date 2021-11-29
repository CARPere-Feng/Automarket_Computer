//
// Created by miniload on 2021/11/28.
//

#include "rclcpp/rclcpp.hpp"

#include "../motor/include/miniloadCore/motor/doubleMotor.h"
#include "../common/include/miniloadCore/common/shelf_pos_config.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  //初始化节点
    rclcpp::Rate loop_rate(100);

    DoubleMotor miniload;
    miniload.disableMotors(':');
    miniload.fastWrite_FastAbs_inc(':',0,0);
    miniload.goOrigin();
    miniload.setPosModeAcc('b',10,10);
    miniload.setPosModeAcc('a',10,10);
    miniload.enable_FastAbs_displacementMode(':');
    miniload.fastWrite_FastAbs_inc(':',shelf_absPos_a[3],shelf_absPos_b[4]);
    miniload.disableMotors(':');

    while (rclcpp::ok()) {
        miniload.Motor_Feedback();

        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
