//
// Created by miniload on 2021/11/28.
//

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "../motor/include/miniloadCore/motor/doubleMotor.h"
#include "../common/include/miniloadCore/common/shelf_pos_config.h"

void moveTo(DoubleMotor& obj,const int& targeta, const int& targetb, const int& threshold) {
    obj.fastWrite_FastAbs_inc(':',targeta,targetb);

    auto errora = std::abs(obj.a_dis_inc_-targeta);
    auto errorb = std::abs(obj.b_dis_inc_-targetb);
    while (errora > threshold || errorb > threshold) {
        obj.Motor_Feedback();
        errora = std::abs(obj.a_dis_inc_-targeta);
        errorb = std::abs(obj.b_dis_inc_-targetb);
        //std::cout << errora << '\t' << errorb << std::endl;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  //初始化节点
    rclcpp::Rate loop_rate(100);

    DoubleMotor miniload;
    miniload.disableMotors(':');
    miniload.setPosModeAcc('b',10,10);
    miniload.setPosModeAcc('a',10,10);
    miniload.Motor_Feedback();
    miniload.enable_FastAbs_displacementMode(':');

    int threshold = 100;
    auto targeta = shelf_absPos_a[5];
    auto targetb = shelf_absPos_b[6];
    moveTo(miniload,targeta, targetb, threshold);

    targeta = shelf_absPos_a[2];
    targetb = shelf_absPos_b[4];
    moveTo(miniload,targeta, targetb, threshold);

    targeta = shelf_absPos_a[5];
    targetb = shelf_absPos_b[2];
    moveTo(miniload,targeta, targetb, threshold);

    miniload.disableMotors(':');

    while (rclcpp::ok()) {
        miniload.Motor_Feedback();

        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}

