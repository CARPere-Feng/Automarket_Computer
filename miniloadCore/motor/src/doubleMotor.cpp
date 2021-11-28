//
// Created by Junda on 2021/11/17.
//

#include "../include/miniloadCore/motor/doubleMotor.h"
#include <iostream>

DoubleMotor::DoubleMotor() :
    CAN_connector_(std::make_shared<can_communication>()),
    a_motor_(std::make_unique<Motor_Control>(CAN_connector_)),
    b_motor_(std::make_unique<Motor_Control>(CAN_connector_)){
    if (a_motor_->Motor_PDO_Open() || b_motor_->Motor_PDO_Open())
        std::cerr << "motor PDO Open Failed" << std::endl;
}

DoubleMotor::~DoubleMotor() {
    a_motor_.reset();
    b_motor_.reset();
    CAN_connector_.reset();
}

bool DoubleMotor::disableVelocityMode(const MotorID &id) {
    switch (id) {
        case 'a': {
            a_motor_->Motor_Mode_Control(motorA_v_config_.Motor_ID1, motorA_v_config_.Work_Mode,
                                         motorA_v_config_.Control_Word_Stop);
            break;
        }
        case 'b': {
            b_motor_->Motor_Mode_Control(motorB_v_config_.Motor_ID1, motorB_v_config_.Work_Mode,
                                         motorB_v_config_.Control_Word_Stop);
            break;
        }
        case ':': {
            a_motor_->Motor_Mode_Control(motorA_v_config_.Motor_ID1, motorA_v_config_.Work_Mode,
                                         motorA_v_config_.Control_Word_Stop);
            b_motor_->Motor_Mode_Control(motorB_v_config_.Motor_ID1, motorB_v_config_.Work_Mode,
                                         motorB_v_config_.Control_Word_Stop);
            break;
        }
        default:
            // ERROR HINTS
            break;
    }
}

bool DoubleMotor::enableVelocityMode(const MotorID &id) {
    switch (id) {
        case 'a': {
            a_motor_->Motor_Mode_Control(motorA_v_config_.Motor_ID1, motorA_v_config_.Work_Mode,
                                         motorA_v_config_.Control_Word);
            break;
        }
        case 'b': {
            b_motor_->Motor_Mode_Control(motorB_v_config_.Motor_ID1, motorB_v_config_.Work_Mode,
                                         motorB_v_config_.Control_Word);
            break;
        }
        case ':': {
            a_motor_->Motor_Mode_Control(motorA_v_config_.Motor_ID1, motorA_v_config_.Work_Mode,
                                         motorA_v_config_.Control_Word);
            b_motor_->Motor_Mode_Control(motorB_v_config_.Motor_ID1, motorB_v_config_.Work_Mode,
                                         motorB_v_config_.Control_Word);
            break;
        }
        default:
            // ERROR HINTS
            break;
    }
}

bool DoubleMotor::writeRPM(const MotorID& id, const int& rpm, const int& rpm_b) {
    enableVelocityMode(id);
    fastWriteRPM(id,rpm,rpm_b);
}

bool DoubleMotor::fastWriteRPM(const MotorID &id, const int &rpm, const double &rpm_b) {
    switch (id) {
        case 'a': {
            a_motor_->Motor_Speed_Control(motorA_v_config_.Motor_ID2, rpm);
            break;
        }
        case 'b': {
            b_motor_->Motor_Speed_Control(motorB_v_config_.Motor_ID2, rpm);
            break;
        }
        case ':': {
            a_motor_->Motor_Speed_Control(motorA_v_config_.Motor_ID2, rpm);
            b_motor_->Motor_Speed_Control(motorB_v_config_.Motor_ID2, rpm_b);
            break;
        }
        default:
            // ERROR HINTS
            break;
    }
}

bool DoubleMotor::stop(const MotorID &id) {
    fastWriteRPM(id,0,0);
}
