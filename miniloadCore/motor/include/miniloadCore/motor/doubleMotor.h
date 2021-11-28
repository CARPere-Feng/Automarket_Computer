//
// Created by miniload on 2021/11/17.
//

#ifndef MINILOADCORE_MOTOR_DOUBLEMOTOR_H
#define MINILOADCORE_MOTOR_DOUBLEMOTOR_H

#include <memory>

#include "../../../../common/include/miniloadCore/common/data_type.h"
#include "../../../../communication/include/miniloadCore/communication/can_drive.h"
#include "motor_control.h"

class DoubleMotor {
public:
    // NO COPY, NO MOVE, NO ASSIGN
    DoubleMotor();
    ~DoubleMotor();

    using MotorID = char;   // 'a' = motor a; 'b' = motor b; ':' = both;

    // stop both motors without disable the motor
    bool stop(const MotorID& id);


    bool disableVelocityMode(const MotorID& id);
    bool enableVelocityMode(const MotorID& id);

    // if only one motor rpm should be write, then set rpm
    // if two motors should be write, set rpm as speed of a, rpm_b as b
    bool writeRPM(const MotorID& id, const int& rpm, const int& rpm_b = 0);

    // if mode has been set, this can be used.
    // This function is the best to use as rpm writing
    bool fastWriteRPM(const MotorID& id, const int& rpm, const double& rpm_b = 0);

    // Unit: m/s
    bool writeVel(const MotorID& id, const double& vel);

    std::shared_ptr<can_communication> CAN_connector_;  // Thread safe
    std::unique_ptr<Motor_Control> a_motor_;
    std::unique_ptr<Motor_Control> b_motor_;
private:
    struct MotorASpeedMode {
        INT16 Motor_ID1 = 0x202;  // mode
        INT16 Motor_ID2 = 0x302;  // speed
        BYTE Control_Word[2] = {0x0f, 0x00};
        BYTE Control_Word_Stop[2] = {0x06, 0x00};
        BYTE Work_Mode = 0xfd;  // -3
        int Speed;

    } motorA_v_config_;
    struct MotorBSpeedMode {
        INT16 Motor_ID1 = 0x201;  // mode
        INT16 Motor_ID2 = 0x301;  // speed
        BYTE Control_Word[2] = {0x0f, 0x00};
        BYTE Control_Word_Stop[2] = {0x06, 0x00};

        BYTE Work_Mode = 0xfd;  // -3
        int Speed;

    } motorB_v_config_;
};  // DoubleMotor

#endif //MINILOADCORE_MOTOR_DOUBLEMOTOR_H
