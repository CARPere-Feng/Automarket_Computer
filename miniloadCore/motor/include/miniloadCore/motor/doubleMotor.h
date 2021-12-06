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

    // 'a' = motor a; 'b' = motor b; ':' = both;
    using MotorID = char;

    // stop both motors without disable the motor
    bool setVelPara2Zero(const MotorID& id);
    // disable the motor, no current will pass through motor after being disabled
    // however, do not disable motor when the motor velocity is not 0
    void disableMotors(const MotorID& id);

    bool disableVelocityMode(const MotorID& id);
    bool enableVelocityMode(const MotorID& id);
    bool disable_REL_DisplacementMode(const MotorID& id);
    bool enable_FastAbs_displacementMode(const MotorID& id);
    bool disable_FastAbs_displacementMode(const MotorID& id);

    // write reletive position moving target
    bool write_REL_inc(const MotorID& id, const int& inc, const int& inc_b = 0);

    // if mode has been set, this can be used.
    // This function is the best to use as rpm writing
    bool fastWriteRPM(const MotorID& id, const int& rpm, const int & rpm_b = 0) const;
    // write fast abs position moving target
    bool fastWrite_FastAbs_inc(const MotorID& id, const int& inc, const int& inc_b = 0);

    // receive state info from motors (A,B)
    // thread safe
    virtual bool Motor_Feedback();

    // go to the origin point
    virtual bool goOrigin(const int& threshold);

    // a and b motor should be set splitely,Unit: rps/s
    bool setPosModeAcc(const MotorID& id, const long long& posAcc, const long long& negAcc);
    void moveToPosition(const int& targeta, const int& targetb, const int& threshold);

    double a_vel_rpm_;
    int a_vel_inc_;
    double a_dis_rot_;
    int a_dis_inc_;
    double b_vel_rpm_;
    int b_vel_inc_;
    double b_dis_rot_;
    int b_dis_inc_;
    std::shared_ptr<can_communication> CAN_connector_;  // Thread safe
    std::unique_ptr<Motor_Control> a_motor_;
    std::unique_ptr<Motor_Control> b_motor_;
private:
    // if only one motor rpm should be write, then set rpm
    // if two motors should be write, set rpm as speed of a, rpm_b as b
    bool writeRPM(const MotorID& id, const int& rpm, const int& rpm_b = 0);
    bool motorPowerOff(const MotorID& id) const;
    bool setPosPara2Zero(const MotorID& id)const;
    bool enable_REL_displacementMode(const MotorID& id);
    void delayus(const int& us);
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
    struct MotorAPosMode {
        INT16 Motor_ID1 = 0x202;    // mode
        INT16 Motor_ID2 = 0x402;    // pos and vel
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

    } motorA_q_config_;
    struct MotorBPosMode {
        INT16 Motor_ID1 = 0x201;    // mode
        INT16 Motor_ID2 = 0x401;    // pos and vel
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

    } motorB_q_config_;
    INT16 motorA_acc_configID_ = 0x502;
    INT16 motorB_acc_configID_ = 0x501;

    hex2int32 vel_rec_buffer_;  // velocity receive buffer
    hex2int32 dis_rec_buffer_;  // displacement receive buffer

    const int maxTrapezoidVel_ = 100;   //UNIT: rpm
};  // DoubleMotor

#endif //MINILOADCORE_MOTOR_DOUBLEMOTOR_H
