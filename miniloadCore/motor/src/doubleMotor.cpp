//
// Created by Junda on 2021/11/17.
//

#include "../include/miniloadCore/motor/doubleMotor.h"
#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>

DoubleMotor::DoubleMotor() :
    CAN_connector_(std::make_shared<can_communication>()),
    a_motor_(std::make_unique<Motor_Control>(CAN_connector_)),
    b_motor_(std::make_unique<Motor_Control>(CAN_connector_)){
    if (a_motor_->Motor_PDO_Open() || b_motor_->Motor_PDO_Open()){
    } else {
        std::cerr << "motor PDO Open Failed" << std::endl;
    }
}

DoubleMotor::~DoubleMotor() {
    a_motor_.reset();
    b_motor_.reset();
    CAN_connector_.reset();
}

bool DoubleMotor::motorPowerOff(const MotorID &id) const {
    BYTE CW_stop[2] = {0x06, 0x00};
    switch (id) {
        case 'a': {
            a_motor_->Motor_Mode_Control(0x202, 0xfc,CW_stop);  // work mode: -4
            break;
        }
        case 'b': {
            b_motor_->Motor_Mode_Control(0x201, 0xfc,CW_stop);
            break;
        }
        case ':': {
            a_motor_->Motor_Mode_Control(0x202, 0xfc,CW_stop);
            b_motor_->Motor_Mode_Control(0x201, 0xfc,CW_stop);
            break;
        }
        default:
            // ERROR HINTS
            return false;
            break;
    }
    return true;
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
            return false;
            break;
    }
    return true;
}

bool DoubleMotor::enableVelocityMode(const MotorID &id) {
    if (setVelPara2Zero(id)){
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
                return false;
                break;
        }
    }else{
        return false;
    }

    return true;
}

bool DoubleMotor::enable_REL_displacementMode(const MotorID &id) {
        switch (id) {
            case 'a': {
                a_motor_->Motor_Mode_Control(motorA_q_config_.Motor_ID1, motorA_q_config_.Work_Mode,
                                             motorA_q_config_.CW_Relative1);
                a_motor_->Motor_Mode_Control(motorA_q_config_.Motor_ID1, motorA_q_config_.Work_Mode,
                                             motorA_q_config_.CW_Relative2);
                break;
            }
            case 'b': {
                b_motor_->Motor_Mode_Control(motorB_q_config_.Motor_ID1, motorB_q_config_.Work_Mode,
                                             motorB_q_config_.CW_Relative1);
                b_motor_->Motor_Mode_Control(motorB_q_config_.Motor_ID1, motorB_q_config_.Work_Mode,
                                             motorB_q_config_.CW_Relative2);
                break;
            }
            case ':': {
                a_motor_->Motor_Mode_Control(motorA_q_config_.Motor_ID1, motorA_q_config_.Work_Mode,
                                             motorA_q_config_.CW_Relative1);
                a_motor_->Motor_Mode_Control(motorA_q_config_.Motor_ID1, motorA_q_config_.Work_Mode,
                                             motorA_q_config_.CW_Relative2);
                b_motor_->Motor_Mode_Control(motorB_q_config_.Motor_ID1, motorB_q_config_.Work_Mode,
                                             motorB_q_config_.CW_Relative1);
                b_motor_->Motor_Mode_Control(motorB_q_config_.Motor_ID1, motorB_q_config_.Work_Mode,
                                             motorB_q_config_.CW_Relative2);
                break;
            }
            default:
                // ERROR HINTS
                return false;
                break;
        }
    return true;
}

bool DoubleMotor::disable_REL_DisplacementMode(const MotorID& id) {
    setPosPara2Zero(id);
    return motorPowerOff(id);
}

bool DoubleMotor::enable_FastAbs_displacementMode(const MotorID &id) {
    if (setPosPara2Zero(id)) {
        switch (id) {
            case 'a':{
                a_motor_->Motor_Mode_Control(motorA_q_config_.Motor_ID1,motorA_q_config_.Work_Mode,
                                             motorA_q_config_.CW_Abs_Fast);
                break;
            }
            case 'b':{
                b_motor_->Motor_Mode_Control(motorB_q_config_.Motor_ID1,motorB_q_config_.Work_Mode,
                                             motorB_q_config_.CW_Abs_Fast);
                break;
            }
            case ':':{
                a_motor_->Motor_Mode_Control(motorA_q_config_.Motor_ID1,motorA_q_config_.Work_Mode,
                                             motorA_q_config_.CW_Abs_Fast);
                b_motor_->Motor_Mode_Control(motorB_q_config_.Motor_ID1,motorB_q_config_.Work_Mode,
                                             motorB_q_config_.CW_Abs_Fast);
                break;
            }
            default:
                return false;
        }
    } else {
        return false;
    }
    return true;
}

bool DoubleMotor::disable_FastAbs_displacementMode(const MotorID &id) {
    return motorPowerOff(id);
}

bool DoubleMotor::write_REL_inc(const MotorID &id, const int &inc, const int &inc_b) {
    switch (id) {
        case 'a': {
            a_motor_->Motor_Pos_Control(motorA_q_config_.Motor_ID2,inc,maxTrapezoidVel_);
            break;
        }
        case 'b': {
            b_motor_->Motor_Pos_Control(motorB_q_config_.Motor_ID2,inc,maxTrapezoidVel_);
            break;
        }
        case ':': {
             long long v_a, v_b, x_a, x_b, delta_xa, delta_xb;
             long double com;
             x_a = inc; x_b = inc_b;
            Motor_Feedback();
            delta_xa = std::abs(x_a - a_dis_inc_);
            delta_xb = std::abs(x_b - b_dis_inc_);
//            std::cout << "delta_xa:" << delta_xa << std::endl;
//            std::cout << "delta_xb" << delta_xb << std::endl;
            long double temp = std::sqrt(delta_xa*delta_xa + delta_xb*delta_xb);
            // if the next move is near the current position, do nothing
            if (temp > 10) {
                com = maxTrapezoidVel_ / temp;
                v_a = com * delta_xa; // rpm
                v_b = com * delta_xb; // rpm
                a_motor_->Motor_Pos_Control(motorA_q_config_.Motor_ID2, x_a, v_a);
                b_motor_->Motor_Pos_Control(motorB_q_config_.Motor_ID2, x_b, v_b);
            }
             break;
        }
        default:
            return false;
    }

    if(enable_REL_displacementMode(id)) return true;
    else {
        return false;
    }
}

bool DoubleMotor::writeRPM(const MotorID& id, const int& rpm, const int& rpm_b) {
    if (enableVelocityMode(id)) {
        return fastWriteRPM(id,rpm,rpm_b);
    }
    return false;
}

bool DoubleMotor::fastWriteRPM(const MotorID &id, const int &rpm, const int &rpm_b) const {
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
            return false;
            break;
    }
    return true;
}

bool DoubleMotor::fastWrite_FastAbs_inc(const MotorID &id, const int &inc, const int &inc_b) {
    switch (id) {
        case 'a': {
            a_motor_->Motor_Pos_Control(motorA_q_config_.Motor_ID2,inc,maxTrapezoidVel_);
            break;
        }
        case 'b': {
            b_motor_->Motor_Pos_Control(motorB_q_config_.Motor_ID2,inc,maxTrapezoidVel_);
            break;
        }
        case ':': {
            long long v_a, v_b, x_a, x_b, delta_xa, delta_xb;
            long double com;
            x_a = inc; x_b = inc_b;
            Motor_Feedback();
            delta_xa = std::abs(x_a - a_dis_inc_);
            delta_xb = std::abs(x_b - b_dis_inc_);
            long double temp = std::sqrt(delta_xa*delta_xa + delta_xb*delta_xb);
            // if the next move is near the current position, do nothing
            if (temp > 1000) {
                com = maxTrapezoidVel_/temp;
                v_a = com*delta_xa; // rpm
                v_b = com*delta_xb; // rpm
//                std::cout << "delta_xa:" << delta_xa << "\tv_a:" << v_a << std::endl;
//                std::cout << "delta_xb:" << delta_xb << "\tv_b:" << v_b << std::endl;
                a_motor_->Motor_Pos_Control(motorA_q_config_.Motor_ID2, x_a, v_a);
                b_motor_->Motor_Pos_Control(motorB_q_config_.Motor_ID2, x_b, v_b);
            }
            break;
        }
        default:
            return false;
    }
    return true;
}

bool DoubleMotor::setVelPara2Zero(const MotorID &id) {
    switch (id) {
        case ':':{
            a_motor_->Motor_Speed_Control(motorA_v_config_.Motor_ID2,0);
            b_motor_->Motor_Speed_Control(motorB_v_config_.Motor_ID2,0);
            break;
        }
        case 'a':{
            a_motor_->Motor_Speed_Control(motorA_v_config_.Motor_ID2,0);
            break;
        }
        case 'b':{
            b_motor_->Motor_Speed_Control(motorB_v_config_.Motor_ID2,0);
            break;
        }
        default:{
            motorPowerOff(id);
            return false;
        }
    }
    return true;
}

void DoubleMotor::disableMotors(const MotorID &id) {
    motorPowerOff(id);
}

bool DoubleMotor::Motor_Feedback() {
    // BYTE data[2];
    // CAN_connector_->Can_Channel_Send(0x80, 0, data);

    int count = 0;
    bool flag_tmp[2];
    flag_tmp[0] = false;
    flag_tmp[1] = false;
    std::vector<int> a_dis_list, b_dis_list;
    std::vector<double> a_vel_list, b_vel_list;
    std::shared_lock locker(CAN_connector_->cdf_i_mutex_);
    auto rec_count = CAN_connector_->effective_rec_count_;
    //std::cout << "===received counts in feedback:\t" << rec_count << std::endl;
    while (rec_count < 4){
        rec_count = CAN_connector_->effective_rec_count_;
        //std::cout << "===received counts in feedback:\t" << rec_count << std::endl;
    }
    a_dis_rot_ = 0;
    b_dis_rot_ = 0;
    for (int i = 0; i < rec_count; i++) {
        if (CAN_connector_->Cdf_i[i].uID == 0x282)  // 0x282
        {
//                if (flag_tmp[0]) {
//                    count++;
//                    continue;
//                }
            vel_rec_buffer_.hexVal[0] = CAN_connector_->Cdf_i[i].arryData[0];
            vel_rec_buffer_.hexVal[1] = CAN_connector_->Cdf_i[i].arryData[1];
            vel_rec_buffer_.hexVal[2] = CAN_connector_->Cdf_i[i].arryData[2];
            vel_rec_buffer_.hexVal[3] = CAN_connector_->Cdf_i[i].arryData[3];
            dis_rec_buffer_.hexVal[0] = CAN_connector_->Cdf_i[i].arryData[4];
            dis_rec_buffer_.hexVal[1] = CAN_connector_->Cdf_i[i].arryData[5];
            dis_rec_buffer_.hexVal[2] = CAN_connector_->Cdf_i[i].arryData[6];
            dis_rec_buffer_.hexVal[3] = CAN_connector_->Cdf_i[i].arryData[7];
            a_vel_inc_ = vel_rec_buffer_.integer32;
            a_dis_inc_ = dis_rec_buffer_.integer32;
            a_dis_list.push_back(a_dis_inc_);

            // std::cout << std::hex << "0x" << CAN_connector_->Cdf_i[i].uID << "  : ";
            // for (size_t j = 0; j < 8; j++) {
            //   std::cout << std::hex << "0x" << (int)CAN_connector_->Cdf_i[i].arryData[j]
            //             << "  ";
            // }
            // std::cout << std::endl;

            a_dis_rot_ = (double)a_dis_inc_ / (double)a_motor_->getEncoderResolution();
            // ROS_WARN("left dis : %.6f", left_dis);
            /*for(i=0;i<4;i++)
            {
                    std::cout<<std::to_string(vel_rec_buffer_.hexVal[i])<<std::endl;
            }*/
            a_vel_rpm_ = ((a_vel_inc_ * 1875.0) / (a_motor_->getEncoderResolution() * 512.0 ));
            a_vel_list.push_back(a_vel_rpm_);

            // std::cout<<CAN_connector_->Cdf_i[i].uID<<std::endl;
            // std::cout<<i<<std::endl;
            flag_tmp[0] = true;
            count++;
        } else if (CAN_connector_->Cdf_i[i].uID == 0x281)  // 0x281
        {
//                if (flag_tmp[1]) {
//                    count++;
//                    continue;
//                }
            vel_rec_buffer_.hexVal[0] = CAN_connector_->Cdf_i[i].arryData[0];
            vel_rec_buffer_.hexVal[1] = CAN_connector_->Cdf_i[i].arryData[1];
            vel_rec_buffer_.hexVal[2] = CAN_connector_->Cdf_i[i].arryData[2];
            vel_rec_buffer_.hexVal[3] = CAN_connector_->Cdf_i[i].arryData[3];
            dis_rec_buffer_.hexVal[0] = CAN_connector_->Cdf_i[i].arryData[4];
            dis_rec_buffer_.hexVal[1] = CAN_connector_->Cdf_i[i].arryData[5];
            dis_rec_buffer_.hexVal[2] = CAN_connector_->Cdf_i[i].arryData[6];
            dis_rec_buffer_.hexVal[3] = CAN_connector_->Cdf_i[i].arryData[7];
            b_vel_inc_ = vel_rec_buffer_.integer32;
            b_dis_inc_ = dis_rec_buffer_.integer32;
            b_dis_list.push_back(b_dis_inc_);

            // std::cout << std::hex << "0x" << CAN_connector_->Cdf_i[i].uID << "  : ";
            // for (size_t j = 0; j < 8; j++) {
            //   std::cout << std::hex << "0x" << (int)CAN_connector_->Cdf_i[i].arryData[j]
            //             << "  ";
            // }
            // std::cout << std::endl;

            b_dis_rot_ = (double)b_dis_inc_ / (double)b_motor_->getEncoderResolution();
            /*for(i=0;i<4;i++)
            {
                    std::cout<<std::to_string(vel_rec_buffer_.hexVal[i])<<std::endl;
            }*/
            b_vel_rpm_ = (b_vel_inc_ * 1875.0) / (b_motor_->getEncoderResolution() * 512.0 );
            b_vel_list.push_back(b_vel_rpm_);
            // std::cout<<CAN_connector_->Cdf_i[i].uID<<std::endl;
            //std::cout<<"right_real_speed:"<<std::to_string(right_realtime_Speed)<<std::endl;
            // std::cout<<i<<std::endl;

            flag_tmp[1] = true;
            count++;
        }
    }

    locker.unlock();
    a_dis_inc_ = a_dis_list[0];
    b_dis_inc_ = b_dis_list[0];
    std::cout << "===unlock===\t" << a_dis_inc_ << "\t==\t" << b_dis_inc_ << std::endl;

    // ROS_INFO("data quantity for computation : %d", count);
    if (flag_tmp[0] && flag_tmp[1]) {
        return true;
    } else {
        std::cerr << "one of the motor does not feedback datas!" << std::endl;
        return false;
    }
}

bool DoubleMotor::setPosModeAcc(const MotorID &id, const long long &posAcc, const long long &negAcc){
    assert(posAcc > 0 && negAcc > 0 && posAcc < 25 && negAcc < 25);
    hex2int32 tempacc1,tempacc2;
    if (id == 'a') {
        tempacc1.integer32 = posAcc*65536*(long long)(a_motor_->getEncoderResolution())/4000000;
        tempacc2.integer32 = negAcc*65536*(long long)(a_motor_->getEncoderResolution())/4000000;
        a_motor_->Motor_Acc_Control(motorA_acc_configID_,tempacc1,tempacc2);
    } else if(id == 'b') {
        tempacc1.integer32 = posAcc*65536*(long long)(b_motor_->getEncoderResolution())/4000000;
        tempacc2.integer32 = negAcc*65536*(long long)(b_motor_->getEncoderResolution())/4000000;
        b_motor_->Motor_Acc_Control(motorB_acc_configID_,tempacc1,tempacc2);
    }else {
        std::cerr << "Acceleration setting in position mode is failed, you should only set one motor per time!" << std::endl;
        return false;
    }
    return true;
}

bool DoubleMotor::goOrigin() {
    if (enable_FastAbs_displacementMode(':')) {
        fastWrite_FastAbs_inc(':',0,0);
        return true;
    } else
        return false;
}

bool DoubleMotor::setPosPara2Zero(const MotorID &id) const{
    switch (id) {
        case 'a':{
            a_motor_->Motor_Pos_Control(motorA_q_config_.Motor_ID2,0,0);
            break;
        }
        case 'b':{
            b_motor_->Motor_Pos_Control(motorB_q_config_.Motor_ID2,0,0);
            break;
        }
        case ':':{
            a_motor_->Motor_Pos_Control(motorA_q_config_.Motor_ID2,0,0);
            b_motor_->Motor_Pos_Control(motorB_q_config_.Motor_ID2,0,0);
            break;
        }
        default: {
            return false;
        }
    }
    return true;
}

void DoubleMotor::delayus(const int &us) {
    for (int i = 0; i < us; ++i) {
        for (int j = 0; j < 65535; ++j) {
            ;
        }
    }
}