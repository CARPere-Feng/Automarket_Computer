#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <memory>

#include"../../../../communication/include/miniloadCore/communication/can_drive.h"
#include "../../../../communication/include/miniloadCore/communication/ICANCmd.h"
#include "../../../../common/include/miniloadCore/common/data_type.h"

class Motor_Control
{
public:
	explicit  Motor_Control(std::shared_ptr<can_communication>&);
	~Motor_Control();

public:
	bool Motor_PDO_Open();
	void Motor_Speed_Control(INT16 Motor_RPDO_ID, int Motor_Speed);
	void Motor_Mode_Control(INT16 Motor_RPDO_ID,BYTE WorkMode,BYTE CONTROL_Word[2]);
	void Motor_Lift_Control(INT16 Motor_RPDO_ID,int Target_Position,int Lift_Trapezoid_Speed);
	virtual bool Motor_Feedback();
    void Can_Close();
    void Can_Start();
    void Dec2HexVector(BYTE *data_vec, const int &dec_value, const int &len);
    void Set_Speed_Boundary(const int& upper, const int& lower);

    double left_realtime_Speed;
	double left_dis;
	double left_dis_value;
    double right_realtime_Speed;
	double right_dis;
	double right_dis_value;
    hex2int speed_change;
	hex2int_dis dis_wheel;
private:
    virtual bool is_Speed_Safe(int& vel);

	BYTE PDO_Open[2];
	int encoder_num = 65536;
    int up_vel_ = 50; // unit: rpm
    int low_vel_ = -50; // unit: rpm
    const std::shared_ptr<can_communication>& CAN_obj_;
};

#endif
