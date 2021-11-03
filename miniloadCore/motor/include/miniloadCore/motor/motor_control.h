#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include"../../../../communication/include/miniloadCore/communication/can_drive.h"
#include "../../../../communication/include/miniloadCore/communication/ICANCmd.h"
#define INT16 unsigned int

#define	PI	 3.1415926535897932

union hex2int
{
	int real_speed;
    unsigned char real_time_speed[4];
	
};
union hex2int_dis
{
	int dis_moved;
	unsigned char dis[4];
};

class Motor_Control
{
public:
	Motor_Control();
	~Motor_Control();

public:
	bool Motor_PDO_Open();
	void Motor_Speed_Control(INT16 Motor_RPDO_ID, int Motor_Speed);
	void Motor_Mode_Control(INT16 Motor_RPDO_ID,BYTE WorkMode,BYTE CONTROL_Word[2]);
	void Motor_Lift_Control(INT16 Motor_RPDO_ID,int Target_Position,int Lift_Trapezoid_Speed);
	bool Motor_Feedback();
    void Can_Close();
    void Can_Start();
    void Dec2HexVector(BYTE *data_vec, const int &dec_value, const int &len);

    double left_realtime_Speed;
	double left_dis;
	double left_dis_value;
    double right_realtime_Speed;
	double right_dis;
	double right_dis_value;
    hex2int speed_change;
	hex2int_dis dis_wheel;
private:
	int MotorID_Num1=0x201;
	int MotorID_Num2=0x202;
	BYTE PDO_Open[2];	
	int encoder_num = 65536;


};

#endif
