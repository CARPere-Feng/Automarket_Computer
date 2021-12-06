#ifndef _CAN_DRIVE_H
#define _CAN_DRIVE_H
#include "ICANCmd.h"
#include <thread>
#include <shared_mutex>
class can_communication
{
public:
        can_communication();
        ~can_communication();
public:
        unsigned int Can_Open();
        bool Can_Close();
        bool Can_Channel_Start();
        bool Can_Channel_Stop();
        bool Can_Get_Device_Info();
        unsigned long Can_Channel_Send(UINT Send_ID,BYTE Data_length,BYTE SendMessage[]);
        bool Can_Get_Error_Info();
        bool Can_Clear_Rec_Buffer() const;
        unsigned long Can_Rec_Count()const;
        CAN_DataFrame Cdf_i[2500];
        std::shared_timed_mutex cdf_i_mutex_;
        unsigned long rec_window_;
        int effective_rec_count_;
        std::thread rec_thread_;
private:
        void Can_Channel_Receive(const unsigned long& size);
        DWORD dwDeviceHandle;
        CAN_InitConfig Cic;
        CAN_DeviceInformation Cdi;
        CAN_DataFrame Cdf_o[1];
        CAN_ErrorInformation err;
};

#endif
