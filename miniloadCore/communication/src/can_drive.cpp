#include <iostream>
#include <cassert>
#include <functional>
#include "../include/miniloadCore/communication/ICANCmd.h"
#include "../include/miniloadCore/communication/can_drive.h"
#include "../../common/include/miniloadCore/common/data_type.h"

#define dwType   USBCAN_C_1CH //ACUSB_131B   设备类型
#define dwIndex   0  //设备端口号 从1开始
#define dwChannel   0  //通道号

can_communication::can_communication() :  rec_window_(4),
rec_thread_(&can_communication::Can_Channel_Receive, this, rec_window_)
{
    rec_thread_.detach();
}

can_communication::~can_communication()
{
}

unsigned int can_communication::Can_Open()
{
    dwDeviceHandle = CAN_DeviceOpen(dwType, dwIndex,0);//pDescription为空
    if (dwDeviceHandle == 0)
    {
        std::cout << "open the device failure" << std::endl;
        return 0;
    }
    else 
    {
        std::cout << "open the device success" << std::endl;
         std::cout <<dwDeviceHandle << std::endl;
        return dwDeviceHandle;
     }
}

bool can_communication::Can_Close()
{
    int close_flag;
    close_flag = CAN_DeviceClose(dwDeviceHandle);
    if (close_flag == CAN_RESULT_OK)
    {
        std::cout << "close the device failure" << std::endl;
        return 1;
    }
    else if (close_flag == CAN_RESULT_ERROR)
    {
        std::cout << "close the device failure" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "error!!!!!!" << std::endl;
    }
}


bool can_communication::Can_Channel_Start()
{
    int channel_start_flag;
    Cic.bMode = 0;
    Cic.nBtrType = 1;
    Cic.dwBtr[0] = 0x00; // BTR0 BTR1   0014 -1M 0016-800K 001C-500K 011C-250K 031C-12K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
    Cic.dwBtr[1] = 0X1C;
    Cic.dwBtr[2] = 0;
    Cic.dwBtr[3] = 0;
    Cic.dwAccCode = 0xffffffff;
    Cic.dwAccMask = 0;
    Cic.nFilter = 0;
    channel_start_flag = CAN_ChannelStart(dwDeviceHandle, dwChannel, &Cic); 
    if (channel_start_flag == CAN_RESULT_OK)
    {
        std::cout << "start channel sucess" << std::endl;
        return 1;
    }
    else
    {
        CAN_DeviceClose(dwDeviceHandle);
        std::cout << "start channel failure" << std::endl;
        return 0;
    }
} 

bool can_communication::Can_Channel_Stop()
{
    int channel_stop_flag;
    channel_stop_flag = CAN_ChannelStop(dwDeviceHandle, dwChannel);
    if(channel_stop_flag == CAN_RESULT_OK)
    {
        std::cout << "stop channel sucess" << std::endl;
        return 1;
    }
    else
    {
        std::cout << "stop channel failure" << std::endl;
        return 0;
    }
}
   
bool can_communication::Can_Get_Device_Info()
{
    int get_device_flag;
    get_device_flag = CAN_GetDeviceInfo(dwDeviceHandle, &Cdi); 
    if(get_device_flag == CAN_RESULT_OK)
    {
        std::cout << "get device info sucess" << std::endl;
        return 1;
    }
    else
    {
        std::cout << "get device info failure" << std::endl;
        return 0;
    }
}

// <SendMessage> 0 is low byte, n is high byte </SendMessage>
unsigned long can_communication:: Can_Channel_Send(UINT Send_ID,BYTE Data_length,BYTE SendMessage[])
{
    Cdf_o[0].nSendType = 0;
    Cdf_o[0].bRemoteFlag = 0;
    Cdf_o[0].bExternFlag = 0;
    Cdf_o[0].nDataLen = Data_length;
    Cdf_o[0].uID = Send_ID;
    for (unsigned int j = 0; j < Data_length; j++)
        Cdf_o[0].arryData[j] = SendMessage[j];
    int channel_send_flag = CAN_ChannelSend( dwDeviceHandle, dwChannel, Cdf_o, 1);
    if(channel_send_flag != 0)
    {
        return 1;
    }
    else
    {
        std::cout << "sendmessage failure" << std::endl;
        return 0;
    }
}

void can_communication::Can_Channel_Receive(const unsigned long& size)
{
    assert(size < 2500);
    static int countrec = 0;
    while (true) {
        cdf_i_mutex_.lock();
        Cdf_i[0].bRemoteFlag = 0;
        Cdf_i[0].bExternFlag = 0;

//        // if the receive buffer doesn't have enough datas
        while(Can_Rec_Count() < size) ;

        effective_rec_count_ = CAN_ChannelReceive(dwDeviceHandle, dwChannel, Cdf_i, size);

        cdf_i_mutex_.unlock();
        //std::cout << "===received counts in can driver:\t" << effective_rec_count_ << std::endl;
        if(effective_rec_count_ == 0)
        {
            std::cout << "the device is not exisit or usb lost,you can restart can device" << std::endl;
        } else{
//            hex2int32 temp;
//            for (int i = 0; i <effective_rec_count_; ++i) {
//                for (int j = 0; j < 4; ++j) {
//                    temp.hexVal[j] = Cdf_i[i].arryData[j+4];
//                }
//                std::cout << "received:\t" << temp.integer32 << std::endl;
//            }
        }

    }
}

unsigned long can_communication::Can_Rec_Count() const {
    return CAN_GetReceiveCount(dwDeviceHandle,dwChannel);
}

bool can_communication::Can_Clear_Rec_Buffer() const {
    int channel_clear_flag = CAN_ClearReceiveBuffer(dwDeviceHandle, dwChannel);
    if (channel_clear_flag == 0) {
        std::cout << "receive buffer clearing failed!" << std::endl;
        return false;
    } else{
        return true;
    }
}


bool can_communication::Can_Get_Error_Info()
{
    std::cout<<"ErrorCode : "<<err.uErrorCode<<std::endl;
    return 0;
    /*
    if (CAN_GetErrorInfo(dwDeviceHandle, dwChannel, &err) == CAN_RESULT_OK) 
    {
        std::cout << "get error info success" << std::endl;
        std::cout<<"ErrorCode : "<<err.uErrorCode<<std::endl;
        return 1;
    }
    else
        std::cout << "get error info failure" << std::endl;
        return 0;
    */
}      




  






