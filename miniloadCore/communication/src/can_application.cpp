#include "can_drive_pkg/can_application.h"

using mobile_base::CanApplication;

CanApplication::CanApplication() {}

CanApplication::~CanApplication() {}

void CanApplication::LoadConfig(const std::string& file_address) {
  YAML::Node can_config = YAML::LoadFile(file_address);

  device_type_ = can_config["device_type"].as<int>();
  device_index_ = can_config["device_index"].as<int>();
  can_index_ = can_config["can_index"].as<int>();
  wait_time_ = can_config["wait_time"].as<int>();
  frame_len_ = can_config["frame_len"].as<int>();
}

void CanApplication::ActivateCAN(const std::string& file_address) {
  LoadConfig(file_address);

  int open_state = CAN_DeviceOpen(device_type_ , device_index_,0);//pDescription为空
  CanDiagnostic("open can device", open_state);

  CAN_InitConfig  can_init_config;
  can_init_config.dwAccCode  = 0x00000000;
  can_init_config.dwAccMask = 0xFFFFFFFF;
  can_init_config.nFilter = 0;
  can_init_config.bMode = 0;
  can_init_config.dwBtr[0] = 0x00; 
  can_init_config.dwBtr[1] = 0X1C;
  can_init_config.dwBtr[2] = 0;
  can_init_config.dwBtr[3] = 0;
  can_init_config.nBtrType = 1;
  
  int init_state =
     CAN_ChannelStart(device_type_ , can_index_ , &can_init_config); 
     dwDeviceHandle = CAN_DeviceOpen(device_type_, device_index_,0);//pDescription为空
  CanDiagnostic("init can", init_state);

  int start_state =init_state;
  CanDiagnostic("start can", start_state);
}

void CanApplication::CloseCAN() {
   CAN_DeviceClose(dwDeviceHandle );
}

void CanApplication::CanDiagnostic(const std::string& description,
                                   const int& state) {
  switch (state) {
    case 1: {
      std::cout << "Successful! " << description << std::endl;
      break;
    }
    case 0: {
      std::cout << "Failure! " << description << std::endl;
      break;
    }
    case -1: {
      std::cout << "Lose USB Error! " << description << std::endl;
      break;
    }
    default: {
      std::cout << "Incorrect state feedback of CAN!" << std::endl;
      break;
    }
  }
}

PCAN_DataFrame CanApplication::GetVciObject(const int& obj_num,
                                          const uint& initial_id) {
  PCAN_DataFrame obj_ptr;
  obj_ptr = new CAN_DataFrame[obj_num];
  for (size_t i = 0; i < obj_num; i++) {
    obj_ptr[i].uID = initial_id;
    obj_ptr[i].bRemoteFlag = 0;
    obj_ptr[i].nSendType = 0;
    obj_ptr[i].bRemoteFlag = 0;
    obj_ptr[i].bExternFlag= 0;
  }

  return obj_ptr;
}

void CanApplication::SendCommand(PCAN_DataFrame obj, const uint& obj_len,
                                 const bool& single_frame) {
  for (size_t i = 0; i < obj_len; i++) {
    if (!single_frame) {
      CAN_ChannelSend(dwDeviceHandle, can_index_, obj, frame_len_);
    } else {
      CAN_ChannelSend(dwDeviceHandle, can_index_, obj, 1);
  }
}
                                 }

void CanApplication::GetData (PCAN_DataFrame obj, const int& obj_len) {
  int data_num = CAN_GetReceiveCount(dwDeviceHandle, can_index_);
  if (-1 == data_num) {
    std::cout << "Get data number failure!" << std::endl;
  } else if (0 == data_num) {
    std::cout << "No data in the buffer" << std::endl;
  }

  int receive_num = CAN_ChannelReceive(dwDeviceHandle, can_index_, obj,
                                obj_len, wait_time_);
}
