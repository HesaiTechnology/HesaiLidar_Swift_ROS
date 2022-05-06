/******************************************************************************
 * Copyright 2019 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef FAULT_MESSAGE_H
#define FAULT_MESSAGE_H

#include <map>
#include <vector>

#define PANDAR_AT128_LIDAR_NUM (128)
#define LENS_AZIMUTH_AREA_NUM (12)
#define LENS_ELEVATION_AREA_NUM (8)
 
enum LidarOperateState{
  Boot,
  Init,
  FullPerformance,
  HalfPower,
  SleepMode,
  HighTempertureShutdown,
  FaultShutdown,
  UndefineOperateState = -1,
};

enum LidarFaultState{
  Normal,
  Warning,
  PrePerformanceDegradation,
  PerformanceDegradation,
  PreShutDown,
  ShutDown,
  PreReset,
  Reset,
  UndefineFaultState = -1,
};

enum FaultCodeType{
  UndefineFaultCode = -1,
  CurrentFaultCode = 1,
  HistoryFaultCode = 2,
};

enum DTCState{
  NoFault,
  Fault,
};

enum TDMDataIndicate{
  Invaild = 0,
  LensDirtyInfo = 1,
  UndefineIndicate = -1,
};

enum LensDirtyState{
  UndefineData = -1,
  LensNormal = 0,
  Passable = 1,
  Unpassable = 3,
};

enum HeatingState{
  Off = 0,
  Heating = 1,
  HeatingProhibit = 2,
  UndefineHeatingState = -1,
};

typedef struct AT128FaultMessageInfo {
  uint8_t m_u8Version;
  uint8_t m_u8UTCTime[6];
  uint32_t m_u32Timestamp;
  double m_dTotalTime;
  LidarOperateState m_operateState;
  LidarFaultState m_faultState;
  FaultCodeType m_faultCodeType;
  uint8_t m_u8RollingCounter;
  uint8_t m_u8TotalFaultCodeNum;
  uint8_t m_u8FaultCodeId;
  uint32_t m_u32FaultCode;
  int m_iDTCNum;
  DTCState m_DTCState;
  TDMDataIndicate m_TDMDataIndicate;
  double m_dTemperature;
  LensDirtyState m_LensDirtyState[LENS_AZIMUTH_AREA_NUM][LENS_ELEVATION_AREA_NUM];
  uint16_t m_u16SoftwareId;
  uint16_t m_u16SoftwareVersion;
  uint16_t m_u16HardwareVersion;
  uint16_t m_u16BTversion;
  HeatingState m_HeatingState;
  uint8_t m_Reversed[4];
  uint32_t m_u32CRC;
  uint8_t m_CycberSecurity[32];
} AT128FaultMessageInfo;


#pragma  pack(1) 
typedef class AT128FaultMessageVersion3_s {
  public:
    uint16_t u16Sob;
    uint8_t u8Version;
    uint8_t u8UTCTime[6];
    uint32_t u32Timestamp;
    uint8_t u8OperateState;
    uint8_t u8FaultState;
    uint8_t u8FaultCodeType;
    uint8_t u8RollingCounter;
    uint8_t u8TotalFaultCodeNum;
    uint8_t u8FaultCodeId;
    uint32_t u32FaultCode;
    uint8_t u8TimeDivisionMultiplexing[27];
    uint8_t u8SoftwareVersion[8];
    uint8_t u8HeatingState;
    uint8_t u8Reversed[4];
    uint32_t u32CRC;
    uint8_t u8CycberSecurity[32];
    DTCState ParserDTCState();
    LidarOperateState ParserOperateState();
    LidarFaultState ParserFaultState();
    FaultCodeType ParserFaultCodeType();
    TDMDataIndicate ParserTDMDataIndicate();
    void ParserLensDirtyState(LensDirtyState lensDirtyState[LENS_AZIMUTH_AREA_NUM][LENS_ELEVATION_AREA_NUM]);
    HeatingState ParserHeatingState();
    void ParserAT128FaultMessage(AT128FaultMessageInfo &faultMessageInfo);
    double ParserTemperature();
} AT128FaultMessageVersion3;
#pragma pack()

#endif