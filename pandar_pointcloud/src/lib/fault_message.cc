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

#include <pandar_pointcloud/fault_message.h>
#include <stdlib.h>
#include <string.h>           
void AT128FaultMessageVersion3::ParserAT128FaultMessage(AT128FaultMessageInfo &faultMessageInfo){
  faultMessageInfo.m_u8Version = u8Version;
  memcpy(faultMessageInfo.m_u8UTCTime,u8UTCTime,sizeof(u8UTCTime));
  struct tm t = {0};
  t.tm_year  = u8UTCTime[0];
  if (t.tm_year >= 200) {
    t.tm_year -= 100;
  }
  t.tm_mon = u8UTCTime[1] - 1;
  t.tm_mday = u8UTCTime[2];
  t.tm_hour = u8UTCTime[3];
  t.tm_min = u8UTCTime[4];
  t.tm_sec = u8UTCTime[5];
  t.tm_isdst = 0;
  faultMessageInfo.m_u32Timestamp = u32Timestamp;          
  faultMessageInfo.m_dTotalTime =  mktime(&t) + (static_cast<double>(u32Timestamp)) / 1000000.0;
  faultMessageInfo.m_operateState = ParserOperateState();
  faultMessageInfo.m_faultState = ParserFaultState();
  faultMessageInfo.m_faultCodeType = ParserFaultCodeType();
  faultMessageInfo.m_u8RollingCounter = u8RollingCounter;
  faultMessageInfo.m_u8TotalFaultCodeNum = u8TotalFaultCodeNum;
  faultMessageInfo.m_u8FaultCodeId = u8FaultCodeId;
  faultMessageInfo.m_u32FaultCode = u32FaultCode;
  faultMessageInfo.m_iDTCNum = (u32FaultCode & 0x0000ffc0) >> 6;
  faultMessageInfo.m_DTCState = ParserDTCState();
  faultMessageInfo.m_TDMDataIndicate = ParserTDMDataIndicate();
  faultMessageInfo.m_dTemperature = ParserTemperature();
  ParserLensDirtyState(faultMessageInfo.m_LensDirtyState);
  faultMessageInfo.m_u16SoftwareId = *((uint16_t*)(&u8SoftwareVersion[0]));
  faultMessageInfo.m_u16SoftwareVersion = *((uint16_t*)(&u8SoftwareVersion[2]));
  faultMessageInfo.m_u16HardwareVersion = *((uint16_t*)(&u8SoftwareVersion[4]));
  faultMessageInfo.m_u16BTversion = *((uint16_t*)(&u8SoftwareVersion[6]));
  faultMessageInfo.m_HeatingState = ParserHeatingState();
  memcpy(faultMessageInfo.m_Reversed, u8Reversed, sizeof(u8Reversed));
  faultMessageInfo.m_u32CRC = u32CRC;
  memcpy(faultMessageInfo.m_CycberSecurity, u8CycberSecurity, sizeof(u8CycberSecurity));
}

LidarOperateState AT128FaultMessageVersion3::ParserOperateState(){
  switch (u8OperateState)
  {
  case 0:
    return Boot;
    break;
  case 1:
    return Init;
    break;  
  case 2:
    return FullPerformance;
    break;
  case 3:
    return HalfPower;
    break;
  case 4:
    return SleepMode;
    break;
  case 5:
    return HighTempertureShutdown;
    break;  
  case 6:
    return FaultShutdown;
    break;           
  
  default:
    return UndefineOperateState;
    break;
  }
}

LidarFaultState AT128FaultMessageVersion3::ParserFaultState(){
  switch (u8FaultState)
  {
  case 0:
    return Normal;
    break;
  case 1:
    return Warning;
    break;  
  case 2:
    return PrePerformanceDegradation;
    break;
  case 3:
    return PerformanceDegradation;
    break;
  case 4:
    return PreShutDown;
    break;
  case 5:
    return ShutDown;
    break;  
  case 6:
    return PreReset;
    break;    
  case 7:
    return Reset;
    break;           
  
  default:
    return UndefineFaultState;
    break;
  }

}
FaultCodeType AT128FaultMessageVersion3::ParserFaultCodeType(){
  switch (u8FaultCodeType)
  {
  case 1:
    return CurrentFaultCode;
    break;
  case 2:
    return HistoryFaultCode;
    break;           
  
  default:
    return UndefineFaultCode;
    break;
  }

}
TDMDataIndicate AT128FaultMessageVersion3::ParserTDMDataIndicate(){
  switch (u8TimeDivisionMultiplexing[0])
  {
  case 0:
    return Invaild;
    break;
  case 1:
    return LensDirtyInfo;
    break;           
  
  default:
    return UndefineIndicate;
    break;
  }

}
HeatingState AT128FaultMessageVersion3::ParserHeatingState(){
  switch (u8HeatingState)
  {
  case 0:
    return Off;
    break;
  case 1:
    return Heating;
    break;  
  case 2:
    return HeatingProhibit;
    break;           
  
  default:
    break;
  }
  return UndefineHeatingState;
}

double AT128FaultMessageVersion3::ParserTemperature(){
  double temp = ((double)(*((uint16_t*)(&u8TimeDivisionMultiplexing[1])))) * 0.1f;
  return temp;
}

void AT128FaultMessageVersion3::ParserLensDirtyState(LensDirtyState lensDirtyState[LENS_AZIMUTH_AREA_NUM][LENS_ELEVATION_AREA_NUM]){
  for(int i = 0; i < LENS_AZIMUTH_AREA_NUM; i++){
    uint16_t rawdata = (*((uint16_t*)(&u8TimeDivisionMultiplexing[3 + i * 2])));
    for(int j = 0; j < LENS_ELEVATION_AREA_NUM; j++){
      uint16_t lensDirtyStateTemp = (rawdata << ((LENS_ELEVATION_AREA_NUM - j - 1) * 2)) ;
      uint16_t lensDirtyStateTemp1 = (lensDirtyStateTemp >> ((LENS_ELEVATION_AREA_NUM - 1) * 2)) ;
      if(u8TimeDivisionMultiplexing[0]  == 1){
        switch (lensDirtyStateTemp1)
        {
        case 0:
        {
          lensDirtyState[i][j] = LensNormal;
          break;
        }
        case 1:
        {
          lensDirtyState[i][j] = Passable;
          break;
        }
        case 3:
        {
          lensDirtyState[i][j] = Unpassable;
          break;
        }  
        default:
          lensDirtyState[i][j] = UndefineData;
          break;
        }

      }
      else
      lensDirtyState[i][j] = UndefineData;
    }
    
  }
}

DTCState AT128FaultMessageVersion3::ParserDTCState(){
  switch (u32FaultCode & 0x01)
  {
  case 1:
  {
    return Fault;
    break;
  }
  case 0:
  {
    return NoFault;
    break;
  }  
  
  default:
    break;
  }
  return NoFault;
}
