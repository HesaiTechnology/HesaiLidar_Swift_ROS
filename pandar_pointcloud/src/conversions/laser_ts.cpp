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

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include "laser_ts.h"
#include <fstream>
#include <sstream> 
#include <iostream>
#include <boost/algorithm/string.hpp>

#define OFFSET1   (3148)
#define OFFSET2   (-27778)
#define PAI       (3.14159265358979323846)
#define SEC_TO_NS (1000000000.0)
#define SPEED     (600.0 / SEC_TO_NS)
#define A_TO_R    (PAI / 180.0)
#define R_TO_A    (180.0 / PAI)

#define PANDAR80_BLOCK_TIMESTAMP (33.33)
#define BLOCK_ID_MAX (3)
#define SEC_TO_US (1000000.0)
#define SPEED_US  (3600.0 / SEC_TO_US)

#define PANDAR128_COORDINATE_CORRECTION_H (0.04)
#define PANDAR128_COORDINATE_CORRECTION_B (0.012)

LasersTSOffset::LasersTSOffset() {
  mBInitFlag = false;
  mNLaserNum = 0;

  for (int j = 0; j < CIRCLE; j++) {
    float angle = static_cast<float>(j) / 100.0f;

    mSinAllAngleHB[j] = sinf(A_TO_R * angle) * sqrtf(PANDAR128_COORDINATE_CORRECTION_B * PANDAR128_COORDINATE_CORRECTION_B  + PANDAR128_COORDINATE_CORRECTION_H * PANDAR128_COORDINATE_CORRECTION_H);
    mSinAllAngleH[j] = sinf(A_TO_R * angle) * PANDAR128_COORDINATE_CORRECTION_H;
  }

  for (int j = 0; j < PAI_ANGLE; j++) {
    mArcSin[j] = asinf(float(j - HALF_PAI_ANGLE) / HALF_PAI_ANGLE);
  }

  mShortOffsetIndex.resize(100);
  mLongOffsetIndex.resize(100);
  m_fArctanHB = atanf(PANDAR128_COORDINATE_CORRECTION_H / PANDAR128_COORDINATE_CORRECTION_B) + 0.5f;
}

LasersTSOffset::~LasersTSOffset() {

}

void LasersTSOffset::setFilePath(std::string file) {

  std::ifstream fin(file);
  std::string line;
  if (std::getline(fin, line)) { //first line sequence,chn id,firetime/us
    printf("Parse Lidar firetime now...\n");
  }
  std::vector<std::string>  firstLine;
  boost::split(firstLine, line, boost::is_any_of(","));
  if (firstLine.size() == 2){
    while (std::getline(fin, line)) {
      int sequence = 0;
      int chnId = 0;
      float firetime;
      std::stringstream ss(line);
      std::string subline;
      // std::getline(ss, subline, ',');
      // std::stringstream(subline) >> sequence;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> chnId;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> firetime;
      // printf("seq, chnId, firetime:[%d][%d][%f]\n",sequence, chnId, firetime);
      m_fAzimuthOffset[chnId - 1] = firetime;
    }
  }
  else{
    fin.close();
    FILE             *pFile       = fopen(file.c_str(), "r");
    char             content[512] = {0};
    char             subStr[255]  = {0};
    std::vector<int> mode;
    std::vector<int> state;

    if (NULL == pFile) {
      return;
    }

    fgets(content, 512, pFile);
    strncpy(subStr, content, strchr(content, ',') - content);
    sscanf(subStr, "%f", &mFDist);
    memset(subStr, 0, 255);
    memset(content, 0, 512);

    fgets(content, 512, pFile);
    fillVector(strchr(content, ','), strlen(strchr(content, ',')), mode);
    memset(content, 0, 512);

    fgets(content, 512, pFile);
    fillVector(strchr(content, ','), strlen(strchr(content, ',')), state);
    memset(content, 0, 512);

    if (mode.size() != state.size()) {
      printf("file format error\n");
      return;
    }

    for (int i = 0; i < mode.size(); i++) {
      int index = mode[i] * 10 + state[i];
      if (i % 2 == 0) {
        mLongOffsetIndex[index] = i;
      } else {
        mShortOffsetIndex[index] = i;
      }
    }

    while (!feof(pFile)) {
      fgets(content, 512, pFile);

      std::vector<int> values;
      char             *pValues = strchr(content, ',');

      if (NULL == pValues) {
        break;
      }

      strncpy(subStr, content, pValues - content);
      sscanf(subStr, "%d", &mNLaserNum);
      fillVector(pValues, strlen(pValues), values);

      if (values.size() != mode.size()) {
        continue;
      }

      mVLasers.push_back(values);
    }

    mBInitFlag = true;

  }
}

void LasersTSOffset::fillVector(char *pContent, int nLen, std::vector<int> &vec) {
  char *pNext = strtok(pContent, ",");

  while (pNext != NULL) {
    if (pNext != NULL && strlen(pNext) > 0) {
      vec.push_back(atoi(pNext));
    }

    pNext = strtok(NULL, ",");
  }
}

float LasersTSOffset::getTSOffset(int nLaser, int nMode, int nState, float fDistance, int nMajorVersion) {
  switch (nMajorVersion){
    case 1:
      if (nLaser >= mNLaserNum || !mBInitFlag) {
        return 0;
      }
      if (fDistance >= mFDist) {
        return mVLasers[nLaser][mLongOffsetIndex[nMode * 10 + nState]];
      } 
      else {
        return mVLasers[nLaser][mShortOffsetIndex[nMode * 10 + nState]];
      }
    case 3:
      return m_fAzimuthOffset[nLaser];
    default:
      return 0;
  }
}

int LasersTSOffset::getBlockTS(int nBlock, int nRetMode, int nMode, int nLaserNum) {
  switch (nLaserNum){
    case PANDAR80_LIDAR_NUM:
      if (0x39 == nRetMode || 0x3b == nRetMode || 0x3c == nRetMode) {
        return (((BLOCK_ID_MAX - nBlock)/2) * PANDAR80_BLOCK_TIMESTAMP);
      } 
      else {
        return ((BLOCK_ID_MAX - nBlock) * PANDAR80_BLOCK_TIMESTAMP);
      }
    default:
      int ret = OFFSET1;
      if (nRetMode != 0x39) {
        if (nBlock % 2 == 0) {
          ret += OFFSET2;
        }
        if (nMode != 0) {
          ret += OFFSET2;
        }
      }
      return ret;
  }
}

float LasersTSOffset::getAngleOffset(float nTSOffset, int speed, int nMajorVersion) {
  switch (nMajorVersion){
    case 1:
      return nTSOffset * speed * 6E-9;
    case 3:
      return nTSOffset * speed * 6E-6;
    default:
      return 0;
  }
}


float LasersTSOffset::getAzimuthOffset(std::string type, float azimuth, \
    float originAzimuth, float distance) {
  int  angle = static_cast<int>(100 * (m_fArctanHB + azimuth - originAzimuth));

  if (angle < 0) {
    angle += CIRCLE;
  } else if (angle >= CIRCLE) {
    angle -= CIRCLE;
  }

  if (distance < 0.00001) {
    return 0;
  }

  float value = mSinAllAngleHB[angle] / distance;
  if(value < -1 || value > 1)
    return 0;
  int index = int(value * HALF_PAI_ANGLE) + HALF_PAI_ANGLE;
  if(index < 0 || index > PAI_ANGLE -1)
    return 0;
  return -mArcSin[index];
}

float LasersTSOffset::getPitchOffset(std::string type, float pitch, float distance) {
  int  angle = static_cast<int>(100 * pitch + 0.5f);

  if (angle < 0) {
    angle += CIRCLE;
  } else if (angle >= CIRCLE) {
    angle -= CIRCLE;
  }

  float value =  mSinAllAngleH[angle] / distance;
  if(value < -1.0 || value > 1.0)
    return 0;
  int index = int(value * HALF_PAI_ANGLE) + HALF_PAI_ANGLE;
  if(index < 0 || index > PAI_ANGLE -1)
    return 0;
  return -mArcSin[index];
}

