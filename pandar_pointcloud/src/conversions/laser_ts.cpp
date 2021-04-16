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

#define OFFSET1   (3148)
#define OFFSET2   (-27778)
#define PAI       (3.14159265358979323846)
#define SEC_TO_NS (1000000000.0)
#define SPEED     (3600.0 / SEC_TO_NS)
#define A_TO_R    (PAI / 180.0)
#define R_TO_A    (180.0 / PAI)

#define PANDAR80_BLOCK_TIMESTAMP (33.33)
#define BLOCK_ID_MAX (3)
#define SEC_TO_US (1000000.0)
#define SPEED_US  (3600.0 / SEC_TO_US)

LasersTSOffset::LasersTSOffset() {
  mBInitFlag = false;
  mNLaserNum = 0;

  for (int j = 0; j < CIRCLE; j++) {
    float angle = static_cast<float>(j) / 100.0f;

    mCosAllAngle[j] = cosf(A_TO_R * angle);
    mSinAllAngle[j] = sinf(A_TO_R * angle);
  }

  for (int j = 0; j < PAI_ANGLE; j++) {
    float angle = static_cast<float>(j) / 100.0f - 90.0f;
    mSinPAIAngle[j] = sinf(angle * A_TO_R);
  }

  for (int j = 0; j < HALF_PAI_ANGLE - 1; j++) {
    float angle = static_cast<float>(j) / 100.0f;
    mTanPAIAngle[j] = tanf(angle * A_TO_R);
  }

  mTanPAIAngle[HALF_PAI_ANGLE-1] = mTanPAIAngle[HALF_PAI_ANGLE-2];
  mShortOffsetIndex.resize(100);
  mLongOffsetIndex.resize(100);
}

LasersTSOffset::~LasersTSOffset() {

}

void LasersTSOffset::setFilePath(std::string file) {

  std::ifstream fin(file);
  std::string line;
  if (std::getline(fin, line)) { //first line sequence,chn id,firetime/us
    printf("Parse Lidar firetime now...\n");
  }
  if (line == "sequence,chn id,firetime/us"){
    printf("Parse P80 Lidar firetime now...\n");
    while (std::getline(fin, line)) {
      int sequence = 0;
      int chnId = 0;
      float firetime;
      std::stringstream ss(line);
      std::string subline;
      std::getline(ss, subline, ',');
      std::stringstream(subline) >> sequence;
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

int LasersTSOffset::getTSOffset(int nLaser, int nMode, int nState, float fDistance, int nLaserNum) {
  switch (nLaserNum){
    case PANDAR80_LIDAR_NUM:
      return m_fAzimuthOffset[nLaser];
    default:
      if (nLaser >= mNLaserNum || !mBInitFlag) {
        return 0;
      }
      if (fDistance >= mFDist) {
        return mVLasers[nLaser][mLongOffsetIndex[nMode * 10 + nState]];
      } 
      else {
        return mVLasers[nLaser][mShortOffsetIndex[nMode * 10 + nState]];
      }
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

float LasersTSOffset::getAngleOffset(int nTSOffset, int speed) {
    return static_cast<float>(nTSOffset) * speed * 6E-9;
}


float LasersTSOffset::getAzimuthOffset(std::string type, float azimuth, \
    float originAzimuth, float distance) {
  int   a     = -1;
  float b     = 0.012f;
  float h     = 0.04f;
  float value = b / h;
  int   angle = static_cast<int>(100 * (atanAngle(value) + \
      (azimuth - originAzimuth)) + 0.5f);

  if (angle < 0) {
    angle += CIRCLE;
  } else if (angle >= CIRCLE) {
    angle -= CIRCLE;
  }

  if (distance < 0.00001) {
    return 0;
  }

  value = sqrtf(b * b  + h * h) / distance * mSinAllAngle[angle];

  return a * asinAngle(value);
}

float LasersTSOffset::getPitchOffset(std::string type, float pitch, float distance) {
  int   a     = -1;
  float b     = 0.012f;
  float h     = 0.04;
  int   angle = static_cast<int>(100 * pitch + 0.5f);

  if (angle < 0) {
    angle += CIRCLE;
  } else if (angle >= CIRCLE) {
    angle -= CIRCLE;
  }

  float value = h / distance * mSinAllAngle[angle];

  return a * asinAngle(value);
}

float LasersTSOffset::atanAngle(float value) {
  int i = 0;
  int j = HALF_PAI_ANGLE - 1;

  if (value < mTanPAIAngle[0]) {
    return 0.0f;
  } else if (value > mTanPAIAngle[HALF_PAI_ANGLE-1]) {
    return 90.0f;
  }

  while (i < j - 2) {
    int mid = (i + j) / 2;
    if (mTanPAIAngle[mid] < value) {
      i = mid;
      mid = (i + j) / 2;
    } else if (mTanPAIAngle[mid] > value) {
      j = mid;
      mid = (i + j) / 2;
    } else {
      return (mid / 100.0f);
    }
  }

  return (i + j) / 2 / 100.0f;
}

float LasersTSOffset::asinAngle(float value) {
  int i = 0;
  int j = PAI_ANGLE - 1;

  if (value < mSinPAIAngle[0]) {
    return -90.0f;
  } else if (value > mSinPAIAngle[PAI_ANGLE-1]) {
    return 90.0f;
  }

  while (i < j - 2) {
    int mid = (i + j) / 2;
    if (mSinPAIAngle[mid] < value) {
      i = mid;
      mid = (i + j) / 2;
    } else if (mSinPAIAngle[mid] > value) {
      j = mid;
      mid = (i + j) / 2;
    } else {
      return (mid / 100.0f - 90.0f);
    }
  }

  return (i + j) / 2 / 100.0f - 90.0f;
}
