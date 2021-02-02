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

#ifndef LASER_TS_H_
#define LASER_TS_H_

#include <map>
#include <vector>

#ifndef CIRCLE
#define CIRCLE (36000)
#endif

#ifndef PAI_ANGLE
#define PAI_ANGLE (18001)
#endif

#ifndef HALF_PAI_ANGLE
#define HALF_PAI_ANGLE (9001)
#endif
#define PANDAR80_LIDAR_NUM (80)

class LasersTSOffset {
  public:
    LasersTSOffset();
    ~LasersTSOffset();

    void  setFilePath(std::string sFile);
    int   getTSOffset(int nLaser, int nMode, int nState, float fDistance, int nLaserNum);
    int   getBlockTS(int nBlock, int nRetMode, int nMode, int nLaserNum);
    float getAngleOffset(int nTSOffset, int nLaserId, int nLaserNum);
    float getAzimuthOffset(std::string type, float azimuth, \
        float originAzimuth, float distance);
    float getPitchOffset(std::string type, float pitch, float distance);

  private:
    float                              mFDist;
    bool                               mBInitFlag;
    std::vector<std::vector<int>>      mVLasers;
    int                                mNLaserNum;
    std::map<std::pair<int, int>, int> mShortOffsetIndex;
    std::map<std::pair<int, int>, int> mLongOffsetIndex;
    float                              mCosAllAngle[CIRCLE];
    float                              mSinAllAngle[CIRCLE];
    float                              mSinPAIAngle[PAI_ANGLE];
    float                              mTanPAIAngle[HALF_PAI_ANGLE];
    float                              m_fAzimuthOffset[PANDAR80_LIDAR_NUM];

    void fillVector(char *pContent, int nLen, std::vector<int> &vec);
    float atanAngle(float value);
    float asinAngle(float value);
};

#endif  // ASER_TS_H_
