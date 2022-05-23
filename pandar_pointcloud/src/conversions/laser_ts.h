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
#define PAI_ANGLE (180000)
#endif

#ifndef HALF_PAI_ANGLE
#define HALF_PAI_ANGLE (90000)
#endif
#define PANDAR80_LIDAR_NUM (80)
#define PANDAR128_LIDAR_NUM (128)

class LasersTSOffset {
  public:
    LasersTSOffset();
    ~LasersTSOffset();

    void  setFilePath(std::string sFile);
    int ParserFiretimeData(std::string firetimeContent);
    float getTSOffset(int nLaser, int nMode, int nState, float fDistance, int nMajorVersion);
    int   getBlockTS(int nBlock, int nRetMode, int nMode, int nLaserNum);
    float getAngleOffset(float nTSOffset, int speed, int nMajorVersion);
    float getAzimuthOffset(std::string type, float azimuth, float originAzimuth, float distance);
    float getPitchOffset(std::string type, float pitch, float distance);

  private:
    float mFDist;
    bool mBInitFlag;
    std::vector<std::vector<int>> mVLasers;
    int mNLaserNum;
    std::vector<int> mShortOffsetIndex;
    std::vector<int> mLongOffsetIndex;
    float mCosAllAngle[CIRCLE];
    float mSinAllAngleHB[CIRCLE];
    float mSinAllAngleH[CIRCLE];
    float mArcSin[PAI_ANGLE];
    float m_fAzimuthOffset[PANDAR128_LIDAR_NUM];
    float m_fCDAAzimuthOffset[PANDAR128_LIDAR_NUM];
    float m_fCDBAzimuthOffset[PANDAR128_LIDAR_NUM];
    float m_fArctanHB;
    std::array<std::array<float, PANDAR128_LIDAR_NUM>, 4> m_vQT128Firetime;

    void fillVector(char *pContent, int nLen, std::vector<int> &vec);
    float atanAngle(float value);
    float asinAngle(float value);
};

#endif  // ASER_TS_H_

