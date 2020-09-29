/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011, 2012 Austin Robot Technology
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: data_base.h 1554 2011-06-14 22:11:17Z jack.oquin $
 */

/** \file
 *
 *  Point Cloud Library point structures for Velodyne data.
 *
 *  @author Jesse Vera
 *  @author Jack O'Quin
 *  @author Piyush Khandelwal
 *  @author Yang Sheng
 */

#ifndef __PANDAR_POINTCLOUD_POINT_TYPES_H
#define __PANDAR_POINTCLOUD_POINT_TYPES_H

#include <pcl/point_types.h>

namespace pandar_pointcloud
{
  /** Euclidean Pandar40 coordinate, including intensity and ring number. */
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;


struct PointXYZIT {
    PCL_ADD_POINT4D
    uint8_t intensity;
    double timestamp;
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

struct PointXYZITd {
    double x;
    double y;
    double z;
    uint8_t intensity;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment

struct PointXYZd {
    double x;
    double y;
    double z;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

struct PointXYZRGBd {
    double x;
    double y;
    double z;
    PCL_ADD_RGB
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
}; // namespace pandar_pointcloud





POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_pointcloud::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_pointcloud::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (uint8_t, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_pointcloud::PointXYZITd,
                                  (double, x, x)(double, y, y)(double, z, z)(uint8_t, intensity, intensity)(double, timestamp,
                                          timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_pointcloud::PointXYZd,
                                  (double, x, x)(double, y, y)(double, z, z))
POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_pointcloud::PointXYZRGBd,
                                  (double, x, x)(double, y, y)(double, z, z)(float, rgb, rgb))
#endif // __PANDAR_POINTCLOUD_POINT_TYPES_H

