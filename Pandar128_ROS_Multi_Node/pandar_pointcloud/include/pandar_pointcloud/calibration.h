/**
 * \file  calibration.h
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * Copyright (C) 2012, Austin Robot Technology, University of Texas at Austin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *
 * License: Modified BSD License
 *
 * $ Id: 02/14/2012 11:25:34 AM piyushk $
 */

#ifndef __PANDAR_CALIBRATION_H
#define __PANDAR_CALIBRATION_H

#include <map>
#include <string>

namespace pandar_pointcloud {

/** \brief Correction information for a single laser. */
struct PandarLaserCorrection
{
    double azimuthCorrection;
    double verticalCorrection;
    double distanceCorrection;
    double verticalOffsetCorrection;
    double horizontalOffsetCorrection;
    double sinVertCorrection;
    double cosVertCorrection;
    double sinVertOffsetCorrection;
    double cosVertOffsetCorrection;
};

/** \brief Calibration information for the entire device. */
class Calibration {

public:
	static const int laser_count = 40;
	PandarLaserCorrection laser_corrections[laser_count];
    int num_lasers;
    bool initialized;

public:

    Calibration(): initialized(false)
		{}
    Calibration(const std::string& calibration_file)
    {
        read(calibration_file);
    }

    void read(const std::string& calibration_file);
    void write(const std::string& calibration_file);

private:
    void setDefaultCorrections ();
};

} /* pandar_pointcloud */


#endif /* end of include guard: __PANDAR_CALIBRATION_H */


