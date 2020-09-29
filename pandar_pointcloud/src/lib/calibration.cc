/**
 * \file  calibration.cc
 * \brief
 *
 * \author  Piyush Khandelwal (piyushk@cs.utexas.edu)
 * \author Yang Sheng
 * Copyright (C) 2012, Austin Robot Technology,
 *                     The University of Texas at Austin
 *  Copyright (c) 2017 Hesai Photonics Technology, Yang Sheng
 *
 * License: Modified BSD License
 *
 * $ Id: 02/14/2012 11:36:36 AM piyushk $
 */

#include <iostream>
#include <fstream>
#include <string>
#include <limits>

#include <ros/ros.h>
#include <pandar_pointcloud/calibration.h>
#include <boost/filesystem.hpp>
#include <angles/angles.h>

namespace pandar_pointcloud
{

// elevation angle of each line for HS Line 40 Lidar, Line 1 - Line 40
static const float hesai_elev_angle_map[] = {
    6.96, 5.976, 4.988, 3.996,
    2.999, 2.001, 1.667, 1.333,
    1.001, 0.667, 0.333, 0,
    -0.334, -0.667, -1.001, -1.334,
    -1.667, -2.001, -2.331, -2.667,
    -3, -3.327, -3.663, -3.996,
    -4.321, -4.657, -4.986, -5.311,
    -5.647, -5.974, -6.957, -7.934,
    -8.908, -9.871, -10.826, -11.772,
    -12.705, -13.63, -14.543, -15.444
};

// Line 40 Lidar azimuth Horizatal offset ,  Line 1 - Line 40
static const float hesai_horizatal_azimuth_offset_map[] = {
    0.005, 0.006, 0.006, 0.006,
    -2.479, -2.479, 2.491, -4.953,
    -2.479, 2.492, -4.953, -2.479,
    2.492, -4.953, 0.007, 2.491,
    -4.953, 0.006, 4.961, -2.479,
    0.006, 4.96, -2.478, 0.006,
    4.958, -2.478, 2.488, 4.956,
    -2.477, 2.487, 2.485, 2.483,
    0.004, 0.004, 0.003, 0.003,
    -2.466, -2.463, -2.46, -2.457
};

void Calibration::setDefaultCorrections ()
{
    for (int i = 0; i < laser_count; i++)
    {
        laser_corrections[i].azimuthCorrection = hesai_horizatal_azimuth_offset_map[i];
        laser_corrections[i].distanceCorrection = 0.0;
        laser_corrections[i].horizontalOffsetCorrection = 0.0;
        laser_corrections[i].verticalOffsetCorrection = 0.0;
        laser_corrections[i].verticalCorrection = hesai_elev_angle_map[i];
        laser_corrections[i].sinVertCorrection = std::sin (hesai_elev_angle_map[i] / 180.0 * M_PI);
        laser_corrections[i].cosVertCorrection = std::cos (hesai_elev_angle_map[i] / 180.0 * M_PI);
    }
}

void Calibration::read(const std::string& calibration_file) 
{
	initialized = true;
	num_lasers = laser_count;
    setDefaultCorrections ();
    if (calibration_file.empty ())
        return;

    boost::filesystem::path file_path(calibration_file);
    if (!boost::filesystem::is_regular(file_path)) {
        file_path = boost::filesystem::path("pandar40.csv");
        if (!boost::filesystem::is_regular(file_path))
            return;
    }

    std::ifstream ifs(file_path.string());
    if (!ifs.is_open())
        return;

    std::string line;
    if (std::getline(ifs, line)) {	// first line "Laser id,Elevation,Azimuth"
        std::cout << "parsing correction file." << std::endl;
    }

    int line_cnt = 0;
    while (std::getline(ifs, line)) {
        if (line_cnt++ >= 40)
            break;

        int line_id = 0;
        double elev, azimuth;

        std::stringstream ss(line);
        std::string subline;
        std::getline(ss, subline, ',');
        std::stringstream(subline) >> line_id;
        std::getline(ss, subline, ',');
        std::stringstream(subline) >> elev;
        std::getline(ss, subline, ',');
        std::stringstream(subline) >> azimuth;
        line_id--;
        laser_corrections[line_id].azimuthCorrection = azimuth;
        laser_corrections[line_id].distanceCorrection = 0.0;
        laser_corrections[line_id].horizontalOffsetCorrection = 0.0;
        laser_corrections[line_id].verticalOffsetCorrection = 0.0;
        laser_corrections[line_id].verticalCorrection = elev;
        laser_corrections[line_id].sinVertCorrection = std::sin (angles::from_degrees(elev));
        laser_corrections[line_id].cosVertCorrection = std::cos (angles::from_degrees(elev));
    }
}

void Calibration::write(const std::string& calibration_file) {
}

} /* pandar_pointcloud */
