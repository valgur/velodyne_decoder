// Copyright (C) 2012, 2019 Austin Robot Technology, Piyush Khandelwal, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace velodyne_decoder {

/** \brief correction values for a single laser
 *
 * Correction values for a single laser (as provided by db.xml from
 * Velodyne).  Includes parameters for Velodyne HDL-64E S2.1.
 *
 * http://velodynelidar.com/lidar/products/manual/63-HDL64E%20S2%20Manual_Rev%20D_2011_web.pdf
 */

/** \brief Correction information for a single laser. */
struct LaserCorrection {
  /** parameters in db.xml */
  float rot_correction             = 0;
  float vert_correction            = 0;
  float dist_correction            = 0;
  bool two_pt_correction_available = false;
  float dist_correction_x          = 0;
  float dist_correction_y          = 0;
  float vert_offset_correction     = 0;
  float horiz_offset_correction    = 0;
  int max_intensity                = 255;
  int min_intensity                = 0;
  float focal_distance             = 0;
  float focal_slope                = 0;

  /** cached values calculated when the calibration file is read */
  float cos_rot_correction  = 1; ///< cosine of rot_correction
  float sin_rot_correction  = 0; ///< sine of rot_correction
  float cos_vert_correction = 1; ///< cosine of vert_correction
  float sin_vert_correction = 0; ///< sine of vert_correction

  uint16_t laser_ring = -1; ///< ring number for this laser
};

/** \brief Calibration information for the entire device. */
class Calibration {
public:
  float distance_resolution_m = 0.002f;
  std::vector<LaserCorrection> laser_corrections;
  int num_lasers            = 0;
  bool initialized          = false;
  bool advanced_calibration = false;

public:
  Calibration() = default;

  explicit Calibration(const std::string &calibration_file) { read(calibration_file); }

  Calibration(std::vector<LaserCorrection> laser_corrs, float distance_resolution_m)
      : distance_resolution_m(distance_resolution_m), laser_corrections(std::move(laser_corrs)) {
    num_lasers           = (int)laser_corrections.size();
    initialized          = true;
    advanced_calibration = isAdvancedCalibration();
  }

  void read(const std::string &calibration_file);
  void write(const std::string &calibration_file);

private:
  bool isAdvancedCalibration();
};

class CalibDB {
public:
  CalibDB();

  Calibration getDefaultCalibration(const std::string &model_id) const;
  std::vector<std::string> getAvailableModels() const;

  const std::unordered_map<std::string, Calibration> &getAllDefaultCalibrations() const {
    return calibrations_;
  };

private:
  std::unordered_map<std::string, Calibration> calibrations_;
};

} // namespace velodyne_decoder
