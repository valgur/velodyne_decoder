// Copyright (c) 2012, 2019, Austin Robot Technology, Piyush Khandelwal, Joshua Whitley
// Copyright (c) 2021-2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "velodyne_decoder/types.h"

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

  uint16_t laser_idx  = -1; ///< index of the laser in the measurements block of a packet
  uint16_t laser_ring = -1; ///< ring number for this laser
};

/** \brief Calibration information for the entire device. */
class Calibration {
public:
  float distance_resolution_m = 0.002f;
  std::vector<LaserCorrection> laser_corrections;
  int num_lasers = 0;

public:
  Calibration() = default;
  explicit Calibration(const std::string &calibration_file);
  Calibration(std::vector<LaserCorrection> laser_corrs, float distance_resolution_m);

  bool isAdvancedCalibration() const;

  static Calibration read(const std::string &calibration_file);
  void write(const std::string &calibration_file) const;

  static Calibration fromString(const std::string &calibration_content);
  std::string toString() const;

private:
  void assignRingNumbers();
};

class CalibDB {
public:
  CalibDB();

  Calibration getDefaultCalibration(ModelId model_id) const;
  std::vector<ModelId> getAvailableModels() const;

  const std::unordered_map<ModelId, Calibration> &getAllDefaultCalibrations() const {
    return calibrations_;
  };

private:
  std::unordered_map<ModelId, Calibration> calibrations_;
};

} // namespace velodyne_decoder
