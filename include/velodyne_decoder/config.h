// Copyright (c) 2021, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace velodyne_decoder {

struct Config {
  // PacketDecoder params
  std::string model;
  std::string calibration_file; ///< calibration file path
  float min_range = 0.1;        ///< minimum range to publish
  float max_range = 200;        ///< maximum range to publish
  int min_angle   = 0;          ///< minimum angle to publish
  int max_angle   = 36000;      ///< maximum angle to publish

  // ScanDecoder params
  double rpm                  = -1;    ///< device rotation rate
  bool timestamp_first_packet = false; ///< whether we are timestamping based on
                                       ///< the first or last packet in the scan
  bool gps_time = false;               ///< true: use the packet's time field,
                                       ///< false: use the time of arrival

  void setMinAngleDeg(double min_angle_) { min_angle = std::lround(min_angle_ * 100); }
  double getMinAngleDeg() const { return static_cast<double>(min_angle) / 100; }
  void setMaxAngleDeg(double max_angle_) { max_angle = std::lround(max_angle_ * 100); }
  double getMaxAngleDeg() const { return static_cast<double>(max_angle) / 100; }

  static const std::vector<std::string> SUPPORTED_MODELS;
  static const std::vector<std::string> TIMINGS_AVAILABLE;
};

inline const std::vector<std::string> Config::SUPPORTED_MODELS = //
    {"HDL-32E", "HDL-64E", "HDL-64E_S2", "HDL-64E_S3", "VLP-16", "VLP-32C", "VLS-128"};

inline const std::vector<std::string> Config::TIMINGS_AVAILABLE = //
    {"HDL-32E", "VLP-16", "VLP-32C", "VLS-128"};

} // namespace velodyne_decoder
