// Copyright (c) 2021, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "velodyne_decoder/calibration.h"

namespace velodyne_decoder {

struct Config {
  // PacketDecoder params
  float min_range = 0.1; ///< minimum range to publish
  float max_range = 200; ///< maximum range to publish
  float min_angle = 0;   ///< minimum angle to publish
  float max_angle = 360; ///< maximum angle to publish
  std::string model;
  std::optional<Calibration> calibration; ///< calibration info, optional

  // ScanDecoder params
  bool timestamp_first_packet = false; ///< whether we are timestamping based on
                                       ///< the first or last packet in the scan
  bool gps_time = false;               ///< true: use the packet's time field,
                                       ///< false: use the time of arrival

  /// Takes care of model name aliases and deprecations
  static std::string standardizeModelId(const std::string &model) {
    if (model == "VLS-128") // deprecated in favor of "Alpha Prime"
      return "Alpha Prime";
    if (model == "HDL-64E_S2.1")
      return "HDL-64E_S2";
    return model;
  }

  static const std::vector<std::string> SUPPORTED_MODELS;
  static const std::vector<std::string> TIMINGS_AVAILABLE;
};

inline const std::vector<std::string> Config::SUPPORTED_MODELS = //
    {"HDL-32E", "HDL-64E", "HDL-64E_S2", "HDL-64E_S3", "VLP-16", "VLP-32C", "Alpha Prime"};

inline const std::vector<std::string> Config::TIMINGS_AVAILABLE = //
    {"HDL-32E", "VLP-16", "VLP-32C", "Alpha Prime"};

} // namespace velodyne_decoder
