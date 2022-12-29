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
#include "velodyne_decoder/types.h"

namespace velodyne_decoder {

struct Config {
  // PacketDecoder params
  float min_range = 0.1; ///< minimum range to publish (m)
  float max_range = 200; ///< maximum range to publish (m)
  float min_angle = 0;   ///< minimum angle to publish (deg)
  float max_angle = 360; ///< maximum angle to publish (deg)

  std::optional<ModelId>
      model; ///< model ID, optional for most models (exceptions: HDL-64E, VLP-32A, VLP-32B)
  std::optional<Calibration> calibration; ///< calibration info, optional

  // ScanDecoder params
  bool timestamp_first_packet = false; ///< whether we are timestamping based on
                                       ///< the first or last packet in the scan
  bool gps_time = false;               ///< true: use the packet's time field,
                                       ///< false: use the time of arrival

  static const std::vector<ModelId> SUPPORTED_MODELS;
  static const std::vector<ModelId> TIMINGS_AVAILABLE;
};

inline const std::vector<ModelId> Config::SUPPORTED_MODELS = //
    {ModelId::HDL64E_S1, ModelId::HDL64E_S2, ModelId::HDL64E_S3, ModelId::HDL32E,
     ModelId::VLP32A,    ModelId::VLP32B,    ModelId::VLP32C,    ModelId::VLP16,
     ModelId::PuckHiRes, ModelId::AlphaPrime};

inline const std::vector<ModelId> Config::TIMINGS_AVAILABLE = //
    {ModelId::HDL32E, ModelId::VLP16, ModelId::VLP32C, ModelId::AlphaPrime};

} // namespace velodyne_decoder
