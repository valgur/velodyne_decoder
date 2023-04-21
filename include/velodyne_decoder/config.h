// Copyright (c) 2021, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

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

  /// model ID, optional for most models (exceptions: HDL-64E, VLP-32A, VLP-32B)
  std::optional<ModelId> model;
  /// calibration info, optional
  std::optional<Calibration> calibration;

  // ScanBatcher params

  /// Azimuth at which to start a new scan (deg).
  /// If unset, the scan is split whenever it covers >= 360 deg at an arbitrary azimuth.
  std::optional<float> cut_angle = std::nullopt;
  /// whether we are timestamping based on the first or last packet in the scan
  bool timestamp_first_packet = false;
  /// true: use the packet's time field, false: use the time of arrival
  bool use_device_time = false;

  static const std::vector<ModelId> SUPPORTED_MODELS;
};

inline const std::vector<ModelId> Config::SUPPORTED_MODELS = //
    {ModelId::HDL64E_S1, ModelId::HDL64E_S2, ModelId::HDL64E_S3, ModelId::HDL32E,
     ModelId::VLP32A,    ModelId::VLP32B,    ModelId::VLP32C,    ModelId::VLP16,
     ModelId::PuckHiRes, ModelId::AlphaPrime};

} // namespace velodyne_decoder
