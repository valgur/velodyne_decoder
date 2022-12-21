// Copyright (c) 2021, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <array>
#include <optional>

#include "velodyne_decoder/config.h"
#include "velodyne_decoder/scan_decoder.h"
#include "velodyne_decoder/types.h"

namespace velodyne_decoder {

class StreamDecoder {
public:
  explicit StreamDecoder(const Config &config);

  std::optional<std::pair<Time, PointCloud>> decode(Time stamp, const RawPacketData &packet);
  std::optional<std::pair<Time, PointCloud>> decode(const VelodynePacket &packet);

protected:
  Config config_;
  ScanDecoder scan_decoder_;
  std::vector<VelodynePacket> scan_packets_;
  int initial_azimuth_ = -1;
  int prev_coverage_   = 0;
};

} // namespace velodyne_decoder