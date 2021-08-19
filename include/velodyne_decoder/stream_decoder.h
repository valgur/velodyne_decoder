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

  static int calc_packets_per_scan(const std::string &model, double rpm);

protected:
  Config config_;
  ScanDecoder scan_decoder_;
  int packets_per_scan_;
  std::vector<VelodynePacket> scan_packets_;
};

} // namespace velodyne_decoder