// Copyright (c) 2021, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "velodyne_decoder/types.h"

#include <string>

namespace velodyne_decoder {

class PointCloudAggregator {
public:
  PointCloudAggregator(float max_range, float min_range, int scans_per_packet)
      : max_range(max_range), min_range(min_range), scans_per_packet(scans_per_packet) {}

  float max_range;
  float min_range;
  int scans_per_packet;

  virtual void init(const std::vector<VelodynePacket> &scan_packets) {
    cloud.clear();
    cloud.reserve(scan_packets.size() * scans_per_packet);
  }

  void newLine() {}

  void addPoint(float x, float y, float z, uint16_t ring, uint16_t /*azimuth*/, float distance,
                float intensity, float time) {
    if (pointInRange(distance)) {
      cloud.emplace_back(PointXYZIRT{x, y, z, intensity, ring, time});
    }
  }

  constexpr bool pointInRange(float range) const {
    return range >= min_range && range <= max_range;
  }

  PointCloud cloud;
};

} /* namespace velodyne_decoder */
