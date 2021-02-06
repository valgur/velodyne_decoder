// Copyright (C) 2012, 2019 Austin Robot Technology, Jack O'Quin, Joshua Whitley, Sebastian Pütz
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
