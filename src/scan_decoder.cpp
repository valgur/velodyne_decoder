// Copyright (c) 2021, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#include "velodyne_decoder/scan_decoder.h"

namespace velodyne_decoder {

ScanDecoder::ScanDecoder(const Config &config)
    : packet_decoder_(config),
      cloud_aggregator_(config.max_range, config.min_range, packet_decoder_.scansPerPacket()) {}

PointCloud ScanDecoder::decode(Time scan_stamp, const std::vector<VelodynePacket> &scan_packets) {
  cloud_aggregator_.init(scan_packets);
  for (const auto &packet : scan_packets) {
    packet_decoder_.unpack(packet, cloud_aggregator_, scan_stamp);
  }
  return cloud_aggregator_.cloud;
}

inline PointCloud ScanDecoder::decode(const VelodyneScan &scan) {
  return decode(scan.stamp, scan.packets);
}

} // namespace velodyne_decoder
