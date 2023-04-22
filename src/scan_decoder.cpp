// Copyright (c) 2021-2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#include "velodyne_decoder/scan_decoder.h"

namespace velodyne_decoder {

ScanDecoder::ScanDecoder(const Config &config) : packet_decoder_(config) {}

PointCloud ScanDecoder::decode(Time scan_stamp, const std::vector<VelodynePacket> &scan_packets) {
  cloud_.clear();
  for (const auto &packet : scan_packets) {
    packet_decoder_.unpack(packet, cloud_, scan_stamp);
  }
  return cloud_;
}

inline PointCloud ScanDecoder::decode(const VelodyneScan &scan) {
  return decode(scan.stamp, scan.packets);
}

} // namespace velodyne_decoder
