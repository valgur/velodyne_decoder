// Copyright (c) 2021-2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#include "velodyne_decoder/scan_decoder.h"

namespace velodyne_decoder {

ScanDecoder::ScanDecoder(const Config &config) : packet_decoder_(config) {}

PointCloud ScanDecoder::decode(TimePair scan_stamp,
                               const std::vector<VelodynePacket> &scan_packets) {
  cloud_.clear();
  for (const auto &packet : scan_packets) {
    packet_decoder_.unpack(packet, cloud_, scan_stamp);
  }
  return cloud_;
}

PointCloud ScanDecoder::decode(Time scan_host_stamp,
                               const std::vector<VelodynePacket> &scan_packets) {
  // The header stamp in a VelodyneScan message contains only the host time.
  // Need to figure out whether it corresponds to the first or last packet and get the
  // device time from it.
  // TODO: apply timestamp_first_packet config option in ScanDecoder instead?
  double front_delta = std::abs(scan_packets.front().stamp.host - scan_host_stamp);
  double back_delta  = std::abs(scan_packets.back().stamp.host - scan_host_stamp);
  TimePair scan_stamp =
      front_delta < back_delta ? scan_packets.front().stamp : scan_packets.back().stamp;
  return decode(scan_stamp, scan_packets);
}

inline PointCloud ScanDecoder::decode(const VelodyneScan &scan) {
  return decode(scan.stamp, scan.packets);
}

std::optional<ModelId> ScanDecoder::modelId() const { return packet_decoder_.modelId(); }

std::optional<DualReturnMode> ScanDecoder::returnMode() const {
  return packet_decoder_.returnMode();
}

} // namespace velodyne_decoder
