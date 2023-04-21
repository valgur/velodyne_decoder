// Copyright (c) 2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#include "velodyne_decoder/stream_decoder.h"

#include <optional>
#include <utility>

namespace velodyne_decoder {

StreamDecoder::StreamDecoder(const Config &config) : scan_batcher_(config), scan_decoder_(config) {}

std::optional<std::pair<Time, PointCloud>> //
StreamDecoder::decode(Time stamp, const RawPacketData &packet) {
  bool scan_complete = scan_batcher_.push(stamp, packet);
  if (scan_complete)
    return decodeCollectedPackets();
  return std::nullopt;
}

std::optional<std::pair<Time, PointCloud>> //
StreamDecoder::decode(const VelodynePacket &packet) {
  return decode(packet.stamp, packet.data);
}

std::pair<Time, PointCloud> StreamDecoder::decodeCollectedPackets() {
  Time scan_stamp = scan_batcher_.scanTimestamp();
  std::pair<Time, PointCloud> result{
      scan_stamp, scan_decoder_.decode(scan_stamp, *scan_batcher_.scanPackets())};
  scan_batcher_.reset();
  return result;
}

std::optional<std::pair<Time, PointCloud>> StreamDecoder::finish() {
  if (scan_batcher_.empty()) {
    return std::nullopt;
  }
  return decodeCollectedPackets();
}

} // namespace velodyne_decoder
