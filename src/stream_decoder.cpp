// Copyright (c) 2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#include "velodyne_decoder/stream_decoder.h"

#include <optional>
#include <utility>

namespace velodyne_decoder {

StreamDecoder::StreamDecoder(const Config &config) : scan_batcher_(config), scan_decoder_(config) {}

std::optional<std::pair<TimePair, PointCloud>> //
StreamDecoder::decode(const VelodynePacket &packet) {
  bool scan_complete = scan_batcher_.push(packet);
  if (scan_complete)
    return decodeCollectedPackets();
  return std::nullopt;
}

std::pair<TimePair, PointCloud> StreamDecoder::decodeCollectedPackets() {
  TimePair scan_stamp = scan_batcher_.scanTimestamp();
  std::pair<TimePair, PointCloud> result{
      scan_stamp, scan_decoder_.decode(scan_stamp, *scan_batcher_.scanPackets())};
  scan_batcher_.reset();
  return result;
}

std::optional<std::pair<TimePair, PointCloud>> StreamDecoder::finish() {
  if (scan_batcher_.empty()) {
    return std::nullopt;
  }
  return decodeCollectedPackets();
}

} // namespace velodyne_decoder
