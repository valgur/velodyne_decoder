/*
 *  Copyright (C) 2007, 2009-2012 Austin Robot Technology, Patrick Beeson, Jack O'Quin
 *  Copyright (C) 2021, Martin Valgur
 *
 *  License: Modified BSD Software License Agreement
 */

#include "velodyne_decoder/stream_decoder.h"

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include "velodyne_decoder/time_conversion.h"

namespace velodyne_decoder {

int getPacketAzimuth(const RawPacketData &data) {
  // get the azimuth of the last block
  uint16_t azimuth = 0;
  memcpy(&azimuth, &data[(BLOCKS_PER_PACKET - 1) * SIZE_BLOCK + 2], sizeof(azimuth));
  return (int)azimuth;
}

StreamDecoder::StreamDecoder(const Config &config) : config_(config), scan_decoder_(config) {
  if (config_.model.has_value() && config_.model == ModelId::HDL64E_S1) {
    config_.gps_time = false;
  }
}

std::optional<std::pair<Time, PointCloud>> //
StreamDecoder::decode(Time stamp, const RawPacketData &packet) {
  if (config_.gps_time) {
    stamp = getPacketTimestamp(&(packet[PACKET_SIZE - 6]), stamp);
  }

  double duration                 = 0.0;
  const double duration_threshold = 0.3; // max scan duration at ~4 Hz
  if (!scan_packets_.empty()) {
    duration = stamp - scan_packets_.front().stamp;
  }
  bool duration_exceeded = duration > duration_threshold;

  int azimuth = getPacketAzimuth(packet);
  if (initial_azimuth_ < 0) {
    initial_azimuth_ = azimuth;
  }

  if (!duration_exceeded) {
    scan_packets_.emplace_back(stamp, packet);
  }

  const int MAX_ANGLE = 36000;
  int scan_coverage   = (azimuth - initial_azimuth_ + MAX_ANGLE) % MAX_ANGLE;
  if ((prev_coverage_ > MAX_ANGLE / 2 && prev_coverage_ > scan_coverage) || duration_exceeded) {
    // decode scan
    auto result = decodeCollectedPackets();
    // reset
    initial_azimuth_ = getPacketAzimuth(scan_packets_.back().data);
    prev_coverage_   = 0;
    scan_packets_.clear();
    if (duration_exceeded) {
      scan_packets_.emplace_back(stamp, packet);
    }
    return result;
  }
  prev_coverage_ = scan_coverage;
  return std::nullopt;
}

std::optional<std::pair<Time, PointCloud>> //
StreamDecoder::decode(const VelodynePacket &packet) {
  return decode(packet.stamp, packet.data);
}

std::pair<Time, PointCloud> StreamDecoder::decodeCollectedPackets() {
  Time scan_stamp;
  if (config_.timestamp_first_packet) {
    scan_stamp = scan_packets_.front().stamp;
  } else {
    scan_stamp = scan_packets_.back().stamp;
  }
  return {scan_stamp, scan_decoder_.decode(scan_stamp, scan_packets_)};
}

std::optional<std::pair<Time, PointCloud>> StreamDecoder::finish() {
  if (scan_packets_.empty()) {
    return std::nullopt;
  }
  return decodeCollectedPackets();
}

} // namespace velodyne_decoder
