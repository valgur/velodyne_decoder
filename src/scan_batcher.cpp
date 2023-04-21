// Copyright (c) 2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#include "velodyne_decoder/scan_batcher.h"
#include "velodyne_decoder/time_conversion.h"

#include <cmath>
#include <cstring>
#include <memory>

namespace velodyne_decoder {

int getPacketAzimuth(const RawPacketData &data) {
  // get the azimuth of the last block
  uint16_t azimuth = 0;
  memcpy(&azimuth, &data[(BLOCKS_PER_PACKET - 1) * SIZE_BLOCK + 2], sizeof(azimuth));
  return (int)azimuth;
}

template <typename PacketT>
ScanBatcher<PacketT>::ScanBatcher(const Config &config)
    : timestamp_first_packet_(config.timestamp_first_packet),
      use_device_time_(config.use_device_time &&
                       !(config.model.has_value() && config.model == ModelId::HDL64E_S1)),
      cut_angle_(!config.cut_angle || config.cut_angle < 0
                     ? -1
                     : (int)(std::lround(*config.cut_angle * 100)) % 36000) {
  if (cut_angle_ >= 0) {
    initial_azimuth_ = cut_angle_;
  }
}

template <typename PacketT> Time ScanBatcher<PacketT>::scanTimestamp() const {
  if (empty()) {
    return {};
  }
  return timestamp_first_packet_ ? scan_packets_->front().stamp : scan_packets_->back().stamp;
}

template <typename PacketT>
void ScanBatcher<PacketT>::reset(std::shared_ptr<std::vector<PacketT>> &&scan_packets) {
  if (cut_angle_ < 0) {
    initial_azimuth_ = empty() ? -1 : getPacketAzimuth(scan_packets_->back().data);
  } else {
    initial_azimuth_ = cut_angle_;
  }
  scan_packets->clear();
  scan_packets_  = scan_packets;
  scan_complete_ = false;
  coverage_      = 0;
  if (kept_last_packet_) {
    push(kept_last_packet_->stamp, kept_last_packet_->data);
    kept_last_packet_ = std::nullopt;
  }
}

template <typename PacketT>
bool ScanBatcher<PacketT>::push(Time stamp, const RawPacketData &packet) {
  if (use_device_time_) {
    stamp = getPacketTimestamp(&(packet[PACKET_SIZE - 6]), stamp);
  }

  if (scan_complete_) {
    reset();
  }

  int azimuth = getPacketAzimuth(packet);
  if (initial_azimuth_ < 0) {
    initial_azimuth_ = azimuth;
  }

  double duration = empty() ? 0.0 : stamp - scan_packets_->front().stamp;
  if (duration > duration_threshold_) {
    kept_last_packet_ = {stamp, packet};
    scan_complete_    = true;
    return true;
  }

  scan_packets_->emplace_back(stamp, packet);

  const int MAX_ANGLE = 36000;
  int new_coverage    = (azimuth - initial_azimuth_ + MAX_ANGLE) % MAX_ANGLE;
  scan_complete_      = coverage_ > MAX_ANGLE / 2 && coverage_ > new_coverage;
  coverage_           = new_coverage;
  return scan_complete_;
}

template class ScanBatcher<VelodynePacket>;

} // namespace velodyne_decoder