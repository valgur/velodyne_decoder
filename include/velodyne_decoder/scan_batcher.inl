// Copyright (c) 2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#include "velodyne_decoder/config.h"
#include "velodyne_decoder/time_conversion.h"
#include "velodyne_decoder/types.h"

#include <cstring>
#include <memory>
#include <optional>
#include <vector>

namespace velodyne_decoder {
Time getPacketTime(const VelodynePacket &packet) { return packet.stamp; }

int getPacketAzimuth(gsl::span<const uint8_t> data) {
  // get the azimuth of the last block
  uint16_t azimuth = 0;
  memcpy(&azimuth, &data[(BLOCKS_PER_PACKET - 1) * SIZE_BLOCK + 2], sizeof(azimuth));
  return (int)azimuth;
}

template <typename T>
ScanBatcher<T>::ScanBatcher(const Config &config)
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

template <typename T> bool ScanBatcher<T>::push(const T &packet) {
  Time stamp = getPacketTime(packet);
  if (use_device_time_) {
    uint32_t usec_since_toh = parseUint32(&packet.data[PACKET_SIZE - 6]);
    stamp                   = getPacketTimestamp(usec_since_toh, stamp);
  }

  if (scan_complete_) {
    reset();
  }

  int azimuth = getPacketAzimuth(packet.data);
  if (initial_azimuth_ < 0) {
    initial_azimuth_ = azimuth;
  }

  double duration = empty() ? 0.0 : stamp - getPacketTime(scan_packets_->front());
  if (duration > duration_threshold_) {
    kept_last_packet_ = packet;
    scan_complete_    = true;
    return true;
  }

  scan_packets_->push_back(packet);

  const int MAX_ANGLE = 36000;
  int new_coverage    = (azimuth - initial_azimuth_ + MAX_ANGLE) % MAX_ANGLE;
  scan_complete_      = coverage_ > MAX_ANGLE / 2 && coverage_ > new_coverage;
  coverage_           = new_coverage;
  return scan_complete_;
}

template <typename T> bool ScanBatcher<T>::empty() const { return scan_packets_->empty(); }

template <typename T> size_t ScanBatcher<T>::size() const { return scan_packets_->size(); }

template <typename T> bool ScanBatcher<T>::scanComplete() const { return scan_complete_; }

template <typename T> Time ScanBatcher<T>::scanTimestamp() const {
  if (empty())
    return {};
  return timestamp_first_packet_ ? getPacketTime(scan_packets_->front())
                                 : getPacketTime(scan_packets_->back());
}

template <typename T> const std::shared_ptr<std::vector<T>> &ScanBatcher<T>::scanPackets() const {
  return scan_packets_;
}

template <typename T> void ScanBatcher<T>::reset(std::shared_ptr<std::vector<T>> scan_packets) {
  if (cut_angle_ < 0) {
    initial_azimuth_ = empty() ? -1 : getPacketAzimuth(scan_packets_->back().data);
  } else {
    initial_azimuth_ = cut_angle_;
  }
  scan_packets->clear();
  scan_packets_  = std::move(scan_packets);
  scan_complete_ = false;
  coverage_      = 0;
  if (kept_last_packet_) {
    push(std::move(*kept_last_packet_));
    kept_last_packet_ = std::nullopt;
  }
}

template <typename T> void ScanBatcher<T>::reset() { reset(std::make_shared<std::vector<T>>()); }

} // namespace velodyne_decoder
