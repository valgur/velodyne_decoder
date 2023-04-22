// Copyright (c) 2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "velodyne_decoder/config.h"
#include "velodyne_decoder/time_conversion.h"
#include "velodyne_decoder/types.h"

#include <cstring>
#include <memory>
#include <optional>
#include <vector>

namespace velodyne_decoder {

inline Time get_time(const VelodynePacket &packet) { return packet.stamp; }

template <typename RawPacketDataT> inline int getPacketAzimuth(const RawPacketDataT &data) {
  // get the azimuth of the last block
  uint16_t azimuth = 0;
  memcpy(&azimuth, &data[(BLOCKS_PER_PACKET - 1) * SIZE_BLOCK + 2], sizeof(azimuth));
  return (int)azimuth;
}

template <typename PacketT = VelodynePacket> class ScanBatcher {
public:
  inline explicit ScanBatcher(const Config &config)
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

  /** @brief Pushes a packet into the batcher.
   * @return true if the scan is complete, false otherwise.
   */
  inline bool push(const PacketT &packet) {
    Time stamp = get_time(packet);
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

    double duration = empty() ? 0.0 : stamp - get_time(scan_packets_->front());
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

  [[nodiscard]] inline bool empty() const { return scan_packets_->empty(); }
  [[nodiscard]] inline size_t size() const { return scan_packets_->size(); }

  /** @brief True if the current scan is complete.
   */
  [[nodiscard]] inline bool scanComplete() const { return scan_complete_; }

  /** @brief The timestamp of the current scan. Zero if the scan is empty.
   */
  [[nodiscard]] inline Time scanTimestamp() const {
    if (empty())
      return {};
    return timestamp_first_packet_ ? get_time(scan_packets_->front())
                                   : get_time(scan_packets_->back());
  }

  /** @brief The contents of the current scan.
   */
  [[nodiscard]] inline const std::shared_ptr<std::vector<PacketT>> &scanPackets() const {
    return scan_packets_;
  }

  inline void reset(std::shared_ptr<std::vector<PacketT>> scan_packets) {
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

  inline void reset() { reset(std::make_shared<std::vector<PacketT>>()); }

private:
  std::shared_ptr<std::vector<PacketT>> scan_packets_ = std::make_shared<std::vector<PacketT>>();
  int initial_azimuth_                                = -1;
  int coverage_                                       = 0;
  bool scan_complete_                                 = false;
  std::optional<PacketT> kept_last_packet_            = std::nullopt;

  // config
  const bool timestamp_first_packet_;
  const bool use_device_time_;
  const int cut_angle_;
  const double duration_threshold_ = 0.3; // max scan duration at ~4 Hz
};

extern template class ScanBatcher<VelodynePacket>;

} // namespace velodyne_decoder
