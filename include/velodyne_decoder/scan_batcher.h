// Copyright (c) 2023, Martin Valgur
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "velodyne_decoder/config.h"
#include "velodyne_decoder/types.h"

#include <memory>
#include <optional>
#include <vector>

namespace velodyne_decoder {
template <typename PacketT = VelodynePacket> class ScanBatcher {
public:
  explicit ScanBatcher(const Config &config);

  /** @brief Pushes a packet into the batcher.
   * @return true if the scan is complete, false otherwise.
   */
  bool push(Time stamp, const RawPacketData &packet);

  /** @brief Pushes a packet into the batcher.
   * @return true if the scan is complete, false otherwise.
   */
  bool push(const PacketT &packet) { return push(packet.stamp, packet.data); }

  [[nodiscard]] bool empty() const { return scan_packets_->empty(); }
  [[nodiscard]] size_t size() const { return scan_packets_->size(); }

  /** @brief True if the current scan is complete.
   */
  [[nodiscard]] bool scanComplete() const { return scan_complete_; }

  /** @brief The timestamp of the current scan. Zero if the scan is empty.
   */
  [[nodiscard]] Time scanTimestamp() const;

  /** @brief The contents of the current scan.
   */
  [[nodiscard]] const std::shared_ptr<std::vector<PacketT>> &scanPackets() const {
    return scan_packets_;
  }

  void reset(std::shared_ptr<std::vector<PacketT>> &&scan_packets =
                 std::make_shared<std::vector<PacketT>>());

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
